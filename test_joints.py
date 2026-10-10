#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What a run says about the joints of a world (see 'Reading the joints' in 'simulate_gazebo.py').

PartCAD's own exporter writes every model free today, so the worlds here are
written by hand.

Most of this needs no Gazebo, for the reason 'test_simulate_gazebo.py' gives:
what the plugin does is read what a program says and turn it into what PartCAD
reads, and the places that is quietly wrong are all on this side of the pipe --

* which models are given the publisher, and under which topic, which decides
  whether a joint is read at all;
* the type, which the messages do not carry and the world does;
* the units: radians to degrees and metres to millimetres, both of which look
  plausible in the wrong direction;
* which message a reading takes, out of a stream that runs alongside the poses
  rather than in step with them;
* and that every joint that is not reported is named in 'warnings'.

The last test is the end-to-end one: a pendulum and a slider in a real Gazebo,
checked against physics with a known answer. It is skipped without a Gazebo,
which CI's runners do not have; run it in the image the simulation uses:

    docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 \\
        --entrypoint bash \\
        ghcr.io/partcad/partcad-sim-gazebo:latest -c \\
        'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest \\
         && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_joints.py'
"""

import math
import os
import sys
import threading
import time
from xml.etree import ElementTree

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate_gazebo as sim  # noqa: E402

# One message as Harmonic's JointStatePublisher prints it, trimmed to three
# joints: a hinge, a slide, and a 'continuous' joint DART built as a fixed one,
# which is what a joint with no 'axis1' at all is. Note what is not there: no
# joint 'type', and no field whose value is zero.
JOINT_MESSAGE = """header {
  stamp {
    sec: 3
    nsec: 500000000
  }
}
name: "rig"
id: 4
pose {
  position {
    z: 2
  }
  orientation {
    w: 1
  }
}
joint {
  name: "swing"
  id: 12
  parent: "base"
  child: "arm"
  pose {
    position {
      x: -0.5
    }
    orientation {
      w: 1
    }
  }
  axis1 {
    xyz {
      y: 1
    }
    limit_lower: -inf
    limit_upper: inf
    position: 1.5707963267948966
    velocity: -6.2639
  }
}
joint {
  name: "drop"
  id: 13
  parent: "base"
  child: "carriage"
  pose {
    position {
    }
    orientation {
      w: 1
    }
  }
  axis1 {
    xyz {
      z: 1
    }
    limit_lower: -100
    limit_upper: 100
    velocity: -0.5
  }
}
joint {
  name: "spin"
  id: 15
  parent: "base"
  child: "wheel"
  pose {
    position {
    }
    orientation {
      w: 1
    }
  }
}
"""


def world(models):
    """An SDFormat world around some '<model>' elements, parsed."""
    return ElementTree.fromstring('<sdf version="1.9"><world name="w">%s</world></sdf>' % models).find("world")


def joint(name, kind, extra=""):
    return '<joint name="%s" type="%s"><parent>a</parent><child>b</child>%s</joint>' % (name, kind, extra)


def model(name, *contents):
    return '<model name="%s"><link name="a"/><link name="b"/>%s</model>' % (name, "".join(contents))


def rig(**joints):
    """The model 'JOINT_MESSAGE' is about, with the joints it names typed as given."""
    return sim.JointModel("rig", "/t/0", {name: (kind, name, declared) for name, (kind, declared) in joints.items()})


#
# Which joints are read, and under which names
#


def test_a_model_with_a_joint_to_read_is_given_a_publisher_on_a_topic_of_this_runs_own():
    element = world(model("rig", joint("swing", "revolute")))

    models = sim.plan_joints(element, "/partcad/42/joint_state", [])

    assert [(m.scoped, m.topic) for m in models] == [("rig", "/partcad/42/joint_state/0")]
    plugin = element.find("model/plugin")
    assert plugin.get("filename") == "gz-sim-joint-state-publisher-system"
    assert plugin.get("name") == "gz::sim::systems::JointStatePublisher"
    assert plugin.findtext("topic") == "/partcad/42/joint_state/0"


def test_a_model_with_nothing_to_read_is_left_as_it_is():
    """A fixed joint moves nothing, and a free model has no joint at all."""
    element = world(model("box") + model("bolted", joint("weld", "fixed")))

    assert sim.plan_joints(element, "/p", []) == []
    assert element.find(".//plugin") is None


def test_nothing_in_a_static_model_is_read_however_deep():
    """The world says none of it moves, so there is nothing to say about it."""
    element = world(
        '<model name="shelf"><static>true</static><link name="a"/><link name="b"/>%s%s</model>'
        % (joint("door", "revolute"), model("drawer", joint("runner", "prismatic")))
    )

    assert sim.plan_joints(element, "/p", []) == []


def test_a_turn_is_revolute_while_something_limits_it_and_continuous_when_nothing_does():
    """The line the MuJoCo plugin draws too, so either engine reads the same type
    for the same joint whichever way its file had to spell it."""
    limit = "<axis><xyz>0 0 1</xyz><limit><lower>%s</lower><upper>%s</upper></limit></axis>"
    element = world(
        model(
            "rig",
            joint("stopped", "revolute", limit % (-1.5, 1.5)),
            joint("free", "revolute", "<axis><xyz>0 0 1</xyz></axis>"),
            joint("defaulted", "revolute", limit % (-1e16, 1e16)),
            joint("declared", "continuous"),
            joint("slide", "prismatic"),
            joint("thread", "screw"),
        )
    )

    (rig_model,) = sim.plan_joints(element, "/p", [])

    assert {name: kind for name, (kind, _reported, _declared) in rig_model.joints.items()} == {
        "stopped": "revolute",
        "free": "continuous",
        "defaulted": "continuous",
        "declared": "continuous",
        "slide": "prismatic",
        "thread": "screw",
    }


def test_a_joint_of_a_type_that_is_not_read_is_named_in_the_warnings():
    warnings = []
    element = world(model("rig", joint("socket", "ball"), joint("cardan", "universal")))

    assert sim.plan_joints(element, "/p", warnings) == []
    assert len(warnings) == 2
    assert "'socket' of model 'rig'" in warnings[0] and "two of a ball joint's three coordinates" in warnings[0]
    assert "'cardan' of model 'rig'" in warnings[1]


def test_a_joint_is_reported_under_its_own_name_unless_another_one_has_it():
    """And then both are scoped, so that neither is the one the plain name means.
    A nested model is scoped the way Gazebo scopes it."""
    element = world(
        model("left", joint("elbow", "revolute"), joint("grip", "prismatic"))
        + model("right", joint("elbow", "revolute"), model("hand", joint("wrist", "revolute")))
    )

    models = sim.plan_joints(element, "/p", [])

    reported = {m.scoped: sorted(r for _kind, r, _declared in m.joints.values()) for m in models}
    assert reported == {
        "left": ["grip", "left::elbow"],
        "right": ["right::elbow"],
        "right::hand": ["wrist"],
    }
    assert [m.topic for m in models] == ["/p/0", "/p/1", "/p/2"]


#
# The world that runs
#


def test_a_world_with_no_joint_to_read_runs_as_it_was_handed_over(tmp_path):
    """Byte for byte: a world of free models -- every world PartCAD exports
    today -- runs exactly as it did before joints were read at all."""
    scene = tmp_path / "scene.world"
    scene.write_text('<sdf version="1.9"><world name="w">%s</world></sdf>' % model("box"), encoding="utf-8")

    assert sim.prepare_world(str(scene), []) == (str(scene), [])
    assert sorted(os.listdir(tmp_path)) == ["scene.world"]


def test_a_world_that_is_not_xml_is_left_for_gazebo_to_refuse(tmp_path):
    """Which says what is wrong with it the way it always has."""
    scene = tmp_path / "scene.world"
    scene.write_text("<sdf><world", encoding="utf-8")

    assert sim.prepare_world(str(scene), []) == (str(scene), [])


def test_a_world_with_joints_runs_as_a_copy_beside_it_and_the_original_is_not_touched(tmp_path):
    """Beside it, because a world names its meshes relative to where it is."""
    scene = tmp_path / "scene.world"
    text = '<sdf version="1.9"><world name="w">%s</world></sdf>' % model("rig", joint("swing", "revolute"))
    scene.write_text(text, encoding="utf-8")

    run_file, models = sim.prepare_world(str(scene), [])

    assert run_file == str(tmp_path / "scene.joint-states.world")
    assert [m.scoped for m in models] == ["rig"]
    assert scene.read_text(encoding="utf-8") == text
    copied = ElementTree.parse(run_file).getroot()
    assert copied.find("world/model/plugin/topic").text == models[0].topic


def test_a_world_that_cannot_be_copied_runs_as_it_is_and_says_why_there_are_no_joints(tmp_path, monkeypatch):
    scene = tmp_path / "scene.world"
    scene.write_text(
        '<sdf version="1.9"><world name="w">%s</world></sdf>' % model("rig", joint("swing", "revolute")),
        encoding="utf-8",
    )

    def refuse(*_args, **_kwargs):
        raise PermissionError("read-only")

    monkeypatch.setattr(ElementTree.ElementTree, "write", refuse)
    warnings = []

    assert sim.prepare_world(str(scene), warnings) == (str(scene), [])
    assert "the joints are not reported" in warnings[0]


#
# Reading a joint-state message
#


def test_a_turn_is_read_in_degrees_and_a_move_in_millimetres():
    """The units of an interface's 'motion:'. Gazebo's are radians and metres."""
    states = sim.joint_states(
        sim.parse_message(JOINT_MESSAGE),
        rig(swing=("continuous", "revolute"), drop=("prismatic", "prismatic")),
        [],
    )

    assert states["swing"]["type"] == "continuous"
    assert states["swing"]["pos"] == pytest.approx(90.0)
    assert states["swing"]["vel"] == pytest.approx(math.degrees(-6.2639))
    assert states["drop"] == {"type": "prismatic", "pos": 0.0, "vel": pytest.approx(-500.0)}


def test_a_coordinate_the_message_omits_reads_as_zero():
    """Protobuf text format prints no field whose value is the default -- which
    for a slide at its starting point is its position."""
    states = sim.joint_states(sim.parse_message(JOINT_MESSAGE), rig(drop=("prismatic", "prismatic")), [])

    assert states["drop"]["pos"] == 0.0


def test_there_is_no_effort_because_gazebo_does_not_publish_one():
    """Not a zero: a zero nobody measured would be read as a measurement."""
    states = sim.joint_states(sim.parse_message(JOINT_MESSAGE), rig(swing=("continuous", "revolute")), [])

    assert "effort" not in states["swing"]


def test_a_joint_gazebo_ran_with_no_freedom_is_named_rather_than_reported_as_still():
    """Which is what Harmonic's DART makes of every 'continuous' joint, and the
    warning says what to write instead."""
    warnings = []

    states = sim.joint_states(sim.parse_message(JOINT_MESSAGE), rig(spin=("continuous", "continuous")), warnings)

    assert states == {}
    assert len(warnings) == 1
    assert "'spin' of model 'rig'" in warnings[0]
    assert "a 'revolute' with no limits" in warnings[0]


def test_a_joint_that_is_not_in_the_message_is_named_too():
    warnings = []

    sim.joint_states(sim.parse_message(JOINT_MESSAGE), rig(missing=("revolute", "revolute")), warnings)

    assert "'missing' of model 'rig'" in warnings[0]


def test_a_warning_is_given_once_however_many_readings_run_into_it():
    warnings = []
    for _ in range(3):
        sim.joint_states(sim.parse_message(JOINT_MESSAGE), rig(spin=("continuous", "continuous")), warnings)

    assert len(warnings) == 1


def test_a_joint_is_reported_under_the_name_it_was_planned_under():
    planned = sim.JointModel("left", "/t/0", {"swing": ("revolute", "left::swing", "revolute")})

    assert list(sim.joint_states(sim.parse_message(JOINT_MESSAGE), planned, [])) == ["left::swing"]


def test_a_reading_of_the_poses_starts_with_no_joints_rather_than_none():
    """So a validation can walk 'after["joints"]' of any world without asking first."""
    assert sim.snapshot(sim.parse_message('header {\n}\npose {\n  name: "box"\n}\n'))["joints"] == {}


#
# Which message a reading takes
#


def stamped(seconds):
    """A joint-state message taken at 'seconds' of simulated time."""
    whole = int(seconds)
    return JOINT_MESSAGE.replace(
        "sec: 3\n    nsec: 500000000", "sec: %d\n    nsec: %d" % (whole, round((seconds - whole) * 1e9))
    )


def test_the_joint_clock_is_the_pose_clock():
    """The same sum of the same two fields, so the two compare equal at one instant."""
    for text in (JOINT_MESSAGE, stamped(0.427), "header {\n}\n", "header {\n  stamp {\n  }\n}\n"):
        assert sim.message_time(text) == sim.snapshot(sim.parse_message(text))["time"]


def stream_of(*seconds):
    stream = sim.JointStream(rig(swing=("continuous", "revolute")))
    stream.follow(iter("".join(stamped(s) for s in seconds).splitlines(keepends=True)))
    return stream


def test_a_reading_takes_the_message_of_its_own_instant():
    stream = stream_of(0.001, 0.002, 0.003)

    assert sim.message_time(stream.at(0.002)) == pytest.approx(0.002)


def test_a_reading_whose_instant_was_missed_takes_the_latest_one_before_it():
    stream = stream_of(0.001, 0.002, 0.004)

    assert sim.message_time(stream.at(0.003)) == pytest.approx(0.002)


def test_a_reading_from_before_the_stream_began_takes_the_first_one_there_is():
    stream = stream_of(0.010, 0.011)

    assert sim.message_time(stream.at(0.002)) == pytest.approx(0.010)


def test_a_reading_waits_for_a_stream_that_is_behind():
    """The joints run alongside the poses, not in step with them."""
    stream = sim.JointStream(rig(swing=("continuous", "revolute")))
    read, write = os.pipe()
    with os.fdopen(read) as reader, os.fdopen(write, "w") as writer:
        follower = threading.Thread(target=stream.follow, args=(reader,), daemon=True)
        follower.start()
        writer.write(stamped(0.001))
        writer.flush()

        def catch_up():
            time.sleep(0.2)
            writer.write(stamped(0.002) + stamped(0.003))
            writer.flush()

        threading.Thread(target=catch_up, daemon=True).start()
        assert sim.message_time(stream.at(0.002, patience=5)) == pytest.approx(0.002)


def test_a_stream_that_says_nothing_is_waited_for_once():
    """A publisher that is not there, rather than one that is slow."""
    stream = sim.JointStream(rig(swing=("continuous", "revolute")))

    started = time.monotonic()
    assert stream.at(1.0, patience=0.2) is None
    assert stream.at(2.0, patience=5) is None
    assert time.monotonic() - started < 2


def test_what_no_reading_can_ask_for_any_more_is_forgotten():
    """Which is everything before the latest message at or before a pose already
    seen: a reading is never asked for at an earlier instant than that."""
    stream = stream_of(0.001, 0.002, 0.003, 0.004)

    stream.forget_before(0.0025)

    assert [stamp for stamp, _text in stream.entries] == pytest.approx([0.002, 0.003, 0.004])
    assert sim.message_time(stream.at(0.0025)) == pytest.approx(0.002)


def test_a_reading_is_filled_from_every_stream_and_a_silent_one_is_named():
    reading = {"time": 3.5, "bodies": {}, "joints": {}}
    silent = sim.JointStream(sim.JointModel("other", "/t/1", {"lift": ("prismatic", "lift", "prismatic")}))
    silent.ended = True
    warnings = []

    sim.fill(reading, [stream_of(3.5), silent], warnings)

    assert list(reading["joints"]) == ["swing"]
    assert warnings == ["the joints of model 'other' are not reported: Gazebo published no joint states for it"]


#
# In a real Gazebo
#

G = 9.81
# A 1 kg pendulum half a metre out along +x from a hinge about +y, written out
# horizontal; a 2 kg carriage on a vertical slide; and two joints that are not
# reported, each for its own reason.
PENDULUM = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="joints">
    <gravity>0 0 -%(g)s</gravity>
    <model name="rig">
      <pose>0 0 2 0 0 0</pose>
      <link name="base">
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
      <joint name="anchor" type="fixed"><parent>world</parent><child>base</child></joint>
      <link name="arm">
        <pose>0.5 0 0 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>4e-5</ixx><iyy>4e-5</iyy><izz>4e-5</izz></inertia></inertial>
      </link>
      <joint name="swing" type="revolute">
        <parent>base</parent><child>arm</child>
        <pose>-0.5 0 0 0 0 0</pose>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>
      <link name="carriage">
        <pose>0 1 0 0 0 0</pose>
        <inertial><mass>2</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
      <joint name="drop" type="prismatic">
        <parent>base</parent><child>carriage</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-100</lower><upper>100</upper></limit></axis>
      </joint>
      <link name="bob">
        <pose>0.5 -1 0 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>4e-5</ixx><iyy>4e-5</iyy><izz>4e-5</izz></inertia></inertial>
      </link>
      <joint name="socket" type="ball"><parent>base</parent><child>bob</child><pose>-0.5 0 0 0 0 0</pose></joint>
      <link name="wheel">
        <pose>0 -2 0 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
      <joint name="spin" type="continuous">
        <parent>base</parent><child>wheel</child><axis><xyz>0 0 1</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>
""" % {"g": G}
INERTIA = 1.0 * 0.5**2 + 4e-5


@pytest.fixture(scope="module")
def gazebo():
    try:
        sim.locate_gazebo()
    except sim.GazeboMissing:
        pytest.skip("no Gazebo here; see this file's docstring for running it in the simulation's image")


@pytest.fixture(scope="module")
def swung(gazebo, tmp_path_factory):
    directory = tmp_path_factory.mktemp("joints")
    (directory / "scene.world").write_text(PENDULUM, encoding="utf-8")
    # A quarter of the pendulum's period at an amplitude of 90 degrees, which is
    # the time it takes to swing down to the bottom.
    quarter = 0.4186
    result = sim.process(
        str(directory), {"scene_file": str(directory / "scene.world"), "duration": quarter, "samples": 3}
    )
    result["directory"] = directory
    return result


def test_in_gazebo_a_pendulum_loses_no_energy_on_the_way_down(swung):
    """At every reading once the swing is under way -- and so in degrees and
    degrees per second, since in any other units this does not hold.

    Not before then: Gazebo integrates with a semi-implicit Euler step of a
    millisecond, whose position runs a step ahead of the exact one, which is
    some per cent of a swing a few dozen milliseconds old and nothing by the
    time it has fallen a few degrees.
    """
    readings = [swung["before"]] + swung.get("samples", []) + [swung["after"]]
    for reading in [r for r in readings if r["time"] >= 0.1]:
        swing = reading["joints"]["swing"]
        fallen = 1.0 * G * 0.5 * math.sin(math.radians(swing["pos"]))
        moving = 0.5 * INERTIA * math.radians(swing["vel"]) ** 2
        assert moving == pytest.approx(fallen, rel=2e-2, abs=1e-3)
    bottom = math.degrees(math.sqrt(2 * 1.0 * G * 0.5 / INERTIA))
    assert swung["after"]["joints"]["swing"]["pos"] == pytest.approx(90.0, abs=10.0)
    assert swung["after"]["joints"]["swing"]["vel"] == pytest.approx(bottom, rel=1e-2)
    assert swung["after"]["joints"]["swing"]["type"] == "continuous"


def test_in_gazebo_a_carriage_on_a_slide_falls_in_millimetres(swung):
    t = swung["after"]["time"]
    drop = swung["after"]["joints"]["drop"]

    assert drop["type"] == "prismatic"
    assert drop["pos"] == pytest.approx(-0.5 * G * t * t * 1000.0, rel=1e-2)
    assert drop["vel"] == pytest.approx(-G * t * 1000.0, rel=1e-2)


def test_in_gazebo_the_joints_that_are_not_reported_are_named(swung):
    assert set(swung["after"]["joints"]) == {"swing", "drop"}
    assert any("'socket'" in warning for warning in swung["warnings"])
    assert any("'spin'" in warning for warning in swung["warnings"])


def test_in_gazebo_the_copy_of_the_world_is_gone_when_the_run_is(swung):
    assert sorted(os.listdir(swung["directory"])) == ["scene.world"]


def test_in_gazebo_a_world_of_free_models_reports_no_joints(gazebo, tmp_path):
    (tmp_path / "scene.world").write_text(
        """<sdf version="1.9"><world name="free"><model name="box"><pose>0 0 1 0 0 0</pose><link name="l">
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        </link></model></world></sdf>""",
        encoding="utf-8",
    )

    result = sim.process(str(tmp_path), {"scene_file": str(tmp_path / "scene.world"), "duration": 0.2})

    assert result["before"]["joints"] == {}
    assert result["after"]["joints"] == {}
    assert "warnings" not in result
