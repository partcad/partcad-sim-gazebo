#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What can be checked about the simulation plugin without a Gazebo in the room.

Gazebo is a native program rather than a wheel, so a contributor and a CI runner
alike may not have one. That rules out an end-to-end run, and it does *not* rule
out the parts most likely to be quietly wrong -- which here is nearly all of it,
because what this plugin does is read what a program says and turn it into what
PartCAD reads:

* the protobuf text-format parser, which is the one thing that fails silently
  rather than loudly: a field it drops becomes a body that did not move;
* splitting one stream into messages, which decides which reading is 'after';
* the quaternion, whose component order in the message is not the order PartCAD
  states one in, so a copy rather than a re-ordering is wrong by a rotation;
* the unit conversion, which is a factor of a thousand and looks plausible in
  either direction until something is compared against a part's own dimensions;
* and that a machine with no Gazebo is told which of the two ways to get one
  rather than shown a traceback.

Run it with `pytest`. It needs nothing but the standard library.
"""

import os
import sys
import time

import pytest
from xml.etree import ElementTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate_gazebo as sim  # noqa: E402

#
# Finding a Gazebo
#


def test_a_machine_with_no_gazebo_is_told_both_ways_to_get_one(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda _name, path=None: None)

    with pytest.raises(sim.GazeboMissing) as raised:
        sim.find_gazebo()
    message = str(raised.value)
    # The two answers, because which one applies is a fact about the machine.
    assert "gazebosim.org" in message
    assert "docker" in message


def test_the_newest_generation_present_is_the_one_used(monkeypatch):
    """'gz sim' today, 'ign gazebo' before it. A machine may well have both."""
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name, path=None: "/usr/bin/" + name)

    binary, args, topic = sim.find_gazebo()
    assert binary == "/usr/bin/gz"
    assert args == ["sim"]
    assert topic == "topic"


def test_an_older_generation_is_run_the_way_it_spells_itself(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name, path=None: "/usr/bin/ign" if name == "ign" else None)

    binary, args, _topic = sim.find_gazebo()
    assert binary == "/usr/bin/ign"
    assert args == ["gazebo"]


def test_an_override_names_the_one_program_both_tools_are_subcommands_of(monkeypatch):
    """'gz sim' and 'gz topic' are one program; there is no 'topic' beside it."""
    monkeypatch.setenv("PC_GZ", "/opt/gz/bin/gz")

    binary, args, topic = sim.find_gazebo()
    assert (binary, args, topic) == ("/opt/gz/bin/gz", ["sim"], "topic")


def test_a_gazebo_ros_installed_is_found_through_the_environment_ros_sets_up(monkeypatch):
    """ROS keeps 'gz' off PATH until its setup script has run, which a container
    started by anything but its own entrypoint has not."""
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name, path=None: "/ros/bin/gz" if path == "/ros/bin" else None)
    monkeypatch.setattr(sim, "ros_environment", lambda: {"PATH": "/ros/bin", "GZ_CONFIG_PATH": "/ros/share"})

    (binary, args, topic), environment = sim.locate_gazebo()

    assert (binary, args, topic) == ("/ros/bin/gz", ["sim"], "topic")
    assert environment["GZ_CONFIG_PATH"] == "/ros/share"


def test_a_gazebo_on_path_is_run_in_this_environment(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name, path=None: "/usr/bin/" + name)

    _found, environment = sim.locate_gazebo()

    assert environment is None


def test_with_neither_the_machine_is_still_told_both_ways_to_get_one(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name, path=None: None)
    monkeypatch.setattr(sim, "ros_environment", lambda: None)

    with pytest.raises(sim.GazeboMissing):
        sim.locate_gazebo()


#
# Reading what the server publishes
#

POSE_MESSAGE = """header {
  stamp {
    sec: 3
    nsec: 500000000
  }
}
pose {
  name: "block"
  id: 8
  position {
    x: 0.1
    y: -0.02
    z: 0.5
  }
  orientation {
    x: 0
    y: 0
    z: 0.7071068
    w: 0.7071068
  }
}
"""


def test_a_pose_is_read_in_millimetres():
    """SDFormat is metres and PartCAD is millimetres, everywhere and always."""
    reading = sim.snapshot(sim.parse_message(POSE_MESSAGE))

    assert reading["bodies"]["block"]["pos"] == pytest.approx([100.0, -20.0, 500.0])


def test_the_quaternion_is_stated_the_way_partcad_states_one():
    """'w x y z'. The message prints 'x y z w', which is a different rotation."""
    reading = sim.snapshot(sim.parse_message(POSE_MESSAGE))

    assert reading["bodies"]["block"]["quat"] == pytest.approx([0.7071068, 0.0, 0.0, 0.7071068])


def test_the_simulated_clock_comes_out_of_the_message():
    """Which is what a run waits on, rather than on the wall clock."""
    assert sim.snapshot(sim.parse_message(POSE_MESSAGE))["time"] == pytest.approx(3.5)


def test_a_field_the_message_omits_reads_as_zero():
    """Protobuf text format prints no field whose value is the default."""
    reading = sim.snapshot(
        sim.parse_message(
            'header {\n}\npose {\n  name: "still"\n  position {\n  }\n  orientation {\n    w: 1\n  }\n}\n'
        )
    )

    assert reading["time"] == 0.0
    assert reading["bodies"]["still"]["pos"] == [0.0, 0.0, 0.0]
    assert reading["bodies"]["still"]["quat"] == [1.0, 0.0, 0.0, 0.0]


def test_an_entity_with_no_name_is_kept_under_its_id():
    """Something moved; a reading that omits it is worse than an odd name."""
    reading = sim.snapshot(sim.parse_message("header {\n}\npose {\n  id: 12\n  position {\n    z: 1\n  }\n}\n"))

    assert list(reading["bodies"]) == ["entity_12"]


def test_every_entity_in_one_message_is_one_reading():
    """A reading is where everything was at one instant, not one body's history."""
    text = POSE_MESSAGE + 'pose {\n  name: "ground"\n  id: 4\n  position {\n  }\n}\n'

    assert sorted(sim.snapshot(sim.parse_message(text))["bodies"]) == ["block", "ground"]


#
# One stream, many messages
#


def test_a_stream_is_split_where_the_next_message_starts():
    """'gz topic -e' prints one message after another with nothing between."""
    stream = (POSE_MESSAGE + POSE_MESSAGE.replace("sec: 3", "sec: 9")).splitlines(keepends=True)

    times = [sim.snapshot(sim.parse_message(text))["time"] for text in sim.messages(stream)]
    assert times == pytest.approx([3.5, 9.5])


def test_the_last_message_is_not_lost_at_the_end_of_the_stream():
    """It is the one 'after' is, so losing it loses the whole run."""
    assert len(list(sim.messages(POSE_MESSAGE.splitlines(keepends=True)))) == 1


def test_a_stream_that_said_nothing_is_no_messages_rather_than_one_empty_one():
    assert list(sim.messages([])) == []


def test_a_subscriber_replaced_is_heard_no_more_its_end_included():
    """The poses are subscribed to again when a running server sends none (see 'RESUBSCRIBE'): whatever the
    subscriber replaced had queued, and the end of it as it is stopped, is not the end of the one read now."""
    replaced = (POSE_MESSAGE + POSE_MESSAGE.replace("sec: 3", "sec: 4")).splitlines(keepends=True)
    poses = sim.Messages(iter(replaced))
    # Its two messages and its end, queued before it is replaced.
    deadline = time.monotonic() + 5
    while poses.queue.qsize() < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert poses.queue.qsize() == 3
    read, write = os.pipe()
    with os.fdopen(read, encoding="utf-8") as current, os.fdopen(write, "w", encoding="utf-8") as feed:
        poses.follow(current)
        assert poses.next(0.2) is None
        feed.write(POSE_MESSAGE.replace("sec: 3", "sec: 9") + POSE_MESSAGE)
        feed.flush()
        text = poses.next(5)
        assert sim.snapshot(sim.parse_message(text))["time"] == pytest.approx(9.5)
        feed.close()
        assert sim.snapshot(sim.parse_message(poses.next(5)))["time"] == pytest.approx(3.5)
        assert poses.next(5) is sim.ENDED
    assert poses.count == 2


#
# The world's name, which is what its topics are named after
#


def test_the_world_name_is_read_out_of_the_file(tmp_path):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="warehouse"/></sdf>', encoding="utf-8")

    assert sim.world_name(str(path)) == "warehouse"


#
# The gravity a run is under
#


def test_a_world_that_states_no_gravity_runs_under_sdformats_own(tmp_path):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="w"/></sdf>', encoding="utf-8")

    assert sim.world_gravity(str(path)) == [0.0, 0.0, -9.8]


def test_a_world_runs_under_the_gravity_the_scene_wrote_into_it(tmp_path):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="w"><gravity>0 0 -1.62</gravity></world></sdf>', encoding="utf-8")

    assert sim.world_gravity(str(path)) == [0.0, 0.0, -1.62]


@pytest.mark.parametrize("gravity", [[0.0, -9.81], 9.81, "down", [0.0, 0.0, "x"], [0.0, 0.0, float("nan")]])
def test_a_gravity_that_is_not_three_numbers_is_refused_before_gazebo_starts(tmp_path, gravity):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="w"/></sdf>', encoding="utf-8")

    with pytest.raises(ValueError, match="gravity"):
        sim.world_gravity(str(path), gravity)
    assert "<gravity>" not in path.read_text(encoding="utf-8")


def test_a_world_whose_gravity_is_not_three_numbers_is_named(tmp_path):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="w"><gravity>0 -9.8</gravity></world></sdf>', encoding="utf-8")

    with pytest.raises(ValueError, match="scene.world"):
        sim.world_gravity(str(path))


def test_a_gravity_the_simulation_is_handed_beats_the_scenes_and_is_what_gazebo_loads(tmp_path):
    """'params: {gravity: ...}' on a 'simulate:' is the explicit, per-run answer.

    Written into the world rather than only reported: the report used to say
    whatever the request said while Gazebo ran under the file's.
    """
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="w"><gravity>0 0 -1.62</gravity></world></sdf>', encoding="utf-8")

    assert sim.world_gravity(str(path), [0.0, 0.0, -3.72]) == [0.0, 0.0, -3.72]
    assert sim.world_gravity(str(path)) == [0.0, 0.0, -3.72]
    assert sim.world_name(str(path)) == "w"


def test_a_file_that_names_no_world_says_so_rather_than_reading_nothing(tmp_path):
    """Every topic is namespaced under the name, so a wrong one reads silence."""
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><model name="block"/></sdf>', encoding="utf-8")

    with pytest.raises(Exception, match="names no world"):
        sim.world_name(str(path))


#
# The step and the pace
#


def test_a_timestep_is_the_world_s_max_step_size_written_into_the_file_it_loads(tmp_path):
    path = tmp_path / "w.world"
    path.write_text('<sdf version="1.11"><world name="w"><gravity>0 0 -9.8</gravity></world></sdf>', encoding="utf-8")

    assert sim.world_physics(str(path)) == 0.001
    assert ElementTree.parse(str(path)).getroot().find("world/physics") is None
    assert sim.world_physics(str(path), timestep=0.0005) == 0.0005
    physics = ElementTree.parse(str(path)).getroot().find("world/physics")
    assert physics.get("type") == "dart" and float(physics.findtext("max_step_size")) == 0.0005
    assert physics.find("real_time_factor") is None


def test_a_world_s_own_physics_is_kept_and_only_what_is_asked_for_changed(tmp_path):
    path = tmp_path / "w.world"
    path.write_text(
        '<sdf version="1.11"><world name="w"><physics name="mine" type="dart"><max_step_size>0.002</max_step_size>'
        "<real_time_factor>1</real_time_factor></physics></world></sdf>",
        encoding="utf-8",
    )

    assert sim.world_physics(str(path)) == 0.002
    assert sim.world_physics(str(path), real_time_factor=0.25) == 0.002
    (physics,) = ElementTree.parse(str(path)).getroot().findall("world/physics")
    assert physics.get("name") == "mine"
    assert float(physics.findtext("max_step_size")) == 0.002 and float(physics.findtext("real_time_factor")) == 0.25


def test_a_run_is_slowed_only_as_far_as_its_samples_need():
    """Sixty pose messages a second of the wall clock, and one a step at most."""
    assert sim.pace(0, 10.0, 0.001) is None
    # Sixty readings over ten seconds: Gazebo's own pace gives them.
    assert sim.pace(60, 10.0, 0.001) is None
    # 599 over 0.6 s is one a millisecond: sixty a second, a sixteenth as fast.
    assert sim.pace(599, 0.6, 0.001) == pytest.approx(0.8 * 0.001 * 60.0)
    # More than one a step is one a step.
    assert sim.pace(5999, 0.6, 0.001) == pytest.approx(0.8 * 0.001 * 60.0)
    assert sim.pace(999, 0.5, 0.0005) == pytest.approx(0.8 * 0.0005 * 60.0)
