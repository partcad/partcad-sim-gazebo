#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What a run reports for each PartCAD joint, read back through the joint map the exporter wrote.

A world PartCAD exports re-zeroes every joint at its start, so Gazebo counts
each one from where the assembly shows it; the sidecar beside the world carries
each joint's map back into PartCAD's terms ('kinematics.joint_map'), and the run
turns what Gazebo publishes into PartCAD's reading with PartCAD's own
'kinematics.reading' -- one entry per PartCAD joint, in degrees and
millimetres from PartCAD's zero:

    {"type": "revolute", "pos": 35.7, "vel": 114.6}
    {"type": "screw", "pos": {"turnZ": 95.7, "moveZ": 0.16}, "vel": {"turnZ": 114.6, "moveZ": 3.2}}
    {"type": "ball", "pos": {"turnX": ..., "turnY": ..., "turnZ": ...}, "quat": [w, x, y, z], "vel": [x, y, z]}

which is the shape the MuJoCo plugin reports too, so one validation reads both.

The worlds and sidecars here are the exporter's own, for the rig of
'test_export_joints.py'; what Gazebo publishes is written by hand, in the text
format 'gz topic -e' prints, so that this needs PartCAD and nothing else. The
same readings in a real Gazebo are 'test_export_joints.py''s 'in_gazebo' tests.
"""

import math
import os
import sys

import pytest

import partcad as pc

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))

import kinematics  # noqa: E402
import naming  # noqa: E402
import simulate_gazebo as sim  # noqa: E402
import urdf_common  # noqa: E402
from test_export_joints import (  # noqa: E402,F401
    SCENE_LOCATION,
    export,
    export_world,
    in_a_scene,
    rig,
    stand_in_geometry,
)


def exported(module, tmp_path, bore, params=""):
    """The rig with 'bore' started at 'params', exported as a run exports it: its world, names, and joint."""
    tree = in_a_scene(rig(tmp_path, bore, params=params), SCENE_LOCATION)
    _result, path, sdf = export(module, tmp_path, tree, static=False, names_file=True)
    found = kinematics.bodies(tree)
    return sdf.find("world"), sim.Names(naming.read_sidecar(str(path))), found


def joint_message(*axes):
    """A JointStatePublisher message for the joint 'arm-base', its axes at '(position, velocity)' each."""
    lines = ["header {", "  stamp {", "    sec: 1", "  }", "}", 'name: "subject"', "joint {", '  name: "arm-base"']
    for tag, (position, velocity) in zip(("axis1", "axis2"), axes):
        lines += ["  %s {" % tag, "    position: %r" % position, "    velocity: %r" % velocity, "  }"]
    lines.append("}")
    return sim.parse_message("\n".join(lines) + "\n")


def read(world, names, *axes):
    warnings = []
    (model,) = sim.plan_joints(world, "/p", warnings, names)
    states = sim.joint_states(joint_message(*axes), model, warnings)
    assert warnings == []
    return states["arm-base"]


def test_a_joint_of_one_freedom_reads_a_number_from_partcad_s_zero(tmp_path, stand_in_geometry, export_world):
    """Gazebo's 0.1 rad from where it started is PartCAD's 30 + 5.73 degrees."""
    world, names, _found = exported(export_world, tmp_path, "motion: revolute", ", toParams: {turnZ: 30}")

    assert read(world, names, (0.1, 2.0)) == {
        "type": "revolute",
        "pos": pytest.approx(30.0 + math.degrees(0.1)),
        "vel": pytest.approx(math.degrees(2.0)),
    }
    # And in millimetres for a move.
    world, names, _found = exported(
        export_world, tmp_path / "slide", "parameters: {moveZ: [-5, 50]}\nmotion: prismatic", ", toParams: {moveZ: 3}"
    )
    assert read(world, names, (0.002, -0.5)) == {"type": "prismatic", "pos": pytest.approx(5.0), "vel": -500.0}


def test_a_joint_of_several_freedoms_reads_an_object_keyed_by_parameter(tmp_path, stand_in_geometry, export_world):
    world, names, found = exported(export_world, tmp_path, "motion: universal", ", toParams: {turnZ: 10, turnY: 20}")
    assert found.by_name("arm").joint.parameter_names() == ["turnZ", "turnY"]

    state = read(world, names, (0.1, 1.0), (-0.2, 0.5))

    assert state["type"] == "universal"
    assert state["pos"] == pytest.approx({"turnZ": 10.0 + math.degrees(0.1), "turnY": 20.0 - math.degrees(0.2)})
    assert state["vel"] == pytest.approx({"turnZ": math.degrees(1.0), "turnY": math.degrees(0.5)})
    # A turn at nought is a field protobuf's text format leaves out.
    message = joint_message((0.1, 1.0))
    message["joint"]["axis2"] = {}
    (model,) = sim.plan_joints(world, "/q", [], names)
    assert sim.joint_states(message, model, [])["arm-base"]["pos"]["turnY"] == pytest.approx(20.0)


def test_a_screw_reads_its_turn_and_the_move_that_follows_it(tmp_path, stand_in_geometry, export_world):
    world, names, found = exported(export_world, tmp_path, "threadStep: 10\nmotion: screw", ", toParams: {turnZ: 90}")
    move = next(freedom for freedom in found.by_name("arm").joint.freedoms if freedom.kind == kinematics.MOVE)

    state = read(world, names, (0.1, 2.0))

    turn = 90.0 + math.degrees(0.1)
    assert state["pos"] == pytest.approx({"turnZ": turn, "moveZ": move.ratio * turn + move.offset})
    assert state["vel"] == pytest.approx({"turnZ": math.degrees(2.0), "moveZ": move.ratio * math.degrees(2.0)})


def _pose_block(name, pose, seconds):
    """One pose of a PosePublisher's 'Pose_V', as 'gz topic -e' prints it: metres, 'x y z w'."""
    (w, x, y, z), (px, py, pz) = pose
    whole = int(seconds)
    return "\n".join(
        [
            "pose {",
            "  header {",
            "    stamp {",
            "      sec: %d" % whole,
            "      nsec: %d" % round((seconds - whole) * 1e9),
            "    }",
            "  }",
            '  name: "%s"' % name,
            "  position {",
            "    x: %r" % (px / 1000.0),
            "    y: %r" % (py / 1000.0),
            "    z: %r" % (pz / 1000.0),
            "  }",
            "  orientation {",
            "    x: %r" % x,
            "    y: %r" % y,
            "    z: %r" % z,
            "    w: %r" % w,
            "  }",
            "}",
        ]
    )


def _ball_world(module, tmp_path):
    world, names, found = exported(module, tmp_path, "motion: ball", ", toParams: {turnX: 10, turnY: 20, turnZ: 30}")
    entry = names.posed["arm-base"]
    return world, names, found, entry


def _child_at(entry, parent, rotation):
    """Where the ball puts its child, turned 'rotation' from its start, given its parent's pose (millimetres)."""
    ball = entry["ball"]
    on_parent = urdf_common.compose(parent, sim._pose_data(ball["in_parent"]))
    turned = urdf_common.compose(on_parent, (tuple(rotation), (0.0, 0.0, 0.0)))
    return urdf_common.compose(turned, kinematics.inverse(sim._pose_data(ball["in_child"])))


def test_a_ball_is_read_from_its_links_and_its_speed_from_their_poses_a_step_either_side(
    tmp_path, stand_in_geometry, export_world
):
    """Gazebo publishes two of its three coordinates, so its two links say where it is, and how fast it turns."""
    world, names, found, entry = _ball_world(export_world, tmp_path)
    joint = found.by_name("arm").joint
    warnings = []
    models = sim.plan_joints(world, "/p", warnings, names)
    # Not a joint state, but its model's links, every step.
    assert [(model.topic, model.links) for model in models] == [("/model/subject/pose", True)]
    plugin = world.find("model[@name='subject']/plugin")
    assert plugin.get("name") == "gz::sim::systems::PosePublisher"
    assert plugin.findtext("use_pose_vector_msg") == "true" and plugin.findtext("update_frequency") == "-1"
    assert warnings == []

    # Turned from its start about a line of its own at 2 rad/s, read at t = 1 s.
    parent = found.by_name("subject").world
    axis, rate = (0.6, 0.0, 0.8), 2.0

    def turned(seconds):
        return urdf_common.axis_angle_to_quat(axis, math.degrees(rate * seconds))

    child = _child_at(entry, parent, turned(1.0))
    reading = {
        "time": 1.0,
        "bodies": {
            "subject": {"pos": list(parent[1]), "quat": list(parent[0])},
            "arm": {"pos": list(child[1]), "quat": list(child[0])},
        },
        "joints": {},
    }
    stream = sim.LinkStream(models[0])
    text = "\n\n".join(
        _pose_block("subject::subject_link", parent, t)
        + "\n"
        + _pose_block("subject::arm", _child_at(entry, parent, turned(t)), t)
        for t in (0.999, 1.0, 1.001)
    )
    stream.follow(iter((text + "\n\n").splitlines(keepends=True)))
    assert [stamp for stamp, _text in stream.entries] == pytest.approx([0.999, 1.0, 1.001])

    sim.fill(reading, [stream], warnings, 0.1, names)

    state = reading["joints"]["arm-base"]
    assert state["type"] == "ball"
    # PartCAD's rotation is the start's and then Gazebo's: R_s . R_e.
    rotation = urdf_common.quat_mul(tuple(entry["map"]["rotation"]), turned(1.0))
    assert sim_close(state["quat"], rotation)
    # Its three turns are that rotation: PartCAD's own joint at them puts the
    # child where it is.
    expected = kinematics.compose(parent, joint.location([state["pos"][name] for name in joint.parameter_names()]))
    assert _same_pose(expected, child)
    # The angular velocity about the ball's own moving frame, in deg/s.
    assert state["vel"] == pytest.approx([math.degrees(rate) * v for v in axis], abs=1e-6)
    assert warnings == []


def test_a_ball_whose_links_say_nothing_either_side_is_read_without_a_speed(tmp_path, stand_in_geometry, export_world):
    world, names, found, entry = _ball_world(export_world, tmp_path)
    models = sim.plan_joints(world, "/p", [], names)
    stream = sim.LinkStream(models[0])
    stream.ended = True
    parent = found.by_name("subject").world
    child = _child_at(entry, parent, urdf_common.IDENTITY_Q)
    reading = {
        "time": 0.5,
        "bodies": {
            "subject": {"pos": list(parent[1]), "quat": list(parent[0])},
            "arm": {"pos": list(child[1]), "quat": list(child[0])},
        },
        "joints": {},
    }
    warnings = []

    sim.fill(reading, [stream], warnings, 0.1, names)

    state = reading["joints"]["arm-base"]
    assert "vel" not in state
    assert state["pos"] == pytest.approx({"turnX": 10.0, "turnY": 20.0, "turnZ": 30.0})
    assert warnings == [
        "the ball 'arm-base' is reported without its 'vel': Gazebo published no poses of its links either side of a "
        "reading to tell its speed by"
    ]


def _stream_of(model, poses_at, moments=(0.499, 0.5, 0.501)):
    """A link stream of 'model', fed what its PosePublisher would print at each of 'moments'.

    'poses_at(t)' is '{name: pose}' at 't', millimetres: the model's own pose
    in the world under its name, and each link's in the model under
    '<model>::<link>'.
    """
    stream = sim.LinkStream(model)
    text = "\n\n".join(
        "\n".join(_pose_block(name, pose, moment) for name, pose in poses_at(moment).items()) for moment in moments
    )
    stream.follow(iter((text + "\n\n").splitlines(keepends=True)))
    return stream


def test_a_floating_child_is_read_as_its_pose_and_its_rates_against_the_body_it_was_connected_to(
    tmp_path, stand_in_geometry, export_world
):
    """Its pose from every pose message; its rates from its own model's and its parent's poses over the last step."""
    world, names, found = exported(export_world, tmp_path, "motion: floating", ", toParams: {moveZ: 100}")
    joint = found.by_name("arm").joint
    warnings = []
    # No joint publishes it: it is a model of its own. The two models' poses
    # every step say how fast each body goes.
    models = sim.plan_joints(world, "/p", warnings, names)
    assert sorted((model.topic, model.links) for model in models) == [
        ("/model/arm/pose", True),
        ("/model/subject/pose", True),
    ]
    assert world.find("model[@name='arm']/plugin").findtext("publish_model_pose") == "true"

    # Every freedom on the move, and the body it is read against moving and
    # turning under it. By parameter: the order of a joint's freedoms is its
    # declaration's, and nothing here should lean on it.
    start = {"moveX": 40.0, "moveY": -25.0, "moveZ": 130.0, "turnX": 15.0, "turnY": -30.0, "turnZ": 50.0}
    rates = {"moveX": 100.0, "moveY": -50.0, "moveZ": 20.0, "turnX": 30.0, "turnY": 10.0, "turnZ": -40.0}
    assert sorted(joint.parameter_names()) == sorted(start)

    def values(t):
        return [start[name] + rates[name] * (t - 0.5) for name in joint.parameter_names()]

    def parent(t):
        moved = (urdf_common.axis_angle_to_quat((0.0, 0.6, 0.8), 90.0 * t), (200.0 * t, 0.0, -50.0 * t))
        return kinematics.compose(found.by_name("subject").world, moved)

    def child(t):
        return kinematics.compose(parent(t), joint.location(values(t)))

    moments = (0.499999, 0.5)
    streams = [
        _stream_of(
            next(model for model in models if model.scoped == "subject"),
            lambda t: {"subject": parent(t), "subject::subject_link": kinematics.IDENTITY},
            moments,
        ),
        _stream_of(
            next(model for model in models if model.scoped == "arm"),
            lambda t: {"arm": child(t), "arm::arm_link": kinematics.IDENTITY},
            moments,
        ),
    ]
    reading = {
        "time": 0.5,
        "bodies": {
            "subject": {"pos": list(parent(0.5)[1]), "quat": list(parent(0.5)[0])},
            "arm": {"pos": list(child(0.5)[1]), "quat": list(child(0.5)[0])},
        },
        "joints": {},
    }

    sim.fill(reading, streams, warnings, 0.1, names)

    state = reading["joints"]["arm-base"]
    assert state["type"] == "floating"
    assert state["pos"] == pytest.approx(start)
    assert len(state["quat"]) == 4
    vel = dict(state["vel"])
    assert len(vel.pop("angular")) == 3
    # The difference over the last step, of motions that are not straight lines
    # in the world -- which Gazebo's are, over one of its steps (semi-implicit
    # Euler) -- here a microsecond long, to be good to a part in a hundred
    # thousand.
    assert vel == pytest.approx(rates, rel=1e-5, abs=1e-3)
    assert warnings == []


def test_a_floating_child_whose_bodies_say_nothing_either_side_is_read_without_a_speed(
    tmp_path, stand_in_geometry, export_world
):
    world, names, found = exported(export_world, tmp_path, "motion: floating", ", toParams: {moveZ: 100}")
    joint = found.by_name("arm").joint
    parent = found.by_name("subject").world
    # At its start, whatever order its freedoms come in.
    child = kinematics.compose(parent, joint.location(joint.values()))
    reading = {
        "time": 0.5,
        "bodies": {
            "subject": {"pos": list(parent[1]), "quat": list(parent[0])},
            "arm": {"pos": list(child[1]), "quat": list(child[0])},
        },
        "joints": {},
    }
    warnings = []

    sim.fill(reading, [], warnings, 0.1, names)

    state = reading["joints"]["arm-base"]
    assert state["pos"]["moveZ"] == pytest.approx(100.0) and "vel" not in state
    assert warnings == [
        "the floating joint 'arm-base' is reported without its 'vel': Gazebo published no poses of its two bodies "
        "either side of a reading to tell their speeds by"
    ]


def test_a_floating_child_whose_bodies_are_not_both_in_the_reading_names_the_one_that_is_missing(
    tmp_path, stand_in_geometry, export_world
):
    """The body it is read against, or its own: the one to look for, and not the other."""
    _world, names, _found = exported(export_world, tmp_path, "motion: floating")
    at_rest = {"pos": [0.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0]}

    def read(bodies):
        reading, warnings = {"time": 0.5, "bodies": bodies, "joints": {}}, []
        sim.fill(reading, [], warnings, 0.1, names)
        assert reading["joints"] == {}
        return warnings

    assert read({"arm": at_rest}) == [
        "joint 'arm-base' is not reported: the body it is read against, 'subject', is not in the reading"
    ]
    assert read({"subject": at_rest}) == [
        "joint 'arm-base' is not reported: its own body, 'arm', is not in the reading"
    ]


def test_a_reading_waits_for_every_joint_stream_to_reach_its_instant():
    """A stream whose subscriber came up after the server's first step has only a later step to give."""
    model = sim.JointModel("rig", "/t/0", {"swing": ("revolute", "swing", "revolute")})
    late = sim.JointStream(model)
    message = (
        "header {\n  stamp {\n    nsec: 3000000\n  }\n}\n"
        'joint {\n  name: "swing"\n  axis1 {\n    position: 0.5\n  }\n}\n'
    )
    late.follow(iter(message.splitlines(keepends=True)))

    assert sim.fill({"time": 0.001, "bodies": {}, "joints": {}}, [late], [], 0.1) is False
    assert sim.fill({"time": 0.003, "bodies": {}, "joints": {}}, [late], [], 0.1) is True


def test_a_pose_publisher_s_messages_are_told_apart_by_the_blank_line_after_each():
    """A 'Pose_V' has no header of its own, only one in each pose: 'messages' would find none in it."""
    first = _pose_block("m::a", urdf_common.from_packed(None), 0.001)
    second = _pose_block("m::a", urdf_common.from_packed(None), 0.002)
    text = first + "\n" + first + "\n\n" + second + "\n\n"

    found = list(sim.separated(text.splitlines(keepends=True)))

    assert len(found) == 2
    assert [sim.message_time(message) for message in found] == pytest.approx([0.001, 0.002])
    assert len(sim.parse_message(found[0])["pose"]) == 2


def test_a_link_stream_keeps_the_step_before_a_reading_and_gives_that_step_and_the_reading_s():
    stream = sim.LinkStream(sim.JointModel("m", "/model/m/pose", {}, links=True))
    text = "\n\n".join(_pose_block("m::a", urdf_common.from_packed(None), t) for t in (0.001, 0.002, 0.003, 0.004))
    stream.follow(iter((text + "\n\n").splitlines(keepends=True)))

    stream.forget_before(0.003)

    assert [stamp for stamp, _text in stream.entries] == pytest.approx([0.002, 0.003, 0.004])
    # The last step before the reading: the speed the engine holds at it.
    (before, _), (after, _) = stream.last_step(0.003, patience=0.1)
    assert (before, after) == pytest.approx((0.002, 0.003))
    # A step the stream missed: the two nearest, one either side.
    (before, _), (after, _) = stream.last_step(0.0035, patience=0.1)
    assert (before, after) == pytest.approx((0.003, 0.004))
    # Nothing after the last there is.
    assert stream.last_step(0.005, patience=0.1) is None


def _same_pose(a, b, tolerance=1e-9):
    (qa, ta), (qb, tb) = a, b
    same = max(abs(x - y) for x, y in zip(qa, qb))
    flipped = max(abs(x + y) for x, y in zip(qa, qb))
    return min(same, flipped) <= tolerance and max(abs(x - y) for x, y in zip(ta, tb)) <= tolerance * 1000


def sim_close(a, b, tolerance=1e-9):
    return min(max(abs(x - y) for x, y in zip(a, b)), max(abs(x + y) for x, y in zip(a, b))) <= tolerance
