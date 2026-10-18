#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""PartCAD's names in a world SDFormat can load, and back in what a run reports.

A scene stays 'world' in PartCAD, which is what PartCAD's default world is
called and what SDFormat reserves; the exporter writes it under another name
by SDFormat's own rules ('export_world.SDF_NAMES', through PartCAD's
'wrappers/naming.py') and says which in a sidecar beside the world, and the run
reports every body, joint and the world itself under PartCAD's names -- each
body at its link's pose in the world's frame, as the MuJoCo plugin reports it.

Most of this needs PartCAD and nothing else. The tests named 'in_gazebo' need a
Gazebo, and run in the image the simulation uses (see 'test_export_joints.py').
The last one also runs the same scene in MuJoCo, and needs this package's
sibling: set 'PC_SIM_MUJOCO' to a checkout of partcad-sim-mujoco, in an
environment with 'mujoco' installed.
"""

import os
import sys
import xml.etree.ElementTree as ElementTree

import pytest

import partcad as pc

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))

import kinematics  # noqa: E402
import naming  # noqa: E402
import simulate_gazebo as sim  # noqa: E402
from test_export_joints import (  # noqa: E402,F401
    bar,
    export,
    export_world,
    gazebo,
    in_a_scene,
    rig,
    stand_in_geometry,
    weighed,
)


def scene_of(*labels, label="world"):
    """A scene of free parts with these labels, side by side."""
    return {
        "name": "//p:%s" % label,
        "label": label,
        "kind": kinematics.SCENE,
        "assembly": [
            {
                "name": "//p:%d" % index,
                "label": name,
                "kind": kinematics.PART,
                "brep": b"%d" % index,
                "location": [[30.0 * index, 0.0, 0.0], [0.0, 0.0, 1.0], 0.0],
            }
            for index, name in enumerate(labels)
        ],
    }


def test_sdformat_reserves_what_libsdformat_does():
    import export_world

    rules = export_world.SDF_NAMES
    assert [rules.valid(name) for name in ("world", "__model__", "__x__", "a::b", "a b", "a/b", "cube-2")] == [
        False,
        False,
        False,
        False,
        False,
        False,
        True,
    ]


def test_a_scene_called_world_is_written_under_another_name_and_the_sidecar_says_so(tmp_path, export_world):
    result, path, sdf = export(export_world, tmp_path, scene_of("world", "cube", "ground_plane"), names_file=True)

    # The world, and the body called 'world', under names SDFormat accepts; the
    # ground plane is the one told apart from a body called that.
    assert sdf.find("world").get("name") == "world-2"
    models = [model.get("name") for model in sdf.find("world").findall("model")]
    assert models == ["ground_plane-2", "world-2", "cube", "ground_plane"]
    assert any("'world' as 'world-2'" in warning for warning in result["warnings"])

    sidecar = naming.read_sidecar(str(path))
    assert sidecar["format"] == "world"
    assert sidecar["names"]["scene"] == {"world": "world-2"}
    assert sidecar["bodies"] == {
        "world": {"path": ["world-2", "world-2_link"]},
        "cube": {"path": ["cube", "cube_link"]},
        "ground_plane": {"path": ["ground_plane", "ground_plane_link"]},
    }


def test_no_sidecar_is_written_unless_asked_for(tmp_path, export_world):
    _result, path, _sdf = export(export_world, tmp_path, scene_of("cube"))
    assert not os.path.exists(naming.sidecar_path(str(path)))


def test_a_joint_is_recorded_where_it_is_what_it_is_and_how_to_read_it(tmp_path, stand_in_geometry, export_world):
    """One SDFormat joint for one PartCAD joint, of two freedoms here, under PartCAD's name and with its map."""
    import json

    tree = in_a_scene(rig(tmp_path, "motion: universal"))
    _result, path, _sdf = export(export_world, tmp_path, tree, names_file=True)
    joints = naming.read_sidecar(str(path))["joints"]
    joint = kinematics.bodies(tree).by_name("arm").joint
    assert joints == {
        "arm-base": {
            "path": ["subject", "arm-base"],
            "joint": "arm-base",
            "map": json.loads(json.dumps(kinematics.joint_map(joint))),
        },
    }
    assert [coordinate["parameter"] for coordinate in joints["arm-base"]["map"]["coordinates"]] == ["turnZ", "turnY"]


# A model turned a quarter turn about Z and moved 100 mm along X, and its link
# 10 mm along the model's X: in the world, the link is 10 mm along Y from the
# model. Gazebo publishes each relative to its parent.
TURNED = (
    "header {\n  stamp {\n    sec: 1\n  }\n}\n"
    'pose {\n  name: "rig"\n  position {\n    x: 0.1\n  }\n  orientation {\n    z: 0.7071067811865476\n'
    "    w: 0.7071067811865476\n  }\n}\n"
    'pose {\n  name: "rig_link"\n  orientation {\n    w: 1\n  }\n}\n'
    'pose {\n  name: "arm"\n  position {\n    x: 0.01\n  }\n  orientation {\n    w: 1\n  }\n}\n'
    'pose {\n  name: "visual"\n  position {\n    z: 0.5\n  }\n}\n'
)

SIDECAR = {
    "names": {"scene": {"world": "world-2"}, "body": {"subject": "rig", "arm": "arm"}},
    "bodies": {"subject": {"path": ["rig", "rig_link"]}, "arm": {"path": ["rig", "arm"]}},
    "joints": {"swing": {"path": ["rig", "swing-2"], "joint": "swing"}},
}


def test_a_body_is_read_at_its_link_in_the_worlds_frame_under_partcads_name():
    reading = sim.snapshot(sim.parse_message(TURNED), sim.Names(SIDECAR))
    assert sorted(reading["bodies"]) == ["arm", "subject"]
    assert reading["bodies"]["subject"]["pos"] == pytest.approx([100.0, 0.0, 0.0])
    assert reading["bodies"]["arm"]["pos"] == pytest.approx([100.0, 10.0, 0.0])
    assert reading["bodies"]["arm"]["quat"] == pytest.approx([0.7071067811865476, 0.0, 0.0, 0.7071067811865476])


def test_a_world_with_no_sidecar_is_read_as_gazebo_publishes_it():
    reading = sim.snapshot(sim.parse_message(TURNED))
    assert sorted(reading["bodies"]) == ["arm", "rig", "rig_link", "visual"]
    assert reading["bodies"]["arm"]["pos"] == pytest.approx([10.0, 0.0, 0.0])


def test_a_body_whose_link_the_message_does_not_name_once_is_left_out():
    twice = TURNED + 'pose {\n  name: "arm"\n}\n'
    reading = sim.snapshot(sim.parse_message(twice), sim.Names(SIDECAR))
    assert sorted(reading["bodies"]) == ["subject"]


def test_a_joint_is_reported_and_warned_about_under_partcads_names():
    world = ElementTree.fromstring(
        '<world name="world-2"><model name="rig">'
        '<joint name="swing-2" type="revolute"><axis><limit><lower>-1</lower><upper>1</upper></limit></axis></joint>'
        '<joint name="knot" type="gearbox"/>'
        "</model></world>"
    )
    warnings = []
    names = sim.Names(SIDECAR)
    (model,) = sim.plan_joints(world, "/p", warnings, names)
    assert model.joints == {"swing-2": ("revolute", "swing", "revolute")}
    assert model.shown == "subject"
    assert any("of model 'subject'" in warning for warning in warnings)
    assert names.world("world-2") == "world" and names.world("other") == "other"


#
# In Gazebo, and beside MuJoCo
#

# Where the scene puts the pendulum: moved off the origin and turned a quarter
# turn about X, so that the hinge -- the bore's Z -- lies along the world's Y.
# A model's frame that is not the world's is what tells a link's pose in its
# model from its pose in the world.
PLACED = [[120.0, -40.0, 500.0], [1.0, 0.0, 0.0], 90.0]
DURATION = 0.3
# The same pendulum hung so that it starts at rest at the bottom: the bar, along
# the arm's X, points down, and the hinge is horizontal. A third of a turn about
# (1, 1, -1) takes X to -Z and Z to -Y. Nothing in it moves, in either engine,
# so the two can be held to one pose after as well as before.
HANGING = [[120.0, -40.0, 500.0], [3**-0.5, 3**-0.5, -(3**-0.5)], 120.0]


@pytest.fixture
def pendulum(tmp_path, stand_in_geometry):
    tree = in_a_scene(rig(tmp_path, "motion: revolute"), PLACED)
    return tree, weighed(tree, {"arm": bar()})


def _where_partcad_puts(tree, name, angle):
    """Where the body 'name' is in the world, its joint (if any) at 'angle'."""
    found = kinematics.bodies(tree)
    body = found.by_name(name)
    if body.joint is None:
        return body.world
    return kinematics.compose(body.parent.world, body.joint.location([angle]))


def _turn_between(a, b):
    """The angle in degrees one orientation is turned from another, either sign of either quaternion."""
    import math

    dot = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return math.degrees(2.0 * math.acos(dot))


def _close(reading, pose, mm, degrees):
    """Whether a reported body is at 'pose': within 'mm' of its position and 'degrees' of its orientation."""
    quat, pos = pose
    return (
        max(abs(a - b) for a, b in zip(reading["pos"], pos)) <= mm and _turn_between(reading["quat"], quat) <= degrees
    )


def _run_gazebo(tmp_path, tree, properties):
    import export_world

    _result, path, sdf = export(
        export_world,
        tmp_path,
        tree,
        "pendulum.world",
        static=False,
        self_collide=True,
        ground_plane=False,
        names_file=True,
        properties=properties,
    )
    assert sdf.find("world").get("name") == "world-2"
    return sim.process(str(tmp_path), {"scene_file": str(path), "duration": DURATION})


def test_in_gazebo_a_scene_called_world_reports_partcads_names_and_world_poses(
    gazebo, tmp_path, pendulum, export_world
):
    tree, properties = pendulum
    result = _run_gazebo(tmp_path, tree, properties)

    assert result["world"] == "world"
    for reading in (result["before"], result["after"]):
        assert sorted(reading["bodies"]) == ["arm", "subject"]
        assert sorted(reading["joints"]) == ["arm-base"]
        # Every body where the joint's own product puts it at the angle Gazebo
        # reports at that instant -- in the world, not in the model the subject
        # is, which the scene moved and turned.
        angle = reading["joints"]["arm-base"]["pos"]
        assert _close(reading["bodies"]["arm"], _where_partcad_puts(tree, "arm", angle), 0.01, 0.01), angle
        assert _close(reading["bodies"]["subject"], _where_partcad_puts(tree, "subject", 0.0), 0.01, 0.01)
    assert abs(result["after"]["joints"]["arm-base"]["pos"]) > 10.0


def _run_mujoco(tmp_path, tree, properties, monkeypatch):
    checkout = os.environ.get("PC_SIM_MUJOCO")
    if not checkout:
        pytest.skip("set PC_SIM_MUJOCO to a checkout of partcad-sim-mujoco to compare against MuJoCo")
    pytest.importorskip("mujoco")
    import shutil

    import ocp_serialize

    sys.path.append(os.path.abspath(checkout))
    import export_mjcf
    import simulate_mujoco

    from test_export_joints import CUBE_STL

    monkeypatch.setattr(ocp_serialize, "decode_shape", lambda obj: ("shape", obj.get("brep")))
    monkeypatch.setattr(export_mjcf, "write_mesh", lambda shape, path, options: shutil.copyfile(CUBE_STL, path))
    path = tmp_path / "pendulum.xml"
    export_mjcf.process(
        str(path),
        {
            "wrapped": tree,
            "properties": properties,
            "static": False,
            "ground_plane": False,
            "names_file": True,
        },
    )
    return simulate_mujoco.process(str(tmp_path), {"scene_file": str(path), "duration": DURATION})


def test_in_gazebo_and_in_mujoco_one_scene_reads_the_same_bodies_in_the_same_frame(
    gazebo, tmp_path, stand_in_geometry, export_world, monkeypatch
):
    tree = in_a_scene(rig(tmp_path, "motion: revolute"), HANGING)
    properties = weighed(tree, {"arm": bar()})
    (tmp_path / "gazebo").mkdir()
    (tmp_path / "mujoco").mkdir()
    in_mujoco = _run_mujoco(tmp_path / "mujoco", tree, properties, monkeypatch)
    in_gazebo = _run_gazebo(tmp_path / "gazebo", tree, properties)

    for result in (in_gazebo, in_mujoco):
        assert sorted(result["after"]["bodies"]) == ["arm", "subject"]
    # Every body at one pose in both, in the world's frame, before and after:
    # the arm hangs off a model the scene moved and turned, so a link read in
    # its model's frame would be 10 mm from where the world has it, and turned.
    for when in ("before", "after"):
        for name in ("arm", "subject"):
            gazebo_pose = in_gazebo[when]["bodies"][name]
            in_both = in_mujoco[when]["bodies"][name]
            assert _close(in_both, (gazebo_pose["quat"], gazebo_pose["pos"]), 0.05, 0.05), (when, name)
            assert _close(in_both, _where_partcad_puts(tree, name, 0.0), 0.05, 0.05), (when, name)
    print(
        "hanging: Gazebo's arm at %s mm, MuJoCo's at %s mm"
        % (in_gazebo["after"]["bodies"]["arm"]["pos"], in_mujoco["after"]["bodies"]["arm"]["pos"])
    )
