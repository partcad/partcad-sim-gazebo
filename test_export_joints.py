#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""The bodies and joints the world exporter writes, from PartCAD's own 'kinematics'.

The trees here are the ones PartCAD itself builds: a small package of cubes and
two interfaces, a 'motion:' on one of them, composed by PartCAD's own ASSY code
-- with a stand-in for every part's geometry, so nothing needs a CAD kernel --
and handed to the exporter the way 'pc export' and 'pc sim' hand it over. The
same trees the MJCF exporter of partcad-sim-mujoco is tested on.

What is checked, each against an answer that does not come from the exporter:

* **every row of the classification table** (section 8.13.4 of PartCAD's
  simulation design record): a revolute, prismatic, screw, universal or ball
  joint is one SDFormat joint of that type, named as PartCAD names it, with no
  ``continuous`` anywhere and no link nobody built; a floating child is a model
  of its own; and a cylindrical, planar or composite joint is refused, naming
  the joint, its type, the engine and the ways on, before anything is written;
* **every placement, re-zeroed**: the world file is read back with a forward
  kinematics of its own (``sdf_poses``) -- the links' poses, each joint's frame
  and axis as SDFormat states them -- and at Gazebo's zero every part is where
  the assembly shows it, every joint at its start, and every body, moved, is
  where PartCAD's own 'kinematics' puts the start plus the move;
* **what a run reads back**: the joint map each joint's sidecar entry carries
  reads the joint's start at Gazebo's zero;
* **in a real Gazebo**: every native joint is a world SDFormat accepts; started
  away from its zero with nothing to move it, it reads its start; under a
  gravity along no axis, every reading put through PartCAD's own joint is
  where Gazebo has the body; and physics with a closed form -- a bar on a
  revolute joint released horizontal passes the bottom at sqrt(2 m g d / I) and
  swings with the period 4 K(sin 45 deg) sqrt(I / (m g d)) of a right-angle
  swing, started half a turn round it reads 180 and its bottom unwrapped, a
  bound stops it where PartCAD says, a carriage falls g t^2 / 2 from its start
  to its stop, a floating child falls free at g t, a lid falling free beside
  the swinging bar has the rates against it that the engine's own velocities
  give, and a nut on a screw moves as its thread says.

The last are skipped without a Gazebo, which CI's runners do not have; run them
in the image the simulation uses, with PartCAD in the environment:

    docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 \\
        --entrypoint bash \\
        ghcr.io/partcad/partcad-sim-gazebo:latest -c \\
        'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest numpy partcad \\
         && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_export_joints.py'
"""

import asyncio
import json
import math
import os
import shutil
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ElementTree

import pytest

import partcad as pc

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))

import kinematics  # noqa: E402
import naming  # noqa: E402
import urdf_common  # noqa: E402

CUBE_STL = os.path.join(HERE, "tests", "data", "cube.stl")
G = 9.81

# A base with a bore on top, and two parts that fit it: what every joint below
# is made of. 'bore' is where each test says how the connection moves.
RIG = """
parts:
  base:
    type: stl
    path: cube.stl
    implements:
      bore:
        top: [[0, 0, 10], [0, 0, 1], 0]
  arm:
    type: stl
    path: cube.stl
    implements:
      pin:
      bore:
        top: [[0, 0, 10], [0, 0, 1], 0]
  lid:
    type: stl
    path: cube.stl
    implements:
      pin:
interfaces:
  bore:
    ports:
      hole:
{bore}
  pin:
    ports:
      tip: [[0, 0, 0], [0.71, 0.71, 0], 180]
    mates:
      bore: {{}}
assemblies:
  rig:
    type: assy
"""

ONE_JOINT = """
links:
  - part: base
  - part: arm
    connect: {{name: base, with: pin, to: bore{params}}}
"""


def _indented(text, spaces):
    text = textwrap.dedent(text or "").strip("\n")
    return textwrap.indent(text, " " * spaces) if text else ""


@pytest.fixture
def stand_in_geometry(monkeypatch):
    """Every part a shape of no geometry, and every ASSY object composed as PartCAD composes it."""

    async def part(self, ctx):
        return dict(self.get_cache_metadata(), brep=b"BREP")

    async def assembly(self, ctx):
        await self.do_instantiate()
        return await self._get_shape_real(ctx)

    monkeypatch.setattr(pc.Part, "get_wrapped", part)
    monkeypatch.setattr(pc.Assembly, "get_wrapped", assembly)


@pytest.fixture
def export_world(monkeypatch):
    """The exporter, writing every mesh as the 10 mm test cube."""
    import export_world as module
    import ocp_serialize

    monkeypatch.setattr(ocp_serialize, "decode_shape", lambda obj: ("shape", obj.get("brep")))
    monkeypatch.setattr(module, "write_mesh", lambda shape, path, options: shutil.copyfile(CUBE_STL, path))
    return module


def rig(tmp_path, bore, assy=None, params=""):
    """The tree PartCAD builds for 'rig': a base and an arm connected through 'bore'."""
    root = tmp_path / "package"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy(CUBE_STL, root / "cube.stl")
    (root / "partcad.yaml").write_text(RIG.format(bore=_indented(bore, 4)), encoding="utf-8")
    (root / "rig.assy").write_text(textwrap.dedent(assy or ONE_JOINT.format(params=params)), encoding="utf-8")
    ctx = pc.Context(str(root))
    assembly = ctx.get_assembly(":rig")
    assert assembly is not None
    return asyncio.run(assembly.get_wrapped(ctx))


def in_a_scene(tree, location=None, fixed=True):
    """The assembly placed in a scene, as a simulation's world places its subject."""
    subject = dict(tree, label="subject", kind=kinematics.ASSEMBLY)
    if location is not None:
        subject["location"] = location
    if fixed:
        subject["fixed"] = True
    return {"name": "//p:world", "label": "world", "kind": kinematics.SCENE, "assembly": [subject]}


def cube(mass):
    """What PartCAD hands over for the 10 mm test cube (a corner at its origin) at 'mass' kg."""
    moment = mass * 0.01**2 / 6.0
    return {
        "volume": 1000.0,
        "mass": mass,
        "centerOfMass": [5.0, 5.0, 5.0],
        "inertia": {"ixx": moment, "iyy": moment, "izz": moment, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
    }


def weighed(tree, physics=None, mass=0.1):
    """A properties index giving every part of the tree a mass, 'physics' by part name overriding it."""
    index = {}

    def walk(node):
        if kinematics.is_geometry(node):
            name = node.get("name")
            chosen = next((value for key, value in (physics or {}).items() if str(name).endswith(":" + key)), None)
            index[name] = {"physics": chosen if chosen is not None else cube(mass)}
        for child in node.get("assembly") or []:
            walk(child)

    walk(tree)
    return index


def export(module, tmp_path, tree, name="scene.world", **request):
    path = tmp_path / name
    request.setdefault("properties", weighed(tree))
    result = module.process(str(path), dict({"wrapped": tree}, **request))
    return result, path, ElementTree.parse(str(path)).getroot()


def sdf_joints(sdf):
    """The joints of every model, in order, as (name, type)."""
    return [(joint.get("name"), joint.get("type")) for joint in sdf.iter("joint")]


#
# Reading a world back: SDFormat's own forward kinematics, written from its
# specification rather than from the exporter
#


def _pose(element):
    import gazebo_common

    return gazebo_common.parse_pose(None if element is None else element.find("pose"))


def _numbers(text):
    return [float(v) for v in text.split()]


def sdf_poses(model, coordinates=None):
    """Every link of an SDFormat model in the world, its joints at 'coordinates' (SI, by joint name).

    A link's '<pose>' is in the model's frame; a joint's is relative to its
    child link; its '<axis><xyz>' in the frame 'expressed_in' names. A joint at
    its coordinate turns about (or slides along) that axis through its own
    origin, and SDFormat states every link where its joints are at nought. A
    ball's coordinate is a quaternion (w, x, y, z) in the joint's frame.
    """
    coordinates = coordinates or {}
    origin = _pose(model)
    placed = {link.get("name"): _pose(link) for link in model.findall("link")}
    world = {}
    joints = {joint.findtext("child"): joint for joint in model.findall("joint")}

    def frame_of(name):
        return placed[name] if name in placed else kinematics.IDENTITY

    def resolve(name):
        if name in world:
            return world[name]
        joint = joints.get(name)
        if joint is None or joint.findtext("parent") == "world":
            world[name] = urdf_common.compose(origin, placed[name])
            return world[name]
        parent = joint.findtext("parent")
        at_zero = urdf_common.compose(placed[name], _pose(joint))
        in_parent = urdf_common.compose(kinematics.inverse(placed[parent]), at_zero)
        value = coordinates.get(joint.get("name"))
        motion = kinematics.IDENTITY
        if value is not None:
            axes = []
            for tag in ("axis", "axis2"):
                xyz = joint.find(tag + "/xyz")
                if xyz is None:
                    continue
                vector = _numbers(xyz.text)
                expressed = urdf_common.compose(kinematics.inverse(at_zero), frame_of(xyz.get("expressed_in")))
                axes.append(kinematics.rotate(expressed, vector))
            kind = joint.get("type")
            if kind == "revolute":
                motion = kinematics.motion(kinematics.TURN, axes[0], math.degrees(value))
            elif kind == "prismatic":
                motion = kinematics.motion(kinematics.MOVE, axes[0], value * 1000.0)
            elif kind == "screw":
                pitch = float(
                    joint.findtext("screw_thread_pitch") or -2.0 * math.pi / float(joint.findtext("thread_pitch"))
                )
                turn = kinematics.motion(kinematics.TURN, axes[0], math.degrees(value))
                motion = urdf_common.compose(
                    turn, kinematics.motion(kinematics.MOVE, axes[0], pitch * value / (2 * math.pi) * 1000.0)
                )
            elif kind == "universal":
                motion = urdf_common.compose(
                    kinematics.motion(kinematics.TURN, axes[0], math.degrees(value[0])),
                    kinematics.motion(kinematics.TURN, axes[1], math.degrees(value[1])),
                )
            elif kind == "ball":
                motion = (tuple(value), (0.0, 0.0, 0.0))
        moved = urdf_common.compose(urdf_common.compose(in_parent, motion), kinematics.inverse(_pose(joint)))
        world[name] = urdf_common.compose(resolve(parent), moved)
        return world[name]

    for name in placed:
        resolve(name)
    return world


def _close_pose(a, b, tolerance=1e-9):
    (qa, ta), (qb, tb) = a, b
    same = max(abs(x - y) for x, y in zip(qa, qb))
    flipped = max(abs(x + y) for x, y in zip(qa, qb))
    return min(same, flipped) <= tolerance and max(abs(x - y) for x, y in zip(ta, tb)) <= tolerance * 1000


def _flat_poses(tree):
    """Where the tree puts each part, composed node by node from the root: the assembly as drawn."""
    poses = {}

    def walk(node, pose):
        if kinematics.is_geometry(node):
            poses[id(node)] = pose
        for child in node.get("assembly") or []:
            walk(child, urdf_common.compose(pose, urdf_common.from_packed(child.get("location"))))

    walk(tree, urdf_common.from_packed(tree.get("location")))
    return poses


#
# The classification table, row by row
#

# Every joint SDFormat has a type for, started away from its zero wherever it
# has a parameter to start: (id, what the bore declares, the start, the kind).
# SDFormat states no starting value, so every one of them is written re-zeroed
# at its start.
NATIVE_ROWS = [
    ("revolute", "parameters: {turnZ: [-90, 120]}\nmotion: revolute", ", toParams: {turnZ: 30}", kinematics.REVOLUTE),
    ("revolute-open", "motion: revolute", ", toParams: {turnZ: -40}", kinematics.REVOLUTE),
    ("continuous", "motion: continuous", "", kinematics.REVOLUTE),
    (
        "revolute-one-side",
        "parameters: {turnZ: [-90, .inf]}\nmotion: revolute",
        ", toParams: {turnZ: -30}",
        kinematics.REVOLUTE,
    ),
    ("prismatic", "parameters: {moveZ: [-5, 50]}\nmotion: prismatic", ", toParams: {moveZ: 3}", kinematics.PRISMATIC),
    ("screw", "threadStep: 10\nmotion: screw", ", toParams: {turnZ: 90}", kinematics.SCREW),
    ("universal", "motion: universal", ", toParams: {turnZ: 10, turnY: 20}", kinematics.UNIVERSAL),
    ("ball", "motion: ball", ", toParams: {turnX: 10, turnY: 20, turnZ: 30}", kinematics.BALL),
    ("floating", "motion: floating", ", toParams: {moveZ: 100}", kinematics.FLOATING),
]
# The native ones a joint is written for: all but the floating one, whose child
# is a model of its own.
JOINTED_ROWS = [row for row in NATIVE_ROWS if row[3] != kinematics.FLOATING]

# Every joint SDFormat has no type for, which the export refuses.
REFUSED_ROWS = [
    ("cylindrical", "motion: cylindrical", "", kinematics.CYLINDRICAL),
    ("planar", "motion: planar", "", kinematics.PLANAR),
    # Declared the turn first, then the moves across it: 'dof' is the order.
    ("planar-turn-first", "motion: {dof: [turnZ, moveX, moveY]}", ", toParams: {turnZ: 30}", kinematics.PLANAR),
    (
        "composite",
        "parameters: {turnZ: [-90, 90], moveX: [-5, 5]}\nmotion: {dof: [turnZ, moveX]}",
        ", toParams: {moveX: 2, turnZ: -20}",
        kinematics.COMPOSITE,
    ),
]


def _ids(rows):
    return [row[0] for row in rows]


def _models(sdf):
    return {
        model.get("name"): model for model in sdf.find("world").findall("model") if model.get("name") != "ground_plane"
    }


@pytest.mark.parametrize("label, bore, params, kind", NATIVE_ROWS, ids=_ids(NATIVE_ROWS))
def test_every_native_joint_is_one_sdformat_joint_of_its_own_type(
    tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    tree = rig(tmp_path, bore, params=params)
    arm = kinematics.bodies(tree).by_name("arm")
    assert arm.joint.kind == kind

    result, _path, sdf = export(export_world, tmp_path, tree)

    assert result["success"]
    assert sdf.get("version") == "1.11"
    models = _models(sdf)
    if kind == kinematics.FLOATING:
        # Held by nothing: a model of its own, and no joint anywhere. Nothing
        # moves in either, so a file for looking at has both '<static>'.
        assert sorted(models) == ["arm", "rig"]
        assert sdf_joints(sdf) == []
        assert all(model.findtext("static") == "true" for model in models.values())
    else:
        # The root of an assembly exported on its own is welded to the world
        # (the file is for looking at) -- by a joint, so that its joints still
        # move.
        assert sorted(models) == ["rig"] and models["rig"].find("static") is None
        assert sdf_joints(sdf) == [("rig-world", "fixed"), ("arm-base", export_world.NATIVE[kind])]
        assert [link.get("name") for link in models["rig"].findall("link")] == ["rig_link", "arm"]
    # No link nobody built: every link is a body, with its part in it.
    for model in models.values():
        for link in model.findall("link"):
            assert link.find("visual") is not None and link.find("collision") is not None, link.get("name")
    assert all(joint.get("type") != "continuous" for joint in sdf.iter("joint"))
    # Every axis is expressed in its joint's parent link.
    for joint in sdf.iter("joint"):
        for xyz in joint.findall("axis/xyz") + joint.findall("axis2/xyz"):
            assert xyz.get("expressed_in") == joint.findtext("parent")
    # A start is where the joint is written, so there is nothing to say about one.
    assert not any("start" in warning for warning in result["warnings"])


@pytest.mark.parametrize("label, bore, params, kind", REFUSED_ROWS, ids=_ids(REFUSED_ROWS))
def test_a_joint_gazebo_has_no_joint_for_is_refused_and_nothing_is_written(
    tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    tree = rig(tmp_path, bore, params=params)
    joint = kinematics.bodies(tree).by_name("arm").joint
    assert joint.kind == kind
    if label == "planar-turn-first":
        assert [freedom.kind for freedom in joint.freedoms] == [kinematics.TURN, kinematics.MOVE, kinematics.MOVE]
    out = tmp_path / "out"
    out.mkdir()

    with pytest.raises(export_world.Refused) as refused:
        export_world.process(
            str(out / "scene.world"), {"wrapped": tree, "properties": weighed(tree), "names_file": True}
        )

    assert refused.value.joints == [("arm-base", kind)]
    message = str(refused.value)
    # The joint, its type, the engine, and the two ways on: another engine, or
    # the body the joint stands for, modelled and joined by native joints.
    assert "Gazebo cannot simulate the %s joint 'arm-base'" % kind in message
    assert "MuJoCo" in message and "model the body the joint stands for" in message
    assert "native joints" in message
    # Before anything is written: no world, no sidecar, no meshes' directory.
    assert list(out.iterdir()) == []


TWO_REFUSED = """
links:
  - part: base
  - part: arm
    connect: {name: base, with: pin, to: bore, joint: sleeve}
  - part: lid
    connect: {name: arm, with: pin, to: bore, joint: slider, motion: planar}
"""


def test_every_joint_refused_is_named_at_once(tmp_path, stand_in_geometry, export_world):
    """Including one hanging off another that is refused: a fix at a time is a run at a time."""
    tree = rig(tmp_path, "motion: cylindrical", assy=TWO_REFUSED)

    with pytest.raises(export_world.Refused) as refused:
        export(export_world, tmp_path, tree)

    assert refused.value.joints == [("sleeve", kinematics.CYLINDRICAL), ("slider", kinematics.PLANAR)]
    assert "'sleeve'" in str(refused.value) and "'slider'" in str(refused.value)


def _limit(sdf):
    limit = sdf.find(".//joint[@name='arm-base']/axis/limit")
    return None if limit is None else {child.tag: float(child.text) for child in limit}


def test_a_bound_on_neither_side_or_on_one_is_left_out_rather_than_written_as_a_number(
    tmp_path, stand_in_geometry, export_world
):
    """From SDFormat 1.10 an omitted '<lower>' is -inf and an omitted '<upper>' +inf, which DART takes as no stop."""

    def exported(name, bore):
        tree = rig(tmp_path / name, bore)
        _result, path, sdf = export(export_world, tmp_path, tree, name + ".world")
        return kinematics.bodies(tree).by_name("arm").joint.freedoms[0], sdf, path.read_text(encoding="utf-8")

    _freedom, open_turn, text_open = exported("open", "motion: revolute")
    one, one_side, text_one = exported("one", "parameters: {turnZ: [-30, .inf]}\nmotion: revolute")
    both, two_sides, _text = exported("both", "parameters: {turnZ: [-30, 90]}\nmotion: revolute")
    _freedom, pushed, _text = exported("pushed", "motion: revolute\nphysics: {maxEffort: 3}")

    assert open_turn.find(".//joint[@name='arm-base']").get("type") == "revolute"
    assert _limit(open_turn) is None
    side, bound = ("lower", one.lower) if one.lower is not None else ("upper", one.upper)
    assert _limit(one_side) == {side: pytest.approx(math.radians(bound))}
    assert _limit(two_sides) == {
        "lower": pytest.approx(math.radians(both.lower)),
        "upper": pytest.approx(math.radians(both.upper)),
    }
    # A '<limit>' for its effort alone says nothing about where the joint stops.
    assert _limit(pushed) == {"effort": 3.0}
    for text in (text_open, text_one):
        assert "1e+16" not in text and "1e16" not in text


def test_a_joint_is_written_at_its_start_and_its_bounds_about_it(tmp_path, stand_in_geometry, export_world):
    tree = rig(tmp_path, "parameters: {turnZ: [-90, 120]}\nmotion: revolute", params=", toParams: {turnZ: 30}")
    result, _path, sdf = export(export_world, tmp_path, tree)

    arm = kinematics.bodies(tree).by_name("arm")
    freedom = arm.joint.freedoms[0]
    assert (freedom.value, freedom.lower, freedom.upper) == (30.0, -90.0, 120.0)
    # At Gazebo's zero the arm is where the assembly shows it, 30 degrees round.
    assert _close_pose(sdf_poses(sdf.find("world/model[@name='rig']"))["arm"], arm.world)
    assert _limit(sdf) == {"lower": pytest.approx(math.radians(-120.0)), "upper": pytest.approx(math.radians(90.0))}
    assert not any("start" in warning for warning in result["warnings"])


def test_a_start_outside_its_bounds_is_reported(tmp_path, stand_in_geometry, export_world):
    """Gazebo would throw such a joint against its stop at once: PartCAD's body rule says so, and the export with it.

    'limits:' on a target interface are about its port's Z, the contact
    frame's reversed: 'upper: 30' bounds the freedom below at -30, and -90 is
    under it.
    """
    tree = rig(tmp_path, "motion: {type: revolute, limits: {upper: 30}}", params=", toParams: {turnZ: -90}")
    found = kinematics.bodies(tree)
    (problem,) = [problem for problem in found.problems if "outside its bounds" in problem]
    result, _path, _sdf = export(export_world, tmp_path, tree)
    assert problem in result["warnings"]
    assert "'arm-base'" in problem and "-90 deg" in problem


def test_a_joint_s_physics_are_its_dynamics_and_limits_in_si_about_its_start(tmp_path, stand_in_geometry, export_world):
    tree = rig(
        tmp_path,
        "parameters: {moveZ: [0, 50]}\nmotion: prismatic\n"
        "physics: {damping: 4, friction: 0.5, springStiffness: 100, springReference: 20, maxEffort: 30, "
        "maxVelocity: 250, erp: 0.2}",
        params=", toParams: {moveZ: 5}",
    )
    result, _path, sdf = export(export_world, tmp_path, tree)

    axis = sdf.find(".//joint[@name='arm-base']/axis")
    assert float(axis.findtext("dynamics/damping")) == pytest.approx(4.0)
    assert float(axis.findtext("dynamics/friction")) == pytest.approx(0.5)
    assert float(axis.findtext("dynamics/spring_stiffness")) == pytest.approx(100.0)
    # Positions of the joint: millimetres to metres, along the same axis, and
    # about the start -- the spring's reference too.
    joint = kinematics.bodies(tree).by_name("arm").joint
    freedom = joint.freedoms[0]
    assert abs(freedom.value) == pytest.approx(5.0)
    shifted = kinematics.rezeroed(joint)
    assert float(axis.findtext("dynamics/spring_reference")) == pytest.approx(shifted.spring_reference(0) / 1000.0)
    assert shifted.spring_reference(0) == pytest.approx(20.0 - freedom.value)
    assert float(axis.findtext("limit/effort")) == pytest.approx(30.0)
    assert float(axis.findtext("limit/velocity")) == pytest.approx(0.25)
    assert _limit(sdf)["lower"] == pytest.approx((freedom.lower - freedom.value) / 1000.0)
    assert _limit(sdf)["upper"] == pytest.approx((freedom.upper - freedom.value) / 1000.0)
    assert "joint erp" in result["unsupported"]


def test_a_ball_states_its_damping_and_reports_what_it_cannot_say(tmp_path, stand_in_geometry, export_world):
    """DART applies a ball's '<axis>' to each of its three coordinates: a damper there is a damper, the rest a box."""
    tree = rig(
        tmp_path,
        "motion: {type: ball, limits: {upper: 20}}\nphysics: {damping: 0.3, friction: 0.1, springStiffness: 2}",
    )
    assert kinematics.bodies(tree).by_name("arm").joint.turn_limit == 20.0
    result, _path, sdf = export(export_world, tmp_path, tree)

    joint = sdf.find(".//joint[@name='arm-base']")
    assert joint.get("type") == "ball"
    assert float(joint.findtext("axis/dynamics/damping")) == pytest.approx(0.3)
    assert joint.find("axis/dynamics/friction") is None and joint.find("axis/dynamics/spring_stiffness") is None
    assert joint.find("axis/limit") is None and joint.find("axis2") is None
    assert any("'arm-base'" in warning and "friction, springStiffness" in warning for warning in result["warnings"])
    assert any("bounded at 20 deg" in warning and "cone" in warning for warning in result["warnings"])
    # And a ball with nothing to state on its '<axis>' has none.
    _result, _path, plain = export(export_world, tmp_path, rig(tmp_path / "plain", "motion: ball"), "plain.world")
    assert plain.find(".//joint[@name='arm-base']/axis") is None


def test_a_screw_states_its_thread_in_the_units_of_the_version_written(tmp_path, stand_in_geometry, export_world):
    """10 mm a turn: radians a metre, left-handed positive, in 1.9; metres a turn, right-handed, from 1.10."""
    tree = rig(tmp_path, "threadStep: 10\nmotion: screw")
    _result, _path, old = export(export_world, tmp_path, tree, "a.world", version="1.9")
    _result, _path, new = export(export_world, tmp_path, tree, "b.world")

    radians_a_metre = float(old.findtext(".//joint[@name='arm-base']/thread_pitch"))
    metres_a_turn = float(new.findtext(".//joint[@name='arm-base']/screw_thread_pitch"))
    assert abs(metres_a_turn) == pytest.approx(0.01)
    assert radians_a_metre == pytest.approx(-2 * math.pi / metres_a_turn)
    assert old.find(".//screw_thread_pitch") is None and new.find(".//thread_pitch") is None


MIMIC = """
links:
  - part: base
  - part: arm
    connect: {name: base, with: pin, to: bore, joint: drive, toParams: {turnZ: 30}}
  - part: lid
    connect:
      name: arm
      with: pin
      to: bore
      joint: follower
      toParams: {turnZ: -10}
      motion: {type: revolute, mimic: {joint: drive, multiplier: -2, offset: 15}}
"""


def test_a_mimic_is_written_about_both_joints_starts_and_reported_where_the_version_has_none(
    tmp_path, stand_in_geometry, export_world
):
    tree = rig(tmp_path, "motion: revolute", assy=MIMIC)
    _result, _path, new = export(export_world, tmp_path, tree, "a.world")
    result, _path, old = export(export_world, tmp_path, tree, "b.world", version="1.10")

    assert old.find(".//mimic") is None
    assert any("'follower' mimics 'drive'" in warning and "1.11" in warning for warning in result["warnings"])
    mimic = new.find(".//joint[@name='follower']/axis/mimic")
    assert mimic.get("joint") == "drive" and mimic.get("axis") == "axis"
    assert float(mimic.findtext("multiplier")) == pytest.approx(-2.0)
    # 'v_f = m v_l + c' is 'e_f = m e_l + (c + m s_l - s_f)' once both count from their starts.
    found = kinematics.bodies(tree)
    leading, following = found.by_name("arm").joint.freedoms[0], found.by_name("lid").joint.freedoms[0]
    assert (leading.value, following.value) == (30.0, -10.0)
    offset = kinematics.rezeroed_offset(15.0, -2.0, leading.value, following.value)
    assert offset == pytest.approx(15.0 - 60.0 + 10.0)
    assert float(mimic.findtext("offset")) == pytest.approx(math.radians(offset))


def test_a_free_root_is_free_to_move_only_when_asked_and_a_fixed_one_never(tmp_path, stand_in_geometry, export_world):
    tree = rig(tmp_path, "motion: revolute")
    free = in_a_scene(tree, fixed=False)
    bolted = in_a_scene(tree, fixed=True)

    _result, _path, looked_at = export(export_world, tmp_path, free, "a.world")
    _result, _path, simulated = export(export_world, tmp_path, free, "b.world", static=False)
    _result, _path, fixed = export(export_world, tmp_path, bolted, "c.world", static=False)

    assert sdf_joints(looked_at) == [("subject-world", "fixed"), ("arm-base", "revolute")]
    assert sdf_joints(simulated) == [("arm-base", "revolute")]
    assert sdf_joints(fixed) == [("subject-world", "fixed"), ("arm-base", "revolute")]
    for sdf in (looked_at, simulated, fixed):
        assert sdf.find("world/model[@name='subject']/static") is None
        assert sdf.find("world/model[@name='subject']/joint[@type='fixed']/parent") is None or (
            sdf.find("world/model[@name='subject']/joint[@type='fixed']/parent").text == "world"
        )


def test_flatten_is_reported_and_ignored(tmp_path, stand_in_geometry, export_world):
    result, _path, _sdf = export(export_world, tmp_path, rig(tmp_path, "motion: revolute"), flatten=True)
    assert any("'flatten' is no longer an option" in warning for warning in result["warnings"])


#
# Placements: where SDFormat puts every link is where PartCAD says it is
#

SCENE_LOCATION = [[20.0, -30.0, 500.0], [1.0, 0.0, 0.0], 90.0]


def _moved(joint):
    """Gazebo's coordinates of 'joint' moved off its zero -- its start -- and the PartCAD values that puts it at.

    A coordinate in SDFormat's units, as 'sdf_poses' takes it: radians or
    metres, a pair for a universal joint, and a quaternion in the ball's own
    frame for a ball.
    """
    starts = [freedom.value for freedom in joint.freedoms]
    if joint.kind == kinematics.BALL:
        rotation = urdf_common.axis_angle_to_quat((0.3, -0.5, 0.8), 40.0)
        total = urdf_common.quat_mul(kinematics.rezeroed(joint).rotation, rotation)
        return rotation, kinematics.turns_of(total, joint.classification.axes, starts)
    if joint.kind == kinematics.UNIVERSAL:
        steps = [17.0, -11.0]
        return [math.radians(step) for step in steps], [start + step for start, step in zip(starts, steps)]
    if joint.kind == kinematics.SCREW:
        turn = next(freedom for freedom in joint.freedoms if freedom.kind == kinematics.TURN)
        move = next(freedom for freedom in joint.freedoms if freedom.kind == kinematics.MOVE)
        values = list(starts)
        values[turn.index] += 17.0
        values[move.index] = move.ratio * values[turn.index] + move.offset
        return math.radians(17.0), values
    (freedom,) = joint.freedoms
    step = 17.0 if freedom.kind == kinematics.TURN else 3.0
    return step / kinematics.per_si(freedom.kind), [freedom.value + step]


@pytest.mark.parametrize("label, bore, params, kind", JOINTED_ROWS, ids=_ids(JOINTED_ROWS))
def test_at_gazebo_s_zero_every_part_is_where_the_assembly_shows_it_and_moved_where_partcad_puts_it(
    tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    """Re-zeroed: Gazebo's zero of every joint is its start, and Gazebo's 'e' is PartCAD's 's + e'.

    Read back by SDFormat's own forward kinematics ('sdf_poses'), not by
    anything the exporter computed.
    """
    tree = in_a_scene(rig(tmp_path, bore, params=params), SCENE_LOCATION)
    found = kinematics.bodies(tree)
    _result, _path, sdf = export(export_world, tmp_path, tree, static=False)
    model = sdf.find("world/model[@name='subject']")
    links = {body.name: ("subject_link" if body.is_root else body.name) for body in found}

    at_zero = sdf_poses(model)
    flat = _flat_poses(tree)
    for body in found:
        assert _close_pose(at_zero[links[body.name]], body.world), body.name
        link = model.find("link[@name='%s']" % links[body.name])
        for index, part in enumerate(body.parts):
            visual = link.find("visual[@name='%s_visual_%d']" % (links[body.name], index))
            placed = urdf_common.compose(at_zero[links[body.name]], _pose(visual))
            # The assembly as drawn, which is where every joint starts.
            assert _close_pose(placed, flat[id(part.node)]), (body.name, part.label)

    joint = found.by_name("arm").joint
    coordinates, values = _moved(joint)
    moved = sdf_poses(model, {"arm-base": coordinates})
    expected = kinematics.compose(found.by_name("subject").world, joint.location(values))
    assert _close_pose(moved["arm"], expected, 1e-7), kind


def test_a_floating_child_is_a_model_of_its_own_at_its_start_with_no_joint(tmp_path, stand_in_geometry, export_world):
    tree = in_a_scene(rig(tmp_path, "motion: floating", params=", toParams: {moveZ: 100}"), SCENE_LOCATION)
    found = kinematics.bodies(tree)
    arm = found.by_name("arm")
    assert arm.is_floating and arm.parent is None and arm.joint_parent is found.by_name("subject")

    _result, path, sdf = export(export_world, tmp_path, tree, static=False, names_file=True)

    model = sdf.find("world/model[@name='arm']")
    assert model.find("joint") is None and model.find("static") is None
    assert [link.get("name") for link in model.findall("link")] == ["arm_link"]
    assert sdf_joints(sdf) == [("subject-world", "fixed")]
    # At its start: 100 mm along its connection from where its zero would be.
    assert _close_pose(sdf_poses(model)["arm_link"], arm.world)
    zero = kinematics.compose(found.by_name("subject").world, arm.joint.location([0.0] * 6))
    assert math.dist(zero[1], arm.world[1]) == pytest.approx(100.0)
    # What a run reads it by: the body, and the map that says what against.
    entry = naming.read_sidecar(str(path))["joints"]["arm-base"]
    assert entry == {"joint": "arm-base", "body": "arm", "map": json.loads(json.dumps(kinematics.joint_map(arm.joint)))}
    assert entry["map"]["parent"] == "subject"


@pytest.mark.parametrize("label, bore, params, kind", JOINTED_ROWS, ids=_ids(JOINTED_ROWS))
def test_the_sidecar_carries_each_joint_s_map_and_gazebo_s_zero_reads_its_start(
    tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    tree = in_a_scene(rig(tmp_path, bore, params=params), SCENE_LOCATION)
    joint = kinematics.bodies(tree).by_name("arm").joint
    _result, path, _sdf = export(export_world, tmp_path, tree, static=False, names_file=True)

    entry = naming.read_sidecar(str(path))["joints"]["arm-base"]
    assert entry["path"] == ["subject", "arm-base"] and entry["joint"] == "arm-base"
    assert entry["map"]["kind"] == kind and entry["map"]["joint"] == "arm-base"
    starts = dict(zip(joint.parameter_names(), [freedom.value for freedom in joint.freedoms]))
    if kind == kinematics.BALL:
        # Read from its links: the joint's frame in each, one frame at the start.
        assert sorted(entry["ball"]) == ["child", "in_child", "in_parent", "parent"]
        assert (entry["ball"]["parent"], entry["ball"]["child"]) == ("subject", "arm")
        reading = kinematics.reading([entry], [urdf_common.IDENTITY_Q])
    else:
        reading = kinematics.reading([entry], [[0.0] * len(entry["map"]["coordinates"])])
    pos = reading["pos"] if isinstance(reading["pos"], dict) else {joint.parameter_names()[0]: reading["pos"]}
    assert pos == pytest.approx(starts)
    assert reading["type"] == kind


#
# In a real Gazebo
#


@pytest.fixture(scope="module")
def gazebo():
    import simulate_gazebo

    try:
        return simulate_gazebo.locate_gazebo()
    except simulate_gazebo.GazeboMissing:
        pytest.skip("no Gazebo here; see this file's docstring for running it in the simulation's image")


@pytest.mark.parametrize("label, bore, params, kind", NATIVE_ROWS, ids=_ids(NATIVE_ROWS))
def test_in_gazebo_every_native_joint_is_a_world_sdformat_accepts(
    gazebo, tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    (binary, _args, _topic), environment = gazebo
    tree = in_a_scene(rig(tmp_path, bore, params=params), SCENE_LOCATION)
    _result, path, _sdf = export(export_world, tmp_path, tree, static=False, self_collide=True)

    checked = subprocess.run([binary, "sdf", "-k", str(path)], capture_output=True, text=True, env=environment)
    assert checked.returncode == 0 and "Valid" in checked.stdout, checked.stdout + checked.stderr


# The pendulum: a bar 300 mm long and 200 g, hung from one end on the hinge.
# Its centre of mass is half-way along, and about the hinge its moment of
# inertia is a thin rod's: m L^2 / 3. The MJCF exporter's, for comparison.
BAR_MASS, BAR_LENGTH = 0.2, 0.3
BAR_D = BAR_LENGTH / 2.0
BAR_I_PIVOT = BAR_MASS * BAR_LENGTH**2 / 3.0


def bar():
    """The bar's physics as PartCAD states them: along the arm's own X, from the hinge's axis."""
    across = BAR_MASS * BAR_LENGTH**2 / 12.0
    return {
        "volume": 1000.0,
        "mass": BAR_MASS,
        "centerOfMass": [BAR_D * 1000.0, 0.0, 0.0],
        "inertia": {"ixx": 1e-7, "iyy": across, "izz": across, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
    }


def lopsided():
    """A body whose centre of mass is off every axis a joint of the rig turns about, so that gravity turns them all.

    The bar's lies on the line of a universal joint's second turn, which
    gravity then never turns.
    """
    return {
        "volume": 1000.0,
        "mass": 0.2,
        "centerOfMass": [100.0, 70.0, 40.0],
        "inertia": {"ixx": 5e-4, "iyy": 6e-4, "izz": 7e-4, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
    }


def _complete_elliptic_k(k):
    """K(k), the complete elliptic integral of the first kind, by the arithmetic-geometric mean."""
    a, b = 1.0, math.sqrt(1.0 - k * k)
    while abs(a - b) > 1e-15:
        a, b = (a + b) / 2.0, math.sqrt(a * b)
    return math.pi / (2.0 * a)


# Where a run's scene puts the rig: half a metre up and turned, so that every
# frame along the way is one that is not the world's.
PLACED = [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 30.0]
# A gravity that is along no axis of anything, so that it turns every freedom.
OBLIQUE = [3.0, -4.0, -9.0]
# Gazebo's step, which the worlds here leave as it is: a millisecond.
STEP = 0.001


def _run(module, directory, tree, physics=None, **request):
    """'tree' exported as a run exports it, and run in Gazebo: what 'pc sim' hands the validation."""
    import simulate_gazebo

    _result, path, _sdf = module_export(module, directory, tree, physics)
    return simulate_gazebo.process(str(directory), dict({"scene_file": str(path)}, **request))


def module_export(module, directory, tree, physics=None):
    return export(
        module,
        directory,
        tree,
        "run.world",
        static=False,
        self_collide=True,
        ground_plane=False,
        names_file=True,
        properties=weighed(tree, {"arm": physics or bar()}),
    )


def _values(joint, state):
    """The values of a joint's freedoms, by index, out of what a run reported for it."""
    names = joint.parameter_names()
    pos = state["pos"] if isinstance(state["pos"], dict) else {names[0]: state["pos"]}
    return [pos[name] for name in names]


def _body(reading, name):
    body = reading["bodies"][name]
    return tuple(body["quat"]), tuple(body["pos"])


def _readings(result):
    """Every reading of a run, once each, in order."""
    seen, out = set(), []
    for reading in [result["before"]] + result.get("samples", []) + [result["after"]]:
        if reading["time"] not in seen:
            seen.add(reading["time"])
            out.append(reading)
    return out


@pytest.mark.parametrize("label, bore, params, kind", NATIVE_ROWS, ids=_ids(NATIVE_ROWS))
def test_in_gazebo_a_joint_started_away_from_its_zero_reads_its_start(
    gazebo, tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    """With nothing to move it, every joint reads its start -- not nought, Gazebo's -- and every body is there."""
    tree = in_a_scene(rig(tmp_path, bore, params=params), PLACED)
    found = kinematics.bodies(tree)
    joint = found.by_name("arm").joint
    result = _run(export_world, tmp_path, tree, duration=0.1, gravity=[0.0, 0.0, 0.0])

    starts = [freedom.value for freedom in joint.freedoms]
    # The world is read before it runs: the first reading is at t = 0.
    assert result["before"]["time"] == 0.0
    for reading in (result["before"], result["after"]):
        state = reading["joints"]["arm-base"]
        assert state["type"] == kind
        assert _values(joint, state) == pytest.approx(starts, abs=1e-6)
        if kind in (kinematics.BALL, kinematics.FLOATING):
            assert "quat" in state
        for body in found:
            assert _close_pose(_body(reading, body.name), body.world, 1e-6), body.name
    assert "warnings" not in result


@pytest.mark.parametrize("label, bore, params, kind", NATIVE_ROWS, ids=_ids(NATIVE_ROWS))
def test_in_gazebo_every_reading_puts_the_body_where_gazebo_has_it(
    gazebo, tmp_path, stand_in_geometry, export_world, label, bore, params, kind
):
    """The readings round trip: what a run reports, put through PartCAD's own joint, is where Gazebo has the body.

    Under a gravity along no axis, so that every freedom moves; at every
    reading, the child's pose is the pose of the body it moves against times
    'joint.location()' of the values reported. Only a joint Gazebo built where
    the export says, re-zeroed as the map says, passes that -- and only one
    that moves passes the check that it moved, which a weld would fail.
    """
    tree = in_a_scene(rig(tmp_path, bore, params=params), PLACED)
    found = kinematics.bodies(tree)
    arm = found.by_name("arm")
    joint = arm.joint
    result = _run(export_world, tmp_path, tree, lopsided(), duration=0.4, samples=24, gravity=OBLIQUE)

    readings = _readings(result)
    for reading in readings:
        values = _values(joint, reading["joints"]["arm-base"])
        expected = kinematics.compose(_body(reading, arm.joint_parent.name), joint.location(values))
        assert _close_pose(_body(reading, "arm"), expected, 1e-6), (kind, reading["time"])
    starts = [freedom.value for freedom in joint.freedoms]
    ended = _values(joint, result["after"]["joints"]["arm-base"])
    moved = [abs(a - b) for a, b in zip(ended, starts)]
    if kind == kinematics.SCREW:
        assert moved[0] > 5.0
    elif kind == kinematics.FLOATING:
        assert max(moved) > 5.0
    else:
        assert min(moved) > 5.0, moved
    print("%s: %s from %s" % (label, [round(value, 3) for value in ended], starts))
    if kind == kinematics.BALL:
        _check_ball_speed(readings)


def _check_ball_speed(readings):
    """A ball's 'vel' is the turn its rotation makes, over time: held to the readings either side of each."""

    def turned(earlier, later):
        """The turn from one reading's rotation to the next's, degrees about the axes of the first."""
        first, second = (tuple(r["joints"]["arm-base"]["quat"]) for r in (earlier, later))
        turn = urdf_common.quat_mul((first[0], -first[1], -first[2], -first[3]), second)
        if turn[0] < 0:
            turn = tuple(-v for v in turn)
        along = math.sqrt(sum(v * v for v in turn[1:]))
        if along < 1e-15:
            return [0.0, 0.0, 0.0]
        angle = math.degrees(2.0 * math.atan2(along, turn[0]))
        return [v / along * angle for v in turn[1:]]

    for before, reading, after in zip(readings, readings[1:], readings[2:]):
        # The three-point difference for readings unevenly spaced: second order
        # in the spacing, as the even one is.
        h1, h2 = reading["time"] - before["time"], after["time"] - reading["time"]
        back, ahead = turned(before, reading), turned(reading, after)
        rate = [(h1 * h1 * a + h2 * h2 * b) / (h1 * h2 * (h1 + h2)) for b, a in zip(back, ahead)]
        reported = reading["joints"]["arm-base"]["vel"]
        size = math.sqrt(sum(v * v for v in rate))
        # The difference over two pose messages, a thirtieth of a second, is
        # good to a per cent or so of a swing; the reading's is over two steps.
        assert max(abs(a - b) for a, b in zip(reported, rate)) <= 0.03 * size + 3.0, (reading["time"], reported, rate)


@pytest.fixture(scope="module")
def swung(gazebo, tmp_path_factory):
    """The bar released horizontal on a revolute joint about a horizontal axis, run in Gazebo for 1.5 s.

    The hinge is the bore's Z axis, which the scene turns to lie along world Y,
    so that the bar, along the arm's X, starts horizontal at nought.
    """
    return _pendulum(tmp_path_factory, "pendulum", "", duration=1.5, samples=299)


@pytest.fixture(scope="module")
def wound(gazebo, tmp_path_factory):
    """The same bar started half a turn round: horizontal the other way, on a hinge bounded on neither side."""
    return _pendulum(tmp_path_factory, "wound", ", toParams: {turnZ: 180}", duration=0.5, samples=99)


def _pendulum(tmp_path_factory, name, params, **request):
    import _pytest.monkeypatch

    import simulate_gazebo

    patch = _pytest.monkeypatch.MonkeyPatch()
    try:

        async def part(self, ctx):
            return dict(self.get_cache_metadata(), brep=b"BREP")

        async def assembly(self, ctx):
            await self.do_instantiate()
            return await self._get_shape_real(ctx)

        patch.setattr(pc.Part, "get_wrapped", part)
        patch.setattr(pc.Assembly, "get_wrapped", assembly)
        import export_world as module
        import ocp_serialize

        patch.setattr(ocp_serialize, "decode_shape", lambda obj: ("shape", obj.get("brep")))
        patch.setattr(module, "write_mesh", lambda shape, path, options: shutil.copyfile(CUBE_STL, path))

        directory = tmp_path_factory.mktemp(name)
        tree = in_a_scene(rig(directory, "motion: revolute", params=params), [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0])
        _result, path, _sdf = module_export(module, directory, tree)
    finally:
        patch.undo()
    return simulate_gazebo.process(str(directory), dict({"scene_file": str(path)}, **request))


def test_in_gazebo_a_pendulum_released_horizontal_passes_the_bottom_at_the_speed_its_fall_gives_it(swung):
    """omega = sqrt(2 m g d / I_pivot): all of m g d turned into (1/2) I omega^2."""
    expected = math.degrees(math.sqrt(2.0 * BAR_MASS * G * BAR_D / BAR_I_PIVOT))
    readings = [sample["joints"]["arm-base"] for sample in swung["samples"]]

    # Released at nought, and read there: the world is started paused and
    # read before it runs, so the first reading is at t = 0, at rest.
    before = swung["before"]
    assert before["time"] == 0.0
    assert before["joints"]["arm-base"]["pos"] == pytest.approx(0.0, abs=1e-9)
    assert before["joints"]["arm-base"]["vel"] == pytest.approx(0.0, abs=1e-9)
    # Every sample asked for came back, each the first pose message at or past
    # its instant that no sample before it took -- the run paced to give one
    # for each -- and none of them later than the instant after its own: the
    # pose messages come by the wall clock, a step or so either way.
    assert len(swung["samples"]) == 299
    every = 1.5 / 300
    times = [sample["time"] for sample in swung["samples"]]
    assert times == sorted(set(times))
    for index, time_ in enumerate(times):
        assert (index + 1) * every - 1e-9 <= time_ <= (index + 2) * every + STEP + 1e-9, index
    # A hinge bounded on neither side is a revolute, as PartCAD calls it.
    assert swung["before"]["joints"]["arm-base"]["type"] == "revolute"
    fastest = max(readings, key=lambda reading: abs(reading["vel"]))
    print(
        "pendulum: fastest %.3f deg/s at %.2f deg, against %.3f deg/s" % (abs(fastest["vel"]), fastest["pos"], expected)
    )
    assert abs(fastest["vel"]) == pytest.approx(expected, rel=0.01)
    # Within a reading or two of the bottom: near it the speed hardly changes,
    # and the readings are a few degrees apart there.
    assert abs(fastest["pos"]) == pytest.approx(90.0, abs=5.0)
    # And at every reading once under way, what it has fallen is what it moves
    # with: no energy lost, and none made, by the joint.
    for sample in swung["samples"]:
        if sample["time"] < 0.1:
            continue
        reading = sample["joints"]["arm-base"]
        fallen = BAR_MASS * G * BAR_D * math.sin(math.radians(abs(reading["pos"])))
        moving = 0.5 * BAR_I_PIVOT * math.radians(reading["vel"]) ** 2
        assert moving == pytest.approx(fallen, rel=2e-2, abs=2e-3), sample["time"]


def test_in_gazebo_a_pendulum_released_horizontal_swings_with_the_period_of_one(swung):
    """T = 4 K(sin 45 deg) sqrt(I_pivot / (m g d)): a right-angle swing, by the complete elliptic integral."""
    expected = (
        4.0 * _complete_elliptic_k(math.sin(math.radians(45.0))) * math.sqrt(BAR_I_PIVOT / (BAR_MASS * G * BAR_D))
    )
    samples = [(sample["time"], sample["joints"]["arm-base"]["pos"]) for sample in swung["samples"]]
    bottom = 90.0 if samples[len(samples) // 20][1] > 0 else -90.0

    crossings = []
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        o0, o1 = (p0 - bottom) * math.copysign(1.0, bottom), (p1 - bottom) * math.copysign(1.0, bottom)
        if o0 < 0.0 <= o1:
            # Linearly between the two samples either side of it, where the bar
            # moves fastest and so most nearly linearly.
            crossings.append(t0 + (t1 - t0) * (-o0) / (o1 - o0))
    print("pendulum: period %s, against %.5f s" % ([b - a for a, b in zip(crossings, crossings[1:])], expected))
    assert len(crossings) >= 2
    assert crossings[1] - crossings[0] == pytest.approx(expected, rel=0.01)


def test_in_gazebo_a_pendulum_started_half_a_turn_round_reads_its_start_and_its_bottom_unwrapped(wound):
    """It reads 180 at the start, not Gazebo's nought, and passes the bottom at 90 or 270 -- never at -90.

    A lost start would read the bottom at -90 or 90 from nought, and a reading
    wrapped into +-180 would read 270 as -90; the speed there is the bar's
    from 'swung', sqrt(2 m g d / I_pivot), since it falls as far.
    """
    expected = math.degrees(math.sqrt(2.0 * BAR_MASS * G * BAR_D / BAR_I_PIVOT))
    before = wound["before"]["joints"]["arm-base"]
    # At t = 0, exactly where it was started.
    assert wound["before"]["time"] == 0.0
    assert before["pos"] == pytest.approx(180.0, abs=1e-9)
    fastest = max((sample["joints"]["arm-base"] for sample in wound["samples"]), key=lambda state: abs(state["vel"]))
    print("wound: fastest %.3f deg/s at %.2f deg, against %.3f deg/s" % (abs(fastest["vel"]), fastest["pos"], expected))
    assert abs(fastest["vel"]) == pytest.approx(expected, rel=0.01)
    assert min(abs(fastest["pos"] - 90.0), abs(fastest["pos"] - 270.0)) <= 5.0


def test_in_gazebo_a_bound_stops_a_joint_where_partcad_puts_it_not_where_its_start_would(
    gazebo, tmp_path, stand_in_geometry, export_world
):
    """The bar of 'swung' on a hinge bounded at +-45 degrees, started at 20: it falls to the stop, at 45 or -45.

    Bounds written about PartCAD's zero rather than the start would stop it
    at 65 or -25.
    """
    tree = in_a_scene(
        rig(tmp_path, "parameters: {turnZ: [-45, 45]}\nmotion: revolute", params=", toParams: {turnZ: 20}"),
        [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0],
    )
    result = _run(export_world, tmp_path, tree, duration=0.6, samples=60, gravity=[0.0, 0.0, -G])

    readings = [reading["joints"]["arm-base"]["pos"] for reading in _readings(result)]
    # Started at 20, and a few steps under way at the first reading.
    assert readings[0] == pytest.approx(20.0, abs=2.0)
    stop = 45.0 if readings[-1] > 0 else -45.0
    print("stopped: %.4f deg, the stop at %g" % (readings[-1], stop))
    assert readings[-1] == pytest.approx(stop, abs=0.5)
    assert max(abs(value) for value in readings) <= 45.5


def test_in_gazebo_a_carriage_falls_from_its_start_to_its_stop(gazebo, tmp_path, stand_in_geometry, export_world):
    """A slide bounded at [0, 100] mm, started at 50 and pulled toward 0: 50 - g t^2 / 2, and then 0 for good.

    Bounds written about PartCAD's zero rather than the start would hold it
    where it started, against a stop at nought.
    """
    tree = in_a_scene(
        rig(tmp_path, "parameters: {moveZ: [0, 100]}\nmotion: prismatic", params=", toParams: {moveZ: 50}"),
        [[0.0, 0.0, 500.0], [0.0, 0.0, 1.0], 0.0],
    )
    found = kinematics.bodies(tree)
    joint = found.by_name("arm").joint
    (freedom,) = joint.freedoms
    assert (freedom.value, freedom.lower, freedom.upper) == (50.0, 0.0, 100.0)
    # The world's direction of the slide's +1 mm, so that gravity can be set
    # against it.
    along = kinematics.rotate(kinematics.compose(found.by_name("subject").world, joint.frame(0)), freedom.axis)
    result = _run(export_world, tmp_path, tree, duration=0.4, samples=40, gravity=[-G * v for v in along])

    for reading in _readings(result):
        t, pos = reading["time"], reading["joints"]["arm-base"]["pos"]
        falling = 50.0 - 0.5 * G * 1000.0 * t * t
        if falling > 5.0:
            # A step of Gazebo's semi-implicit Euler runs a step ahead.
            assert pos == pytest.approx(falling, abs=0.5), t
        elif falling < -20.0:
            assert pos == pytest.approx(0.0, abs=0.5), t
    print("carriage: at %.4f mm after %.2f s" % (result["after"]["joints"]["arm-base"]["pos"], result["after"]["time"]))


def test_in_gazebo_a_floating_child_falls_free_from_its_start(gazebo, tmp_path, stand_in_geometry, export_world):
    """Its own model, held by nothing: g t^2 / 2 from where it started, 49.05 mm in 0.1 s.

    To a step: Gazebo's semi-implicit Euler moves a body by its new speed, so
    after n steps of dt it has fallen g dt^2 n (n + 1) / 2, which is
    g t (t + dt) / 2.
    """
    tree = in_a_scene(rig(tmp_path, "motion: floating", params=", toParams: {moveZ: 100}"), PLACED)
    arm = kinematics.bodies(tree).by_name("arm")
    result = _run(export_world, tmp_path, tree, duration=0.15, samples=15, gravity=[0.0, 0.0, -G])

    start = arm.world[1][2]
    moves = [name for name, freedom in zip(arm.joint.parameter_names(), arm.joint.freedoms) if freedom.kind == "move"]
    for reading in _readings(result):
        t = reading["time"]
        fallen = 0.5 * G * 1000.0 * t * (t + STEP)
        assert reading["bodies"]["arm"]["pos"][2] == pytest.approx(start - fallen, abs=0.01), t
        state = reading["joints"]["arm-base"]
        assert state["type"] == kinematics.FLOATING
        # Its speed, against a base that does not move, along the three moves
        # (square to each other): g t, exactly. It is read as what the last
        # step moved it by, which is the speed semi-implicit Euler ends a step
        # with -- the engine's own -- where a difference across the reading
        # would be the speed half a step on.
        # At rest at t = 0, where the world is read before it runs.
        speed = math.sqrt(sum(state["vel"][name] ** 2 for name in moves))
        assert speed == pytest.approx(G * 1000.0 * t, rel=1e-6, abs=1e-6), t
        # And it falls without turning.
        assert max(abs(v) for v in state["vel"]["angular"]) < 1e-6
    print(
        "floating: fell %.3f mm in %.3f s, at %.4f mm/s"
        % (
            start - result["after"]["bodies"]["arm"]["pos"][2],
            result["after"]["time"],
            math.sqrt(sum(result["after"]["joints"]["arm-base"]["vel"][name] ** 2 for name in moves)),
        )
    )


FLOATING_OFF_A_SWING = """
links:
  - part: base
  - part: arm
    connect: {name: base, with: pin, to: bore, joint: swing}
  - part: lid
    connect: {name: arm, with: pin, to: bore, joint: loose, motion: floating, toParams: {moveZ: -200}}
"""


def test_in_gazebo_a_floating_child_s_rates_against_a_body_that_turns_are_the_engine_s(
    gazebo, tmp_path, stand_in_geometry, export_world
):
    """A lid falling free beside the bar of 'swung' as it swings: read against the bar, its freedoms all have rates.

    Each is what the last step moved the two bodies by, put through PartCAD's
    'kinematics.reading'. Held to answers that do not come from that:

    * the lid turns against the bar at the bar's own speed, the other way --
      it falls without turning -- which is what Gazebo's joint state says of
      the hinge;
    * every rate is what the same 'kinematics.reading' makes of the two
      bodies' velocities as the engine holds them: the lid falling at exactly
      g t (semi-implicit Euler), the bar turning about its hinge, through its
      own origin, at the speed its joint state reports;
    * and, loosely, each rate is the derivative of the lid's readings
      themselves, a few milliseconds apart -- the slope of the quartic
      through five of them, half a step back. Only loosely: the
      engine's speeds are half a step behind its positions, and a rate
      against a turning body is a speed taken at one position, which no curve
      through the positions has as its derivative; the two differ by about
      the bar's (angular speed squared plus angular acceleration) times the
      lid's distance times half a step, ten or twenty mm/s by the end.
    """
    tree = in_a_scene(
        rig(tmp_path, "motion: revolute", assy=FLOATING_OFF_A_SWING), [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0]
    )
    found = kinematics.bodies(tree)
    lid, arm = found.by_name("lid"), found.by_name("arm")
    assert lid.is_floating and lid.joint_parent is arm
    result = _run(export_world, tmp_path, tree, duration=0.25, samples=30, gravity=[0.0, 0.0, -G])
    assert "warnings" not in result

    readings = _readings(result)
    names = lid.joint.parameter_names()
    joint_map = kinematics.joint_map(lid.joint)
    hinge = kinematics.rotate(
        kinematics.compose(found.by_name("subject").world, arm.joint.frame(0)), arm.joint.freedoms[0].axis
    )
    exact = 0.0
    for reading in readings:
        state = reading["joints"]["loose"]
        # The lid's pose against the bar is where PartCAD's own joint puts it.
        values = [state["pos"][name] for name in names]
        expected = kinematics.compose(_body(reading, "arm"), lid.joint.location(values))
        assert _close_pose(_body(reading, "lid"), expected, 1e-6), reading["time"]
        if reading["time"] < 1.5 * STEP:
            # No step before the first to tell a speed by.
            continue
        swing = math.radians(reading["joints"]["swing"]["vel"])
        spin = math.sqrt(sum(v * v for v in state["vel"]["angular"]))
        assert spin == pytest.approx(abs(math.degrees(swing)), rel=1e-6, abs=1e-6), reading["time"]
        twists = (
            ((0.0, 0.0, -G * reading["time"]), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), tuple(swing * v for v in hinge)),
        )
        poses = [
            (tuple(b["quat"]), tuple(v / 1000.0 for v in b["pos"]))
            for b in (reading["bodies"]["lid"], reading["bodies"]["arm"])
        ]
        engine = kinematics.reading([joint_map], [tuple(poses)], [twists])["vel"]
        for name in names:
            assert state["vel"][name] == pytest.approx(engine[name], rel=1e-6, abs=1e-4), (reading["time"], name)
            exact = max(exact, abs(state["vel"][name] - engine[name]))

    worst = {name: 0.0 for name in names}
    import numpy

    for around in zip(readings, readings[1:], readings[2:], readings[3:], readings[4:]):
        times = numpy.array([reading["time"] for reading in around]) - around[2]["time"]
        for name in names:
            # The quartic through the five readings, its slope half a step back.
            f = [reading["joints"]["loose"]["pos"][name] for reading in around]
            difference = float(numpy.polyval(numpy.polyder(numpy.polyfit(times, f, 4)), -0.5 * STEP))
            rate = around[2]["joints"]["loose"]["vel"][name]
            assert rate == pytest.approx(difference, rel=3e-2, abs=30.0), (around[2]["time"], name, rate, difference)
            worst[name] = max(worst[name], abs(rate - difference))
    swung = abs(result["after"]["joints"]["swing"]["pos"])
    assert swung > 30.0
    fastest = {name: max(abs(r["joints"]["loose"]["vel"][name]) for r in readings) for name in names}
    print(
        "floating off a swing: the bar at %.1f deg; fastest rates %s; against the engine's within %.2e; "
        "against the readings' differences within %s"
        % (
            swung,
            {name: round(value, 2) for name, value in fastest.items()},
            exact,
            {name: round(value, 3) for name, value in worst.items()},
        )
    )


def test_in_gazebo_every_sample_asked_for_comes_back_one_a_step(gazebo, tmp_path, stand_in_geometry, export_world):
    """199 over 0.2 s is one a millisecond, Gazebo's step: far more than the sixty pose messages a second it publishes.

    So the run goes as slow as it must to give them ('pace'), and they come
    back one a step, each at its own instant.
    """
    tree = in_a_scene(rig(tmp_path, "motion: revolute"), [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0])
    result = _run(export_world, tmp_path, tree, duration=0.2, samples=199, gravity=[0.0, 0.0, -G])

    times = [sample["time"] for sample in result["samples"]]
    assert len(times) == 199
    # One a step, each at its own instant or a step past it: the first step
    # after the world is set running may go unpublished, Gazebo publishing
    # poses by the wall clock and having just published the paused world's.
    assert times == sorted(set(times))
    assert all(0.001 * (index + 1) - 1e-9 <= time_ <= 0.001 * (index + 2) + 1e-9 for index, time_ in enumerate(times))
    assert "warnings" not in result


def test_in_gazebo_a_timestep_is_the_step_gazebo_takes(gazebo, tmp_path, stand_in_geometry, export_world):
    """Half a millisecond: the readings fall on its multiples, and a revolute is still read where its body is."""
    tree = in_a_scene(rig(tmp_path, "motion: revolute"), [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0])
    arm = kinematics.bodies(tree).by_name("arm")
    result = _run(
        export_world, tmp_path, tree, bar(), duration=0.05, samples=99, timestep=0.0005, gravity=[0.0, 0.0, -G]
    )

    times = [sample["time"] for sample in result["samples"]]
    assert len(times) == 99
    # On the half milliseconds, each at its own instant or a step past it.
    assert all(abs(time_ / 0.0005 - round(time_ / 0.0005)) < 1e-6 for time_ in times)
    assert any(abs(time_ / 0.001 - round(time_ / 0.001)) > 0.25 for time_ in times)
    assert all(0.0005 * (index + 1) - 1e-9 <= time_ <= 0.0005 * (index + 2) + 1e-9 for index, time_ in enumerate(times))
    # Released at nought and at rest, its turn is the bar's under gravity:
    # half its angular acceleration times t (t + dt), semi-implicit Euler's,
    # and dt the half millisecond asked for.
    accelerating = math.degrees(BAR_MASS * G * BAR_D / BAR_I_PIVOT)
    for sample in result["samples"]:
        t = sample["time"]
        turned = abs(sample["joints"]["arm-base"]["pos"])
        assert turned == pytest.approx(0.5 * accelerating * t * (t + 0.0005), rel=2e-3), t
        expected = kinematics.compose(
            _body(sample, "subject"), arm.joint.location([sample["joints"]["arm-base"]["pos"]])
        )
        assert _close_pose(_body(sample, "arm"), expected, 1e-6), t
