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

Three things are checked, each against an answer that does not come from the
exporter:

* **every row of the classification table** (section 8.8 of PartCAD's
  simulation design record) is written as the SDFormat joints the table says,
  named as PartCAD names them, with no ``continuous`` anywhere;
* **every placement**: the world file is read back with a forward kinematics of
  its own (``sdf_poses``) -- the links' poses, each joint's frame and axis as
  SDFormat states them -- and every part is where the tree puts it with the
  joints at zero, and every body, moved, is where PartCAD's own 'kinematics'
  says that configuration puts it;
* **physics with a closed form**, in a real Gazebo: a bar on a revolute joint
  released horizontal passes the bottom at sqrt(2 m g d / I), and swings with
  the period 4 K(sin 45 deg) sqrt(I / (m g d)) of a pendulum released at a
  right angle; a nut on a screw moves as its thread says.

The last are skipped without a Gazebo, which CI's runners do not have; run them
in the image the simulation uses, with PartCAD in the environment:

    docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 \\
        --entrypoint bash \\
        ghcr.io/partcad/partcad-sim-gazebo:latest -c \\
        'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest numpy partcad \\
         && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_export_joints.py'
"""

import asyncio
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

ROWS = [
    # (what the bore declares, the kind, the SDFormat joints written, dummy links)
    ("motion: {type: revolute, limits: {lower: 0, upper: 90}}", kinematics.REVOLUTE, [("arm-base", "revolute")], 0),
    ("motion: continuous", kinematics.CONTINUOUS, [("arm-base", "revolute")], 0),
    ("motion: {type: prismatic, limits: {lower: 0, upper: 5}}", kinematics.PRISMATIC, [("arm-base", "prismatic")], 0),
    (
        "motion: cylindrical",
        kinematics.CYLINDRICAL,
        [("arm-base.turnZ", "revolute"), ("arm-base.moveZ", "prismatic")],
        1,
    ),
    ("threadStep: 10\nmotion: screw", kinematics.SCREW, [("arm-base", "screw")], 0),
    ("motion: universal", kinematics.UNIVERSAL, [("arm-base", "universal")], 0),
    ("motion: ball", kinematics.BALL, [("arm-base", "ball")], 0),
    (
        "motion: planar",
        kinematics.PLANAR,
        [("arm-base.moveY", "prismatic"), ("arm-base.moveX", "prismatic"), ("arm-base.turnZ", "revolute")],
        2,
    ),
    (
        "motion: floating",
        kinematics.FLOATING,
        [
            ("arm-base.moveY", "prismatic"),
            ("arm-base.moveX", "prismatic"),
            ("arm-base.moveZ", "prismatic"),
            ("arm-base.ball", "ball"),
        ],
        3,
    ),
    (
        "parameters: {turnZ: [-90, 90], moveX: [-5, 5]}\nmotion: {dof: [turnZ, moveX]}",
        kinematics.CHAIN,
        [("arm-base.turnZ", "revolute"), ("arm-base.moveX", "prismatic")],
        1,
    ),
]

# Starting values away from zero where a kind has a parameter to give one: what
# SDFormat cannot state, so every joint is written at zero and these reported.
_STARTS = {
    kinematics.REVOLUTE: ", toParams: {turnZ: 30}",
    kinematics.PRISMATIC: ", withParams: {moveZ: 3}",
    kinematics.CHAIN: ", toParams: {moveX: 2, turnZ: -20}",
}


def _row_id(row):
    return row[1]


@pytest.mark.parametrize("bore, kind, written, dummies", ROWS, ids=[_row_id(row) for row in ROWS])
def test_every_kind_of_joint_is_written_as_the_table_says(
    tmp_path, stand_in_geometry, export_world, bore, kind, written, dummies
):
    tree = rig(tmp_path, bore)
    arm = kinematics.bodies(tree).by_name("arm")
    assert arm.joint.kind == kind

    result, _path, sdf = export(export_world, tmp_path, tree)

    assert result["success"]
    (model,) = [model for model in sdf.find("world").findall("model") if model.get("name") != "ground_plane"]
    # The root of an assembly exported on its own is welded to the world (the
    # file is for looking at) -- by a joint, so that its joints still move.
    assert model.get("name") == "rig" and model.find("static") is None
    assert sorted(sdf_joints(sdf)) == sorted([("rig-world", "fixed")] + written)
    if dummies:
        # A chain is its freedoms in the order the joint applies them.
        expected = ["arm-base." + export_world.freedom_label(freedom) for freedom in arm.joint.freedoms]
        if kind == kinematics.FLOATING:
            expected = expected[:3] + ["arm-base.ball"]
        assert [name for name, _type in sdf_joints(sdf)[1:]] == expected
    assert all(joint.get("type") != "continuous" for joint in sdf.iter("joint"))
    links = [link.get("name") for link in model.findall("link")]
    assert links[0] == "rig_link" and links[-1] == "arm"
    assert len(links) == 2 + dummies
    for link in model.findall("link")[1:-1]:
        # A dummy link: a small mass and inertia, and nothing to see.
        assert float(link.findtext("inertial/mass")) == pytest.approx(export_world.DUMMY_MASS)
        assert link.find("visual") is None and link.find("collision") is None
    if dummies:
        assert any("has no SDFormat type" in warning for warning in result["warnings"])
    # Every axis is expressed in its joint's parent link.
    for joint in model.findall("joint"):
        for xyz in joint.findall("axis/xyz") + joint.findall("axis2/xyz"):
            assert xyz.get("expressed_in") == joint.findtext("parent")


def test_an_unlimited_turn_is_a_revolute_with_no_limit_and_a_limited_one_has_radians(
    tmp_path, stand_in_geometry, export_world
):
    _result, _path, free = export(export_world, tmp_path, rig(tmp_path / "a", "motion: revolute"), "a.world")
    _result, _path, bounded = export(
        export_world,
        tmp_path,
        rig(tmp_path / "b", "motion: {type: revolute, limits: {lower: -30, upper: 90}}"),
        "b.world",
    )

    assert free.find(".//joint[@name='arm-base']").get("type") == "revolute"
    assert free.find(".//joint[@name='arm-base']/axis/limit") is None
    limit = bounded.find(".//joint[@name='arm-base']/axis/limit")
    freedom = (
        kinematics.bodies(rig(tmp_path / "c", "motion: {type: revolute, limits: {lower: -30, upper: 90}}"))
        .by_name("arm")
        .joint.freedoms[0]
    )
    assert sorted([float(limit.findtext("lower")), float(limit.findtext("upper"))]) == pytest.approx(
        sorted([math.radians(freedom.lower), math.radians(freedom.upper)])
    )


def test_a_joint_s_physics_are_its_dynamics_and_limits_in_si(tmp_path, stand_in_geometry, export_world):
    tree = rig(
        tmp_path,
        "parameters: {moveZ: [0, 50]}\nmotion: prismatic\n"
        "physics: {damping: 4, friction: 0.5, springStiffness: 100, springReference: 20, maxEffort: 30, "
        "maxVelocity: 250, erp: 0.2}",
    )
    result, _path, sdf = export(export_world, tmp_path, tree)

    axis = sdf.find(".//joint[@name='arm-base']/axis")
    assert float(axis.findtext("dynamics/damping")) == pytest.approx(4.0)
    assert float(axis.findtext("dynamics/friction")) == pytest.approx(0.5)
    assert float(axis.findtext("dynamics/spring_stiffness")) == pytest.approx(100.0)
    # A position of the joint: millimetres to metres, along the same axis.
    freedom = kinematics.bodies(tree).by_name("arm").joint.freedoms[0]
    assert abs(float(axis.findtext("dynamics/spring_reference"))) == pytest.approx(0.020)
    assert float(axis.findtext("limit/effort")) == pytest.approx(30.0)
    assert float(axis.findtext("limit/velocity")) == pytest.approx(0.25)
    assert sorted(float(axis.findtext("limit/" + side)) for side in ("lower", "upper")) == pytest.approx(
        sorted([freedom.lower / 1000.0, freedom.upper / 1000.0])
    )
    assert "joint erp" in result["unsupported"]


def test_a_screw_states_its_thread_in_the_units_of_the_version_written(tmp_path, stand_in_geometry, export_world):
    """10 mm a turn: radians a metre, left-handed positive, in 1.9; metres a turn, right-handed, from 1.10."""
    tree = rig(tmp_path, "threadStep: 10\nmotion: screw")
    _result, _path, old = export(export_world, tmp_path, tree, "a.world")
    _result, _path, new = export(export_world, tmp_path, tree, "b.world", version="1.10")

    radians_a_metre = float(old.findtext(".//joint[@name='arm-base']/thread_pitch"))
    metres_a_turn = float(new.findtext(".//joint[@name='arm-base']/screw_thread_pitch"))
    assert abs(metres_a_turn) == pytest.approx(0.01)
    assert radians_a_metre == pytest.approx(-2 * math.pi / metres_a_turn)
    assert old.find(".//screw_thread_pitch") is None and new.find(".//thread_pitch") is None


MIMIC = """
links:
  - part: base
  - part: arm
    connect: {name: base, with: pin, to: bore, joint: drive}
  - part: lid
    connect:
      name: arm
      with: pin
      to: bore
      joint: follower
      motion: {type: revolute, mimic: {joint: drive, multiplier: -2, offset: 15}}
"""


def test_a_mimic_is_written_where_the_version_has_one_and_reported_where_it_has_not(
    tmp_path, stand_in_geometry, export_world
):
    tree = rig(tmp_path, "motion: revolute", assy=MIMIC)
    result, _path, old = export(export_world, tmp_path, tree, "a.world")
    _result, _path, new = export(export_world, tmp_path, tree, "b.world", version="1.11")

    assert old.find(".//mimic") is None
    assert any("'follower' mimics 'drive'" in warning and "1.11" in warning for warning in result["warnings"])
    mimic = new.find(".//joint[@name='follower']/axis/mimic")
    assert mimic.get("joint") == "drive" and mimic.get("axis") == "axis"
    assert float(mimic.findtext("multiplier")) == pytest.approx(-2.0)
    assert float(mimic.findtext("offset")) == pytest.approx(math.radians(15.0))


def test_a_starting_value_is_reported_and_the_joint_written_at_zero(tmp_path, stand_in_geometry, export_world):
    tree = rig(tmp_path, "parameters: {turnZ: [-90, 120]}\nmotion: revolute", params=", toParams: {turnZ: 30}")
    result, _path, sdf = export(export_world, tmp_path, tree)

    found = kinematics.bodies(tree)
    arm = found.by_name("arm")
    assert abs(arm.joint.freedoms[0].value) == pytest.approx(30.0)
    assert any("no starting position" in warning and "'arm-base' at" in warning for warning in result["warnings"])
    model = sdf.find("world/model[@name='rig']")
    at_zero = kinematics.compose(found.by_name("rig").world, arm.joint.location([0.0]))
    assert _close_pose(sdf_poses(model)["arm"], at_zero)


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


def _zero_world(found, body, zero_values):
    if not isinstance(body.parent, kinematics.Body):
        return body.world
    return kinematics.compose(
        _zero_world(found, body.parent, zero_values), body.joint.location(zero_values(body.joint))
    )


@pytest.mark.parametrize("bore, kind, written, dummies", ROWS, ids=[_row_id(row) for row in ROWS])
def test_every_part_is_where_partcad_puts_it_and_moves_as_partcad_says(
    tmp_path, stand_in_geometry, export_world, bore, kind, written, dummies
):
    """At zero every part is where 'kinematics' puts it -- the tree as drawn, where it starts at zero -- and,
    moved, every body is where the joint's own product puts it."""
    tree = in_a_scene(rig(tmp_path, bore, params=_STARTS.get(kind, "")), SCENE_LOCATION)
    found = kinematics.bodies(tree)
    _result, _path, sdf = export(export_world, tmp_path, tree, static=False)
    model = sdf.find("world/model[@name='subject']")
    links = {body.name: ("subject_link" if body.is_root else body.name) for body in found}

    at_zero = sdf_poses(model)
    flat = _flat_poses(tree)
    starts_at_zero = kind not in _STARTS
    for body in found:
        expected = _zero_world(found, body, export_world.zero_values)
        assert _close_pose(at_zero[links[body.name]], expected), body.name
        link = model.find("link[@name='%s']" % links[body.name])
        for index, part in enumerate(body.parts):
            visual = link.find("visual[@name='%s_visual_%d']" % (links[body.name], index))
            placed = urdf_common.compose(at_zero[links[body.name]], _pose(visual))
            assert _close_pose(placed, kinematics.compose(expected, part.placement)), (body.name, part.label)
            if starts_at_zero:
                assert _close_pose(placed, flat[id(part.node)]), (body.name, part.label)

    # Moved: every scalar freedom away from zero, a coupled one by its
    # coupling, a ball turned any which way.
    arm = found.by_name("arm")
    joint = arm.joint
    values = export_world.zero_values(joint)
    for freedom in joint.freedoms:
        if freedom.follows is None:
            values[freedom.index] += 17.0 if freedom.kind == kinematics.TURN else 3.0
    for freedom in joint.freedoms:
        if freedom.follows is not None:
            values[freedom.index] = freedom.ratio * values[freedom.follows] + freedom.offset
    classification = joint.classification
    turn = (0.3, -0.5, 0.8), 40.0
    if kind in (kinematics.BALL, kinematics.FLOATING):
        # The ball's quaternion is a turn in its own frame, the native joint's;
        # kinematics' three turns are the same turn as three in sequence.
        quaternion = urdf_common.axis_angle_to_quat(*turn)
        first = 3 if kind == kinematics.FLOATING else 0
        values[first:] = _three_turns(quaternion, classification.axes[first:])

    coordinates = {}
    if kind == kinematics.BALL:
        coordinates["arm-base"] = urdf_common.axis_angle_to_quat(*turn)
    elif kind == kinematics.UNIVERSAL:
        coordinates["arm-base"] = [math.radians(values[0]), math.radians(values[1])]
    else:
        single = kind in (kinematics.REVOLUTE, kinematics.CONTINUOUS, kinematics.PRISMATIC, kinematics.SCREW)
        for freedom in joint.freedoms:
            if kind == kinematics.SCREW and freedom.kind == kinematics.MOVE:
                continue
            if kind == kinematics.FLOATING and freedom.kind == kinematics.TURN:
                continue
            name = "arm-base" if single else "arm-base." + export_world.freedom_label(freedom)
            scale = math.pi / 180.0 if freedom.kind == kinematics.TURN else 0.001
            coordinates[name] = values[freedom.index] * scale
        if kind == kinematics.FLOATING:
            coordinates["arm-base.ball"] = urdf_common.axis_angle_to_quat(*turn)
    assert set(coordinates) == {name for name, _type in written}
    moved = sdf_poses(model, coordinates)
    expected = kinematics.compose(found.by_name("subject").world, joint.location(values))
    assert _close_pose(moved["arm"], expected, 1e-7), kind


def _three_turns(quaternion, axes):
    """The three angles (degrees) about 'axes', in order, that turn as 'quaternion' does -- by Newton's method."""
    angles = [0.0, 0.0, 0.0]

    def turned(values):
        pose = kinematics.IDENTITY
        for axis, value in zip(axes, values):
            moved = kinematics.motion(kinematics.TURN, axis, value)
            pose = pose if moved is None else urdf_common.compose(pose, moved)
        return pose[0]

    def error(values):
        q = turned(values)
        sign = 1.0 if sum(a * b for a, b in zip(q, quaternion)) >= 0 else -1.0
        return [sign * a - b for a, b in zip(q, quaternion)][1:]

    for _ in range(50):
        current = error(angles)
        if max(abs(v) for v in current) < 1e-14:
            break
        jacobian = []
        for i in range(3):
            step = list(angles)
            step[i] += 1e-6
            jacobian.append([(a - b) / 1e-6 for a, b in zip(error(step), current)])
        import numpy

        delta = numpy.linalg.solve(numpy.array(jacobian).T, -numpy.array(current))
        angles = [a + float(d) for a, d in zip(angles, delta)]
    return angles


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


@pytest.mark.parametrize("bore, kind, written, dummies", ROWS, ids=[_row_id(row) for row in ROWS])
def test_in_gazebo_every_kind_of_joint_is_a_world_sdformat_accepts(
    gazebo, tmp_path, stand_in_geometry, export_world, bore, kind, written, dummies
):
    (binary, _args, _topic), environment = gazebo
    tree = in_a_scene(rig(tmp_path, bore), SCENE_LOCATION)
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


def _complete_elliptic_k(k):
    """K(k), the complete elliptic integral of the first kind, by the arithmetic-geometric mean."""
    a, b = 1.0, math.sqrt(1.0 - k * k)
    while abs(a - b) > 1e-15:
        a, b = (a + b) / 2.0, math.sqrt(a * b)
    return math.pi / (2.0 * a)


@pytest.fixture(scope="module")
def swung(gazebo, tmp_path_factory):
    """The bar released horizontal on a revolute joint about a horizontal axis, run in Gazebo for 1.5 s.

    The hinge is the bore's Z axis, which the scene turns to lie along world Y,
    so that the bar, along the arm's X, starts horizontal at nought -- where
    every joint starts in a world this exporter writes.
    """
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

        directory = tmp_path_factory.mktemp("pendulum")
        tree = in_a_scene(rig(directory, "motion: revolute"), [[0.0, 0.0, 500.0], [1.0, 0.0, 0.0], 90.0])
        _result, path, _sdf = export(
            module,
            directory,
            tree,
            "pendulum.world",
            static=False,
            self_collide=True,
            ground_plane=False,
            properties=weighed(tree, {"arm": bar()}),
        )
    finally:
        patch.undo()
    return simulate_gazebo.process(str(directory), {"scene_file": str(path), "duration": 1.5, "samples": 3000})


def test_in_gazebo_a_pendulum_released_horizontal_passes_the_bottom_at_the_speed_its_fall_gives_it(swung):
    """omega = sqrt(2 m g d / I_pivot): all of m g d turned into (1/2) I omega^2."""
    expected = math.degrees(math.sqrt(2.0 * BAR_MASS * G * BAR_D / BAR_I_PIVOT))
    readings = [sample["joints"]["arm-base"] for sample in swung["samples"]]

    # Released at nought: the first reading is a few steps in, where it has
    # fallen as anything starting from rest does, by half its acceleration
    # times the time squared.
    before = swung["before"]
    accelerating = math.degrees(BAR_MASS * G * BAR_D / BAR_I_PIVOT)
    assert abs(before["joints"]["arm-base"]["pos"]) == pytest.approx(0.5 * accelerating * before["time"] ** 2, abs=0.1)
    assert swung["before"]["joints"]["arm-base"]["type"] == "continuous"
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


def test_in_gazebo_a_nut_on_a_screw_moves_as_its_thread_says(gazebo, tmp_path, stand_in_geometry, export_world):
    """Pulled down its axis by its weight, the nut moves 10 mm for every turn, the way PartCAD couples them."""
    import simulate_gazebo

    tree = in_a_scene(rig(tmp_path, "threadStep: 10\nmotion: screw"), [[0.0, 0.0, 800.0], [0.0, 0.0, 1.0], 0.0])
    _result, path, _sdf = export(export_world, tmp_path, tree, static=False, ground_plane=False)
    result = simulate_gazebo.process(str(tmp_path), {"scene_file": str(path), "duration": 0.5})

    arm = kinematics.bodies(tree).by_name("arm")
    move = next(freedom for freedom in arm.joint.freedoms if freedom.kind == kinematics.MOVE)
    turned = result["after"]["joints"]["arm-base"]["pos"]
    assert abs(turned) > 90.0
    # Where the joint's own product puts the nut at that turn, along the world's
    # Z -- which is where Gazebo has it.
    values = [0.0, 0.0]
    values[move.follows] = turned
    values[move.index] = move.ratio * turned + move.offset
    expected = kinematics.compose(kinematics.bodies(tree).by_name("subject").world, arm.joint.location(values))
    fallen = result["after"]["bodies"]["arm"]["pos"]
    print(
        "screw: turned %.2f deg, the nut at %s mm in its model, PartCAD puts it at %s" % (turned, fallen, expected[1])
    )
    assert fallen[2] == pytest.approx(expected[1][2] - 800.0, abs=0.2)
