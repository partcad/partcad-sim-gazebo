#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""The Gazebo world exporter (declared under 'export:' in partcad.yaml beside this).

Writes a PartCAD scene -- or an assembly, or any other shape -- as an SDFormat
``.world`` file plus the mesh files it references. This is the format PartCAD
calls ``world``, after the files it lives in. "SDF" is overloaded and the two
meanings are unrelated: the ``sdf`` *part* type is a signed distance function,
while this is **SDFormat**, the XML dialect Gazebo describes a simulation world
in.

Like PartCAD's URDF exporter, this one writes the assembly *tree* rather than
the geometry it decodes to: PartCAD hands every exporter that tree as
``request["shape_tree"]``, beside the geometry in ``request["wrapped"]`` (see
'wrappers/wrapper_export.py' in PartCAD itself). The geometry keeps the tree's
shape and nothing else about it: every node's ``name`` and ``label`` is dropped
and its placement is baked in rather than staying readable as data. This
exporter needs all three, because that is what the models, the links, the poses
and the properties are built from.

A PartCAD released before 'shape_tree' hands the tree in
'wrapped' instead, and only to an exporter that declares ``decode: false`` -
which is why this one still does, and why both are read: see 'tree_of()'.

Which nodes of the tree are bodies, and how the bodies hang together, is not
decided here. It is PartCAD's body rule, in PartCAD's own 'kinematics' (see
'wrappers/kinematics.py' in PartCAD, and sections 7 and 8.8 of its simulation
design record): every element of a scene is a body of its own, everything
inside an assembly is attached -- welded into one body, or jointed to it by a
connection that keeps a degree of freedom -- and a scene element marked
``fixed: true`` is welded to the world. The MJCF and URDF exporters write the
same rule, so the three cannot come to disagree about what falls apart. This
exporter writes what it returns:

  * the exported object is the ``<world>``,
  * every body nothing in the tree holds -- an element of a scene, the root of
    an assembly -- is a ``<model>`` at that body's pose, and its link is named
    '<model>_link'; every body that hangs off it, at any depth, is another
    ``<link>`` of the same model, under the body's own name,
  * every part welded into a body is a ``<visual>`` and a ``<collision>`` of
    its link, at its place in the body, both referencing a mesh written next to
    the world file; the link's ``<inertial>`` is the body's, its parts combined
    by PartCAD,
  * every joint is SDFormat's own, by what 'kinematics.classify' says it is
    (see 'Planner.jointed'): a ``revolute`` -- never a ``continuous``, which
    Gazebo's DART builds as a fixed joint, so an unlimited turn is a revolute
    with no ``<limit>`` --, a ``prismatic``, a ``screw`` with its thread pitch,
    a ``universal`` and a ``ball``; and anything SDFormat has no type for -- a
    cylindrical or a planar joint, a floating one inside a tree, any other
    combination -- a chain of those through dummy links (see 'DUMMY_MASS'),
    which is reported.

**Names.** One name is never two things in one world: no link is named like a
model, and no joint like a link. SDFormat requires the second within a model,
and Gazebo needs the first: it reports every pose by the entity's bare name, and
a model whose link shared its name was reported where its link sits in it -- at
the origin, for ever. A joint written as one SDFormat joint keeps PartCAD's
name, which is what a validation reads; one written as several is
'<name>.<parameter>' for each, which is the name the MJCF exporter gives the
MuJoCo joint of the same freedom.

**Frames.** A link's frame is its body's own, so a part sits in its link where
the body holds it and the inertial is the body's as PartCAD combined it. Every
link is written where its joint puts it with every freedom at *zero* (see
'STARTS_NOT_WRITTEN'), in its model's frame; a joint's ``<pose>`` is the frame
it turns or slides in, relative to its child link as SDFormat reads it by
default; and its ``<axis><xyz>`` is expressed in its *parent* link, the frame
'kinematics.Joint.axis_in_parent' states it in. So Gazebo's zero of a joint is
PartCAD's zero, its limits are the ones declared, and the joint states a run
reports are PartCAD's values.

Two things this exporter decides that the file does not say, because a static
arrangement does not say them and a *simulation* needs both:

  * **Whether what is free may move.** A body nothing holds is free, and in
    SDFormat that is a model with no joint to the world. ``static: true`` (the
    default, for a file somebody opens to look at) welds every such body to the
    world instead: a model with nothing moving in it is ``<static>``, as it
    always was, and the root of a mechanism gets a fixed joint to ``world``, so
    that the joints *inside* an assembly still move -- a pendulum exported on
    its own still swings on its hinge. A body ``fixed`` to the world gets that
    fixed joint whatever ``static`` says. ``static: false`` is what the
    simulation asks for.
  * **What it stands on.** ``sun`` and ``ground_plane`` are what makes the
    file usable rather than what the scene says.

``flatten`` is no longer an option, and is reported when it is passed: which
nodes are bodies is the body rule's to say.

Two conventions are worth stating, because they are what makes the round trip
close, and both are the ones the URDF exporter uses:

  * Meshes are written in millimetres -- the unit PartCAD uses everywhere --
    and referenced with ``<scale>0.001 0.001 0.001</scale>``, which is how
    SDFormat says "these coordinates are millimetres". Poses are in metres and
    radians, and so is everything a joint states, as SDFormat requires.
  * Geometry is written in its own frame and placed by the element that holds
    it. A shape that appears more than once is written once and referenced by
    every link that uses it.

What the part states about itself wins over anything computed here: mass,
centre of mass, inertia, friction, contact and colour are named PartCAD
properties, and this writes each of them into the SDFormat element that states
it. A part that states none of them is no different: PartCAD works out its
mass, centre of mass and inertia from its solid and the density of what it is
made of (see 'partcad.physics' in PartCAD), and 'kinematics' adds the parts of
a body up with PartCAD's own 'mass_properties', the one copy of that
arithmetic the URDF and MJCF exporters use too. A property PartCAD has and
SDFormat has no spelling for is reported rather than dropped in silence -- see
SDF_STATED and JOINT_STATED.

What the scene says about its *world* arrives in ``request["world"]`` when it
says anything (see 'wrappers/wrapper_export.py' in PartCAD itself). Its gravity
is the world's ``<gravity>``. The fluid it is filled with is not written, and
the export says so -- see 'MEDIUM_NOT_WRITTEN' for why Gazebo's buoyancy system
is not the straightforward answer it looks like.
"""

import math
import os
import re
import sys
from xml.etree import ElementTree

# Pinned before anything that may pull OCP - see the note in ocp_serialize.
import pyexpat  # noqa: F401

# 'gazebo_common' is this package's, and sits beside this file. The other three
# are PartCAD's own and are the sandbox contract every implementation is written
# against: 'ocp_serialize' is the shape and assembly envelope format this is
# handed, 'urdf_common' is the pose arithmetic they are stated in, and
# 'kinematics' is which nodes are bodies, what each weighs, and how the joints
# between them move. A sandbox runs a wrapper out of PartCAD's 'wrappers/'
# directory, so all three are already on sys.path by the time this is imported.
sys.path.append(os.path.dirname(__file__))
import gazebo_common  # noqa: E402
import kinematics  # noqa: E402
import ocp_serialize  # noqa: E402
import urdf_common  # noqa: E402

# Metres per millimetre, for the mesh '<scale>': the meshes are written in
# millimetres and SDFormat reads mesh coordinates as metres after scaling.
MESH_SCALE = 1.0 / urdf_common.MM_PER_M

# Why the fluid a scene is filled with is not in the world this writes, which is
# reported as a warning on every export of a scene that has one.
#
# SDFormat has no world-level fluid -- '<atmosphere>' is a temperature and a
# pressure for sensors, not a density anything is buoyed or dragged by -- and
# Gazebo's answer is a system, 'gz-sim-buoyancy-system' with a
# '<uniform_fluid_density>'. It is not written, for two reasons that are each
# enough on their own:
#
#   * On Gazebo Harmonic -- what the image this package names carries -- the
#     system measures a mesh's volume without its '<scale>'. Every mesh written
#     here is in millimetres under a scale of 0.001, so each link would be
#     buoyed by a volume 10^9 times too large and leave the world. Fixed
#     upstream in gazebosim/gz-sim#3909 (August 2026), in no release this package
#     can yet assume.
#   * A world that names any system of its own gets none of Gazebo's defaults:
#     physics, the scene broadcaster whose poses the simulation reads, user
#     commands. Writing one means writing all of those too, and keeping that
#     list in step with Gazebo's own.
#
# Viscous drag has no world-level system at all; 'gz-sim-hydrodynamics-system'
# is per model, with coefficients a scene does not state. So a Gazebo run of a
# scene filled with water is a run in a vacuum, and says so -- the MuJoCo plugin
# is the one that models a fluid today.
MEDIUM_NOT_WRITTEN = (
    "the scene is filled with %s, which this world does not model: SDFormat has no fluid of its own and "
    "Gazebo's buoyancy system cannot be written for millimetre meshes on Gazebo Harmonic, so Gazebo runs it in "
    "a vacuum"
)

# The one name SDFormat reserves that 'sanitize_name' can produce (the others
# begin and end with '__', which it strips): what a joint says it is attached to
# when it is attached to the world. Nothing in a world may be called it, the
# world included.
RESERVED_NAME = "world"

# SDFormat names end up as XML attributes and as scoped names joined by '::',
# so anything outside this set is replaced.
_UNSAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")

# PartCAD property -> where under '<link>' it is stated, and how to write it.
# The path is relative to the link; the writer creates whatever is missing.
LINK_PHYSICS = {
    "gravity": ("gravity", lambda v: _bool_text(v)),
    "selfCollide": ("self_collide", lambda v: _bool_text(v)),
    "velocityDamping": ("velocity_decay/linear", repr),
}

# PartCAD property -> where under '<collision><surface>' it is stated. The
# mirror image of wrapper_import_world.SURFACE_PHYSICS, and to be kept that way.
SURFACE_PHYSICS = {
    "friction": ("friction/ode/mu", repr),
    "friction2": ("friction/ode/mu2", repr),
    "frictionDirection": ("friction/ode/fdir1", lambda v: " ".join(repr(float(x)) for x in v)),
    "contactStiffness": ("contact/ode/kp", repr),
    "contactDamping": ("contact/ode/kd", repr),
    "minContactDepth": ("contact/ode/min_depth", lambda v: repr(v / urdf_common.MM_PER_M)),
    "maxContactVelocity": ("contact/ode/max_vel", lambda v: repr(v / urdf_common.MM_PER_M)),
    "maxContacts": ("contact/ode/max_contacts", lambda v: str(int(v))),
    "restitution": ("bounce/restitution_coefficient", repr),
}

# Every part property this exporter has an SDFormat spelling for. A 'physics'
# property outside this set is one PartCAD supports and SDFormat does not: it
# is reported through the response and logged at info level, which is the
# mirror image of the reader reporting what it cannot keep. When PartCAD grows
# a physical property, either give it a spelling above or let it be reported
# here - do not let it disappear quietly.
#
# 'density' is here although this writes no '<density>' -- SDFormat grew one
# only in 1.11, for an '<inertial auto="true">' that Gazebo computes itself --
# because it is not lost: it is what the mass, the centre of mass and the
# inertia of a part that states no mass were worked out from, so the file states
# it as the '<inertial>' it comes to.
#
# 'volume' and 'centerOfVolume' are not properties of the part at all, but what
# PartCAD measured its solid to enclose and where, handed to every exporter for
# the one that buoys a body in a fluid at its centre of buoyancy. This one
# writes no fluid (see MEDIUM_NOT_WRITTEN), so nothing is lost by not writing
# them either.
SDF_STATED = (
    frozenset(("mass", "centerOfMass", "inertiaOrientation", "inertia", "density", "volume", "centerOfVolume"))
    | frozenset(LINK_PHYSICS)
    | frozenset(SURFACE_PHYSICS)
)


#
# Joints
#

TURN = kinematics.TURN
MOVE = kinematics.MOVE
IDENTITY = kinematics.IDENTITY

# What SDFormat 1.9 means by "no bound": the default of an open '<lower>' and
# '<upper>'. A joint bounded on one side only is written with this on the
# other, which is also the line the simulation beside this draws between a
# 'revolute' and a 'continuous' it reports (see 'simulate_gazebo.NO_LIMIT').
NO_LIMIT = 1.0e16

# A dummy link -- one between two freedoms of a joint SDFormat has no single
# type for -- weighs this much (kg) and turns this hard (kg.m^2 about each
# axis): enough for Gazebo's physics to accept it, too little to matter beside
# a part. The URDF exporter's numbers, for the same problem.
DUMMY_MASS = 1.0e-3
DUMMY_INERTIA = 1.0e-6

# What an SDFormat joint written for one freedom of a joint of several is called
# after the PartCAD joint's name: '<name>.<parameter>', as the MJCF exporter
# names the MuJoCo joint of the same freedom, so that one validation reads both.
FREEDOM_SEPARATOR = "."

# PartCAD joint physics -> what '<axis><dynamics>' calls it. Per unit of the
# joint's own motion in SI already, as PartCAD states them (N.m.s/rad or N.s/m,
# N.m or N, N.m/rad or N/m): written as they are.
JOINT_DYNAMICS = {"damping": "damping", "friction": "friction", "springStiffness": "spring_stiffness"}
# The one that is a position of the joint, in PartCAD's degrees or millimetres.
SPRING_REFERENCE = "springReference"
# '<limit><effort>' (N.m or N), and '<limit><velocity>' (a speed in degrees or
# millimetres per second in PartCAD).
JOINT_LIMITS = {"maxEffort": "effort", "maxVelocity": "velocity"}
# Every joint property this exporter has a spelling for; anything else a joint
# states (the ODE settings a URDF's '<gazebo>' block carried, say) is reported.
JOINT_STATED = frozenset(JOINT_DYNAMICS) | frozenset(JOINT_LIMITS) | {SPRING_REFERENCE}

# The first SDFormat versions with each of these. A screw's pitch is
# '<thread_pitch>' before 1.10 -- radians per metre, positive for a left-handed
# thread, as Gazebo Classic read it and Harmonic still does -- and
# '<screw_thread_pitch>' from 1.10, metres per turn, positive for a right-handed
# one. '<mimic>' is in an axis from 1.11 and nowhere before; Gazebo Harmonic
# reads it and does not enforce it (its DART has no mimic constraint).
SCREW_THREAD_PITCH_SINCE = (1, 10)
MIMIC_SINCE = (1, 11)

# What is said about the starting value of a joint, which this does not write.
# SDFormat 1.9 has no starting position: '<axis><initial_position>' was in 1.6
# to 1.8 and went with 1.9, and Gazebo never read it. Nor does Harmonic's
# gz-sim reset a joint from anything a world states: the 'SetModelState'
# system that does is Ionic's. So every joint is written at zero -- the links
# where their joints put them at zero, which is what makes a reading of a joint
# PartCAD's value and its limits the declared ones -- and each joint that starts
# elsewhere in the assembly is named. The URDF exporter says the same, for the
# same reason.
STARTS_NOT_WRITTEN = (
    "SDFormat %s states no starting position for a joint, so every joint is written at zero; these start "
    "elsewhere in the assembly: %s"
)

FLATTEN_GONE = (
    "'flatten' is no longer an option of this export, and is ignored: which nodes are bodies is PartCAD's "
    "body rule - every element of a scene a body of its own, everything in an assembly attached"
)


def _bool_text(value):
    return "true" if value else "false"


def sanitize_name(name, fallback):
    """An SDFormat-safe name derived from a PartCAD name.

    PartCAD names carry package paths and ':' separators ("//pub/examples:logo")
    and the '/' that groups a link's own shapes, none of which belong in an
    SDFormat name -- '::' is its scope separator and a bare '/' has no meaning.
    """
    name = _UNSAFE_NAME.sub("_", str(name or "")).strip("_")
    return name or fallback


class NameAllocator:
    """Hands out unique names, since SDFormat requires them within a scope.

    Unique across the whole document rather than per scope: that satisfies the
    requirement everywhere at once, and it keeps the names readable in a
    simulator's entity tree and in what a run reports, where Gazebo states
    every entity by its bare name. The models, links and joints of a world
    share one, so that no two of them answer to one name.

    A name already taken gets '<separator><n>', counting from 'start': the
    world's own names are 'cube-2' after 'cube', as 'kinematics' names a
    second body and the core a second joint of one name, and the meshes are
    'cube_1.stl' after 'cube.stl', as they always were.
    """

    def __init__(self, separator="_", start=1, reserved=()):
        self.used = set(reserved)
        self.separator = separator
        self.start = start

    def take(self, name, fallback="model"):
        base = sanitize_name(name, fallback)
        candidate = base
        suffix = self.start
        while candidate in self.used:
            candidate = "%s%s%d" % (base, self.separator, suffix)
            suffix += 1
        self.used.add(candidate)
        return candidate


def sub(parent, path, text=None):
    """The element at 'path' under 'parent', creating what is missing."""
    element = parent
    for step in path.split("/"):
        found = element.find(step)
        if found is None:
            found = ElementTree.SubElement(element, step)
        element = found
    if text is not None:
        element.text = text
    return element


# Where PartCAD puts a node's own geometry, decoded, on the tree it hands over
# as 'shape_tree'. Spelled out rather than read off 'ocp_serialize', which only
# a PartCAD that hands 'shape_tree' defines it in.
KEY_SHAPE = "shape"


def tree_of(request):
    """The tree to write, as PartCAD handed it over.

    'shape_tree' from a PartCAD that hands every exporter the tree beside the
    geometry; 'wrapped' from one released before it did, which hands the tree
    there instead to an exporter that declares 'decode: false'.
    """
    tree = request.get("shape_tree")
    return tree if tree is not None else request.get("wrapped")


def node_geometry(node):
    """The node's own shape as a live TopoDS_Shape, in its own frame, or None.

    Decoded already on a node of 'shape_tree'; decoded here on one of the raw
    tree an older PartCAD hands over. The placement carried by the node is
    deliberately left off either way: it becomes the ``<pose>`` of the element
    above the geometry, not part of the geometry.
    """
    if not ocp_serialize.is_shape_object(node):
        return None
    if node.get(KEY_SHAPE) is not None:
        return node[KEY_SHAPE]
    without_placement = {key: value for key, value in node.items() if key != ocp_serialize.KEY_LOCATION}
    return ocp_serialize.decode_shape(without_placement)


def write_mesh(shape, path, options):
    """Triangulate 'shape' and write it out as an STL file."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer

    BRepMesh_IncrementalMesh(
        shape,
        theLinDeflection=options["tolerance"],
        isRelative=True,
        theAngDeflection=options["angularTolerance"],
        isInParallel=True,
    )
    writer = StlAPI_Writer()
    writer.ASCIIMode = options["ascii"]
    if not writer.Write(shape, path) or not os.path.exists(path) or os.path.getsize(path) == 0:
        raise Exception("Failed to write the mesh file: %s" % path)


def mesh_reference(shape, node, link_name, state):
    """Write 'shape' out as a mesh (or reuse one) and return the SDFormat URI.

    An identical shape used twice - a repeated fastener, a row of pallets -
    shares one mesh file, the way a world written by hand would.
    """
    key = node.get(ocp_serialize.KEY_BREP)
    mesh_file = state["meshes"].get(key)
    if mesh_file is None:
        mesh_name = state["mesh_names"].take(node.get("label") or link_name, "mesh")
        mesh_file = "%s/%s.stl" % (state["mesh_dir_name"], mesh_name)
        write_mesh(shape, os.path.join(state["mesh_dir"], "%s.stl" % mesh_name), state["options"])
        state["meshes"][key] = mesh_file
    return state["mesh_prefix"] + mesh_file


def carried_inertial(physics):
    """The ``<inertial>`` values a physics dict states, or None if it has no mass.

    Stated by the part or worked out by PartCAD, which this cannot tell apart
    and has no reason to: it restates them in SDFormat's terms and decides
    nothing.
    """
    if not physics or "mass" not in physics:
        return None
    result = {"mass": float(physics["mass"])}
    if "centerOfMass" in physics:
        result["centerOfMass"] = [float(v) for v in physics["centerOfMass"]]
    if "inertiaOrientation" in physics:
        result["inertiaOrientation"] = [float(v) for v in physics["inertiaOrientation"]]
    if "inertia" in physics:
        result["inertia"] = {key: float(value) for key, value in physics["inertia"].items()}
    return result


def write_inertial(link, values):
    """Write one link's ``<inertial>`` block from the values above."""
    if values is None:
        return
    inertial = sub(link, "inertial")
    sub(inertial, "mass", repr(values["mass"]))
    centre = values.get("centerOfMass")
    orientation = values.get("inertiaOrientation")
    if centre or orientation:
        rotation = urdf_common.rpy_to_quat([math.radians(v) for v in (orientation or (0.0, 0.0, 0.0))])
        pose = (rotation, tuple(centre or (0.0, 0.0, 0.0)))
        sub(inertial, "pose", gazebo_common.format_pose(pose))
    inertia = values.get("inertia")
    if inertia:
        block = sub(inertial, "inertia")
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            sub(block, key, repr(float(inertia.get(key, 0.0))))


def color_rgba(text):
    """A PartCAD colour string as SDFormat's ``r g b a``, or None."""
    value = str(text or "").strip().lstrip("#")
    if len(value) not in (6, 8):
        return None
    try:
        channels = [int(value[index : index + 2], 16) / 255.0 for index in range(0, len(value), 2)]
    except ValueError:
        return None
    while len(channels) < 4:
        channels.append(1.0)
    return " ".join("%.4g" % channel for channel in channels)


def write_material(visual, properties):
    """Write a ``<material>`` from what the shape says about its appearance."""
    rgba = color_rgba(properties.get("color"))
    if rgba is None:
        return
    material = sub(visual, "material")
    sub(material, "ambient", rgba)
    sub(material, "diffuse", rgba)


def physics_of(node, state):
    """What PartCAD says about the physics of the part behind one node of the tree."""
    return (state["properties"].get(node.get("name")) or {}).get("physics") or {}


def write_surface(collision, physics):
    """Write the ``<surface>`` of one collision from the link's properties."""
    for name, (path, render) in SURFACE_PHYSICS.items():
        if name in physics:
            sub(collision, "surface/" + path, render(physics[name]))


def add_sun(world):
    """The directional light every Gazebo world needs to show anything."""
    light = ElementTree.SubElement(world, "light")
    light.set("name", "sun")
    light.set("type", "directional")
    sub(light, "cast_shadows", "true")
    sub(light, "pose", "0 0 10 0 0 0")
    sub(light, "diffuse", "0.8 0.8 0.8 1")
    sub(light, "specular", "0.2 0.2 0.2 1")
    sub(light, "direction", "-0.5 0.1 -0.9")


def add_ground_plane(world):
    """The ground every Gazebo world is normally loaded with."""
    model = ElementTree.SubElement(world, "model")
    model.set("name", "ground_plane")
    sub(model, "static", "true")
    link = ElementTree.SubElement(model, "link")
    link.set("name", "link")
    for kind in ("collision", "visual"):
        element = ElementTree.SubElement(link, kind)
        element.set("name", kind)
        plane = sub(element, "geometry/plane")
        sub(plane, "normal", "0 0 1")
        sub(plane, "size", "100 100")
    material = sub(link.find("visual"), "material")
    sub(material, "ambient", "0.8 0.8 0.8 1")
    sub(material, "diffuse", "0.8 0.8 0.8 1")


def gravity_of(request, world):
    """The ``<gravity>`` to write, in m/s^2, or None to leave Gazebo's own.

    An explicit 'gravity' on this export wins -- it is a decision about this
    file -- and then the scene's. Neither is defaulted in this package's
    declaration, and must not be: a default there would be explicit in every
    request and beat every scene that ever stated a gravity.
    """
    if request.get("gravity") is not None:
        return gazebo_common.gravity_vector(request["gravity"], "the export's 'gravity'")
    if world.get("gravity") is not None:
        return gazebo_common.gravity_vector(world["gravity"], "the scene's 'gravity'")
    return None


#
# The plan: models, links and joints as plain data
#


def per_si(kind) -> float:
    """PartCAD's units of a freedom per SDFormat's: degrees per radian, millimetres per metre."""
    return 180.0 / math.pi if kind == TURN else urdf_common.MM_PER_M


def to_si(kind, value):
    """A position, bound or speed of a freedom in SDFormat's radians or metres, from PartCAD's units."""
    return None if value is None else float(value) / per_si(kind)


def _number(value):
    """'value' as a finite number, or None."""
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _shown(value, kind) -> str:
    return "%g %s" % (value, "deg" if kind == TURN else "mm")


def version_of(text):
    """An SDFormat version as numbers, '1.9' -> (1, 9): what decides which elements a joint is written with."""
    try:
        return tuple(int(part) for part in str(text).strip().split("."))
    except ValueError:
        return tuple(int(part) for part in gazebo_common.SDF_VERSION.split("."))


def freedom_label(freedom) -> str:
    """What an SDFormat joint of one freedom is called after its PartCAD joint's name: its parameter."""
    if freedom.parameter:
        return str(freedom.parameter)
    return "%s%d" % (freedom.kind, freedom.index)


def zero_values(joint):
    """Where every freedom of 'joint' is with every SDFormat coordinate of it at zero.

    Nought, but for a screw's coupled freedom, which a screw joint has no
    coordinate of its own for: it is where its leader at nought puts it.
    """
    values = [0.0] * len(joint.freedoms)
    if joint.kind == kinematics.SCREW:
        for freedom in joint.freedoms:
            if freedom.follows is not None:
                values[freedom.index] = (freedom.ratio or 0.0) * values[freedom.follows] + (freedom.offset or 0.0)
    return values


class LinkPlan:
    """One SDFormat link: a body, or a dummy link between two freedoms of one joint.

    'frame' is where the link is in its model's frame -- which is its model's
    root body's frame -- with every joint at zero.
    """

    def __init__(self, name, frame, body=None):
        self.name = name
        self.frame = frame
        self.body = body

    @property
    def dummy(self) -> bool:
        return self.body is None

    def __repr__(self):
        return "<LinkPlan %s%s>" % (self.name, " (dummy)" if self.dummy else "")


class AxisPlan:
    """One '<axis>' or '<axis2>' of a joint: a unit vector in the joint's frame, and the freedom it moves."""

    def __init__(self, vector, freedom, physics=True):
        self.vector = tuple(float(v) for v in vector)
        self.freedom = freedom
        # Whether the joint's physics are written on it: not on a freedom that
        # another one moves, which would charge the same motion twice.
        self.physics = physics
        # (leader joint, leader axis, multiplier, offset) in SI: a '<mimic>'.
        self.mimic = None


class JointPlan:
    """One SDFormat joint, in the model's frame and PartCAD's units; 'write_joint' converts."""

    def __init__(self, name, joint_type, parent, child, frame, source=None):
        self.name = name
        self.type = joint_type
        # A LinkPlan, or None for the world.
        self.parent = parent
        self.child = child
        # The frame the joint turns or slides in, in the model's frame.
        self.frame = frame
        self.axes = []
        # A screw's: the metres a right-handed turn about its '<axis>' moves
        # the child along it.
        self.pitch = None
        # The 'kinematics.Joint' it is written for, whose physics it states.
        self.source = source

    def __repr__(self):
        return "<JointPlan %s %s %s -> %s>" % (
            self.name,
            self.type,
            self.parent.name if self.parent else "world",
            self.child.name,
        )


class ModelPlan:
    """One SDFormat model: a body nothing in the tree holds, and every body that hangs off it."""

    def __init__(self, name, pose, body):
        self.name = name
        # The root body's pose in the world, and so the model's frame.
        self.pose = pose
        self.body = body
        self.links = []
        self.joints = []
        self.static = False


class Planner:
    """The models, links and joints of a world for the bodies 'kinematics.bodies()' found.

    Plain data, and nothing written: 'write_model' turns it into SDFormat, and
    the tests check it against the joints it came from, frame by frame.
    """

    def __init__(self, found, names=None, version=None, static=True, warnings=None):
        self.found = found
        self.names = names if names is not None else NameAllocator("-", 2)
        self.version_text = str(version or gazebo_common.SDF_VERSION)
        self.version = version_of(self.version_text)
        self.static = static
        self.warnings = warnings if warnings is not None else []
        self.unsupported = set()
        self.models = []
        # Body -> (its ModelPlan, its LinkPlan).
        self._placed = {}
        # PartCAD joint name -> the (JointPlan, freedom) pairs it became: what a
        # 'mimic' looks its leader up in.
        self._written = {}
        self._mimics = []
        self._started = []

    def plan(self):
        for body in self.found:
            if not body.parts and not body.children:
                # A frame with nothing in it and nothing under it, which is
                # noise in the world and in what a run reports.
                if body.joint is not None:
                    self.warnings.append(
                        "'%s' holds no geometry, so its joint '%s' is not written" % (body.name, body.joint.name)
                    )
                continue
            if body.is_root:
                self.root(body)
            elif id(body.parent) in self._placed:
                self.jointed(body)
        for model in self.models:
            self.weld(model)
        self._resolve_mimics()
        if self._started:
            self.warnings.append(STARTS_NOT_WRITTEN % (self.version_text, ", ".join(self._started)))
        return self

    def root(self, body):
        model = ModelPlan(self.names.take(body.name, "model"), body.world, body)
        link = LinkPlan(self.names.take("%s_link" % body.name, "link"), IDENTITY, body)
        model.links.append(link)
        self.models.append(model)
        self._placed[id(body)] = (model, link)

    def weld(self, model):
        """Weld the model's root to the world, if it is fixed there or 'static' says so."""
        fixed = model.body.parent == kinematics.WORLD
        if not fixed and not self.static:
            return
        if not fixed and not model.joints:
            # Nothing in it moves: a model of the world, as a scene's models
            # always were.
            model.static = True
            return
        # A '<static>' model moves nothing, its joints included: a mechanism
        # bolted down is its root welded to the world, and the rest free.
        root = model.links[0]
        name = self.names.take("%s-world" % model.body.name, "joint")
        model.joints.insert(0, JointPlan(name, "fixed", None, root, root.frame))

    def jointed(self, body):
        """The joint (or chain of joints) of 'body' to its parent, and its link."""
        model, parent = self._placed[id(body.parent)]
        joint = body.joint
        name = joint.name or "%s-%s" % (body.name, body.parent.name)
        kind = joint.kind
        classification = joint.classification
        zero = zero_values(joint)
        base = parent.frame
        child = LinkPlan(self.names.take(body.name, "link"), kinematics.compose(base, joint.location(zero)), body)
        self._placed[id(body)] = (model, child)

        for freedom in joint.freedoms:
            if freedom.follows is None and abs(freedom.value - zero[freedom.index]) > 1e-9:
                self._started.append("'%s' at %s" % (name, _shown(freedom.value, freedom.kind)))
        self.unsupported.update("joint %s" % key for key in set(joint.physics) - JOINT_STATED)
        if joint.soft_limits:
            self.unsupported.add("joint softLimits")

        if kind in (kinematics.REVOLUTE, kinematics.CONTINUOUS, kinematics.PRISMATIC):
            (freedom,) = joint.freedoms
            plan = self.single(model, name, joint, freedom, freedom.axis, parent, child, base, zero)
            model.links.append(child)
            self._written[joint.name] = [(plan, freedom)]
            self._mimic(plan, joint)
            return

        native = kinematics.compose(base, classification.origin) if classification.origin else base
        if kind == kinematics.SCREW:
            model.links.append(child)
            plan = self.screw(model, name, joint, parent, child, native)
            self._mimic(plan, joint)
            return
        if kind == kinematics.UNIVERSAL:
            model.links.append(child)
            plan = JointPlan(self.names.take(name, "joint"), "universal", parent, child, native, joint)
            plan.axes = [AxisPlan(classification.axes[i], joint.freedoms[i]) for i in range(2)]
            model.joints.append(plan)
            self._written[joint.name] = [(plan, freedom) for freedom in joint.freedoms]
            self._not_one_axis(joint, "universal")
            return
        if kind == kinematics.BALL:
            model.links.append(child)
            plan = JointPlan(self.names.take(name, "joint"), "ball", parent, child, native, joint)
            model.joints.append(plan)
            self._written[joint.name] = [(plan, None)]
            self._ball(name, joint, joint.freedoms)
            self._not_one_axis(joint, "ball")
            return
        if kind == kinematics.FLOATING:
            self.floating(model, name, joint, parent, child, native)
            return
        self.chain(model, name, joint, parent, child, base, zero)

    def single(self, model, name, joint, freedom, vector, parent, child, frame, zero=None):
        """One revolute or prismatic joint for one freedom; 'frame' is the freedom's own, or where the steps start."""
        if zero is not None:
            frame = kinematics.compose(frame, joint.frame(freedom.index, zero))
        joint_type = "revolute" if freedom.kind == TURN else "prismatic"
        plan = JointPlan(self.names.take(name, "joint"), joint_type, parent, child, frame, joint)
        plan.axes.append(AxisPlan(vector, freedom, physics=freedom.follows is None))
        model.joints.append(plan)
        return plan

    def screw(self, model, name, joint, parent, child, frame):
        """A screw: its turn is the joint's coordinate, and its move follows by the thread's pitch."""
        classification = joint.classification
        turn = next(freedom for freedom in joint.freedoms if freedom.kind == TURN)
        move = next(freedom for freedom in joint.freedoms if freedom.kind == MOVE)
        axis = classification.axes[turn.index]
        along = classification.axes[move.index]
        sense = 1.0 if sum(a * b for a, b in zip(axis, along)) > 0 else -1.0
        if move.follows == turn.index:
            per_degree = move.ratio
        else:
            per_degree = 1.0 / turn.ratio if turn.ratio else 0.0
        # Millimetres a degree, along the turn's own axis, to metres a turn.
        pitch = sense * per_degree * 360.0 / urdf_common.MM_PER_M
        plan = JointPlan(self.names.take(name, "joint"), "screw", parent, child, frame, joint)
        plan.axes.append(AxisPlan(axis, turn, physics=turn.follows is None))
        model.joints.append(plan)
        self._written[joint.name] = [(plan, turn)]
        if abs(pitch) < 1e-12:
            plan.type = "revolute"
            self.warnings.append("The screw '%s' has a thread of nought, and is written as a revolute joint" % name)
        else:
            plan.pitch = pitch
        if turn.follows is not None and set(joint.physics) & JOINT_STATED:
            self.warnings.append(
                "The screw '%s' states its physics for its move, which its turn follows, and an SDFormat screw "
                "states them for its turn: not written" % name
            )
        if move.limited:
            self.warnings.append(
                "The screw '%s' bounds its move, and an SDFormat screw is bounded by its turn: not written" % name
            )
        return plan

    def floating(self, model, name, joint, parent, child, frame):
        """Three moves and a ball, through dummy links: a free joint is a model's, not a link's."""
        classification = joint.classification
        previous = parent
        written, dummies = [], []
        for index in range(3):
            freedom = joint.freedoms[index]
            # Every move leaves the frame the ball turns in where it was, so
            # every dummy link sits there, at zero.
            dummy = LinkPlan(self.names.take("%s_link%d" % (name, index + 1), "link"), frame)
            model.links.append(dummy)
            dummies.append(dummy.name)
            label = name + FREEDOM_SEPARATOR + freedom_label(freedom)
            plan = self.single(model, label, joint, freedom, classification.axes[index], previous, dummy, frame)
            written.append((plan, freedom))
            previous = dummy
        model.links.append(child)
        ball = JointPlan(self.names.take(name + FREEDOM_SEPARATOR + "ball", "joint"), "ball", previous, child, frame)
        model.joints.append(ball)
        self._written[joint.name] = written + [(ball, None)]
        self._ball(name, joint, joint.freedoms[3:])
        self._report_chain(name, joint, [plan.name for plan, _ in written] + [ball.name], dummies)
        self._not_one_axis(joint, "chain")

    def chain(self, model, name, joint, parent, child, base, zero):
        """A joint SDFormat has no type for, as one single-axis joint per freedom through dummy links."""
        freedoms = joint.freedoms
        previous = parent
        written, dummies = [], []
        for index, freedom in enumerate(freedoms):
            if index == len(freedoms) - 1:
                link = child
            else:
                # Where the freedom leaves the frame at zero: the next one's
                # parent, which it turns or slides with.
                link = LinkPlan(
                    self.names.take("%s_link%d" % (name, index + 1), "link"),
                    kinematics.compose(base, joint.frame(index, zero)),
                )
                dummies.append(link.name)
            model.links.append(link)
            label = name + FREEDOM_SEPARATOR + freedom_label(freedom)
            plan = self.single(model, label, joint, freedom, freedom.axis, previous, link, base, zero)
            written.append((plan, freedom))
            previous = link
        self._written[joint.name] = written

        for plan, freedom in written:
            if freedom.follows is None:
                continue
            leader_plan, leader = written[freedom.follows]
            if self.version >= MIMIC_SINCE:
                plan.axes[0].mimic = (
                    leader_plan.name,
                    "axis",
                    freedom.ratio * per_si(leader.kind) / per_si(freedom.kind),
                    to_si(freedom.kind, freedom.offset or 0.0),
                )
            else:
                self.warnings.append(
                    "'%s' follows '%s', and SDFormat %s has no <mimic> to say so (1.11 has): it moves freely"
                    % (plan.name, leader_plan.name, self.version_text)
                )
        self._report_chain(name, joint, [plan.name for plan, _ in written], dummies)
        self._not_one_axis(joint, "chain")

    def _report_chain(self, name, joint, written, dummies):
        self.warnings.append(
            "The %s joint '%s' has no SDFormat type: it is written as the joints %s through the dummy links %s, "
            "each of %g kg" % (joint.kind, name, ", ".join(written), ", ".join(dummies), DUMMY_MASS)
        )

    def _ball(self, name, joint, turns):
        """What a ball cannot state: SDFormat gives it no axis, so nothing that is said per axis."""
        if any(freedom.limited for freedom in turns):
            self.warnings.append(
                "The joint '%s' is written as a ball, which SDFormat cannot bound: the limits of its turns are not "
                "written" % name
            )
        stated = sorted(set(joint.physics) & JOINT_STATED)
        if stated:
            self.warnings.append(
                "The joint '%s' is written as a ball, which SDFormat gives no axis to state them on: its %s are "
                "not written" % (name, ", ".join(stated))
            )

    def _not_one_axis(self, joint, kind):
        if joint.mimic:
            self.warnings.append(
                "The joint '%s' mimics another, and is written as a %s: not written" % (joint.name, kind)
            )

    def _mimic(self, plan, joint):
        if isinstance(joint.mimic, dict) and joint.mimic.get("joint"):
            self._mimics.append((plan, joint.name, dict(joint.mimic)))

    def _resolve_mimics(self):
        for plan, name, mimic in self._mimics:
            leader = self._written.get(str(mimic["joint"])) or []
            if self.version < MIMIC_SINCE:
                self.warnings.append(
                    "The joint '%s' mimics '%s', and SDFormat %s has no <mimic> to say so (1.11 has): it moves freely"
                    % (name, mimic["joint"], self.version_text)
                )
                continue
            if len(leader) != 1 or leader[0][1] is None or len(plan.axes) != 1:
                self.warnings.append(
                    "The joint '%s' mimics '%s', and SDFormat can say that only of a joint of one axis following "
                    "another: it is not written" % (name, mimic["joint"])
                )
                continue
            (leader_plan, leader_freedom), follower = leader[0], plan.axes[0].freedom
            multiplier = _number(mimic.get("multiplier"))
            multiplier = 1.0 if multiplier is None else multiplier
            plan.axes[0].mimic = (
                leader_plan.name,
                "axis",
                multiplier * per_si(leader_freedom.kind) / per_si(follower.kind),
                to_si(follower.kind, _number(mimic.get("offset")) or 0.0),
            )


#
# Writing it
#


def _text(value) -> str:
    """A number as SDFormat states it: twelve significant digits, and no '-0' or rounding noise."""
    value = float(value)
    if abs(value) < 1e-12:
        return "0"
    return "%.12g" % value


def _vector(values) -> str:
    return " ".join(_text(v) for v in values)


def write_model(world, plan, state):
    """Add one ``<model>``: its links, and the joints between them."""
    model = ElementTree.SubElement(world, "model")
    model.set("name", plan.name)
    if not urdf_common.is_identity(plan.pose):
        sub(model, "pose", gazebo_common.format_pose(plan.pose))
    if plan.static:
        # A scene states where things are. Left dynamic, every model in it would
        # start falling the moment the world was loaded, which is not what the
        # scene says.
        sub(model, "static", "true")
    if state["options"]["self_collide"]:
        # SDFormat's links of one model pass through each other unless the
        # model says otherwise. Gazebo's DART never collides the two links of
        # one joint; the two ends of a chain through dummy links are not that.
        sub(model, "self_collide", "true")
    for link in plan.links:
        write_link(model, link, state)
    for joint in plan.joints:
        write_joint(model, joint, state)
    return model


def write_link(model, plan, state):
    """Add one ``<link>``: a visual and a collision per part welded into its body, and the body's inertial."""
    link = ElementTree.SubElement(model, "link")
    link.set("name", plan.name)
    if not urdf_common.is_identity(plan.frame):
        sub(link, "pose", gazebo_common.format_pose(plan.frame))
    if plan.dummy:
        inertia = {key: 0.0 for key in ("ixy", "ixz", "iyz")}
        inertia.update(ixx=DUMMY_INERTIA, iyy=DUMMY_INERTIA, izz=DUMMY_INERTIA)
        write_inertial(link, {"mass": DUMMY_MASS, "inertia": inertia})
        return link

    body = plan.body
    # What the body's own node says decides how its parts touch things where a
    # part says nothing itself: a link of several shapes read from a robot
    # description states its friction once, on the link.
    own = physics_of(body.node, state)
    written = 0
    for index, part in enumerate(body.parts):
        shape = node_geometry(part.node)
        if shape is None:
            continue
        uri = mesh_reference(shape, part.node, plan.name, state)
        pose = None if urdf_common.is_identity(part.placement) else gazebo_common.format_pose(part.placement)
        physics = dict(own)
        physics.update(part.physics)
        for kind in ("visual", "collision"):
            element = ElementTree.SubElement(link, kind)
            element.set("name", "%s_%s_%d" % (plan.name, kind, index))
            if pose is not None:
                sub(element, "pose", pose)
            mesh = sub(element, "geometry/mesh")
            sub(mesh, "uri", uri)
            sub(mesh, "scale", "%s %s %s" % ((repr(MESH_SCALE),) * 3))
            if kind == "visual":
                # The appearance belongs to the part, not to the link: a link
                # built from several parts may well have a colour per part.
                write_material(element, state["properties"].get(part.name) or {})
            else:
                write_surface(element, physics)
        state["unsupported"].update(set(part.physics) - SDF_STATED)
        written += 1

    if not written:
        # A frame that carries others, with no geometry of its own.
        return link

    for name, (path, render) in LINK_PHYSICS.items():
        if name in own:
            sub(link, path, render(own[name]))

    if state["options"]["inertial"]:
        # The body as PartCAD combined it: its parts, each where the body holds
        # it, or what an assembly welded whole into it states it weighs.
        values = carried_inertial(body.physics)
        if values is None:
            state["warnings"].append(
                "The link '%s' has no mass: it states none and has no solid to weigh; "
                "it is written without an <inertial>" % plan.name
            )
        write_inertial(link, values)
    state["unsupported"].update(set(own) - SDF_STATED)
    return link


def write_joint(model, plan, state):
    """Add one ``<joint>``, its frame relative to its child link and its axes expressed in its parent."""
    joint = ElementTree.SubElement(model, "joint")
    joint.set("name", plan.name)
    joint.set("type", plan.type)
    sub(joint, "parent", plan.parent.name if plan.parent is not None else "world")
    sub(joint, "child", plan.child.name)
    pose = kinematics.compose(kinematics.inverse(plan.child.frame), plan.frame)
    if not urdf_common.is_identity(pose):
        sub(joint, "pose", gazebo_common.format_pose(pose))
    if plan.pitch is not None:
        if state["version"] >= SCREW_THREAD_PITCH_SINCE:
            sub(joint, "screw_thread_pitch", _text(plan.pitch))
        else:
            # Radians a metre, positive for a left-handed thread: the turn it
            # takes to move the child a metre, against the right-handed sense.
            sub(joint, "thread_pitch", _text(-2.0 * math.pi / plan.pitch))
    if not plan.axes:
        return joint
    in_parent = kinematics.compose(kinematics.inverse(plan.parent.frame), plan.frame)
    for tag, axis in zip(("axis", "axis2"), plan.axes):
        write_axis(ElementTree.SubElement(joint, tag), axis, kinematics.rotate(in_parent, axis.vector), plan, state)
    return joint


def write_axis(element, axis, vector, plan, state):
    """One axis: its direction, its dynamics, its limits, and what it mimics."""
    xyz = sub(element, "xyz", _vector(vector))
    xyz.set("expressed_in", plan.parent.name)
    freedom = axis.freedom
    kind = freedom.kind
    physics = plan.source.physics if (plan.source is not None and axis.physics) else {}

    def number(key):
        value = _number(physics.get(key))
        if value is None and physics.get(key) is not None:
            state["warnings"].append(
                "The joint '%s' states %s as %r, which is not a number" % (plan.name, key, physics[key])
            )
        return value

    dynamics = {tag: number(key) for key, tag in JOINT_DYNAMICS.items()}
    dynamics["spring_reference"] = to_si(kind, number(SPRING_REFERENCE))
    if any(value is not None for value in dynamics.values()):
        block = sub(element, "dynamics")
        for tag in ("damping", "friction", "spring_reference", "spring_stiffness"):
            if dynamics[tag] is not None:
                sub(block, tag, _text(dynamics[tag]))

    effort = number("maxEffort")
    velocity = to_si(kind, number("maxVelocity"))
    if freedom.limited or effort is not None or velocity is not None:
        # Both bounds, an open one as SDFormat's own "none": '<lower>' and
        # '<upper>' are required wherever there is a '<limit>' at all.
        lower = to_si(kind, freedom.lower) if freedom.lower is not None else -NO_LIMIT
        upper = to_si(kind, freedom.upper) if freedom.upper is not None else NO_LIMIT
        limit = sub(element, "limit")
        sub(limit, "lower", _text(min(lower, upper)))
        sub(limit, "upper", _text(max(lower, upper)))
        if effort is not None:
            sub(limit, "effort", _text(effort))
        if velocity is not None:
            sub(limit, "velocity", _text(velocity))

    if axis.mimic is not None:
        leader, leader_axis, multiplier, offset = axis.mimic
        mimic = ElementTree.SubElement(element, "mimic")
        mimic.set("joint", leader)
        mimic.set("axis", leader_axis)
        sub(mimic, "multiplier", _text(multiplier))
        sub(mimic, "offset", _text(offset))
        sub(mimic, "reference", "0")


def process(path, request):
    root = tree_of(request)
    if not isinstance(root, dict) or not (
        ocp_serialize.is_shape_object(root) or ocp_serialize.is_assembly_object(root)
    ):
        raise ValueError("The world exporter needs a shape or a scene to export")

    world_dir = os.path.dirname(os.path.abspath(path)) or "."
    stem = os.path.splitext(os.path.basename(path))[0] or "world"
    world_name = sanitize_name(request.get("world_name") or root.get("label") or stem, stem)
    if world_name == RESERVED_NAME:
        # Gazebo refuses to load a world of that name, and a scene called
        # "world" is the most natural thing there is to call one.
        world_name = "partcad_" + world_name
    mesh_dir_name = request.get("mesh_dir") or "%s_meshes" % stem
    mesh_dir = os.path.join(world_dir, mesh_dir_name)
    os.makedirs(mesh_dir, exist_ok=True)

    warnings = []
    if request.get("flatten") is not None:
        warnings.append(FLATTEN_GONE)
    version = str(request.get("version") or gazebo_common.SDF_VERSION)
    state = {
        # Models, links and joints: one namespace, so that no two of them
        # answer to one name in what Gazebo reports.
        "names": NameAllocator("-", 2, reserved=(RESERVED_NAME,)),
        "mesh_names": NameAllocator(),
        "meshes": {},
        "mesh_dir": mesh_dir,
        "mesh_dir_name": mesh_dir_name,
        # Empty by default, which makes the reference relative to the world file
        # itself - what a standalone world wants. A Gazebo model database sets
        # this to "model://<name>/" so the meshes resolve through it.
        "mesh_prefix": request.get("mesh_prefix") or "",
        # Shape full name -> the properties its part declares ('physics',
        # 'material', 'color'). A part that came from a world states the link's
        # mass, inertia and friction here, and they go back out rather than
        # being recomputed.
        "properties": request.get("properties") or {},
        "unsupported": set(),
        "version": version_of(version),
        "options": {
            "tolerance": request.get("tolerance", 0.1),
            "angularTolerance": request.get("angularTolerance", 0.1),
            "ascii": request.get("ascii", False),
            "inertial": request.get("inertial", True),
            "static": request.get("static", True),
            "self_collide": bool(request.get("self_collide", False)),
        },
        "warnings": warnings,
    }

    sdf = ElementTree.Element("sdf")
    sdf.set("version", version)
    world = ElementTree.SubElement(sdf, "world")
    world.set("name", world_name)

    # What the scene says about its world. Nothing at all for one that says
    # nothing, so that a world written from such a scene is the file it always
    # was and Gazebo applies its own gravity, SDFormat's [0, 0, -9.8].
    scene_world = request.get("world") or {}
    gravity = gravity_of(request, scene_world)
    if gravity is not None:
        sub(world, "gravity", " ".join(repr(v) for v in gravity))
    medium = scene_world.get("medium")
    if medium:
        warnings.append(MEDIUM_NOT_WRITTEN % (medium.get("material") or "a fluid"))

    # Neither is part of what the scene says; both are what makes the file
    # usable. A world with no light renders black and a world with no ground
    # has nothing to stand on, so a simulator's own empty world ships with
    # both. Turn either off with 'sun: false' / 'ground_plane: false'.
    if request.get("sun", True):
        add_sun(world)
    if request.get("ground_plane", True):
        add_ground_plane(world)
        state["names"].take("ground_plane")

    # The bodies, parents first, as PartCAD's body rule finds them in the tree;
    # what the rule set aside (a 'fixed' inside an assembly, a joint inside a
    # scene) is said rather than written.
    found = kinematics.bodies(root, request.get("properties"))
    warnings.extend(found.problems)
    planner = Planner(found, state["names"], version, state["options"]["static"], warnings).plan()
    state["unsupported"].update(planner.unsupported)
    for model in planner.models:
        write_model(world, model, state)

    ElementTree.indent(sdf, space="  ")
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(ElementTree.tostring(sdf, encoding="unicode"))
        f.write("\n")

    return {
        "success": True,
        "exception": None,
        "world_name": world_name,
        "mesh_dir": mesh_dir,
        "meshes": sorted(set(state["meshes"].values())),
        "warnings": warnings,
        # Properties PartCAD holds and SDFormat cannot state. Reported at info
        # level by the caller: nothing is wrong with the file, it just says less
        # than the package does.
        "unsupported": sorted(state["unsupported"]),
    }
