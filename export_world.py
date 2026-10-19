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
  * every joint is one SDFormat joint of SDFormat's own type, by what
    'kinematics.classify' says it is (see 'Planner.jointed', and section 8.13.4
    of PartCAD's simulation design record): a ``revolute`` -- never a
    ``continuous``, which Gazebo's DART builds as a fixed joint, so a turn
    bounded on neither side is a revolute with no ``<lower>`` or ``<upper>`` --,
    a ``prismatic``, a ``screw`` with its thread pitch, a ``universal`` and a
    ``ball``. Those are the five Harmonic's DART builds. A cylindrical, a planar
    (either order) or a composite joint has no SDFormat type, and the export
    **refuses** it ('Refused') rather than write it as something it is not:
    the links of a gram it used to put between the freedoms stood for bodies a
    mechanism really has, with a mass nobody measured.
  * the child of a floating joint holds nothing and is held by nothing: it is a
    ``<model>`` of its own, at its start, with no joint at all -- SDFormat's
    free body. The body it was connected to is what a run reads its pose
    against.

**Names.** Every name is PartCAD's wherever SDFormat can hold it, and is
otherwise transcoded by PartCAD's 'naming' under SDFormat's own rules (see
'SDF_NAMES'): a scene called 'world' -- which is what PartCAD's default world is
called -- is still 'world' in PartCAD, and a world SDFormat can load under
another name here. One name is never two things in one world: no link is named
like a model, and no joint like a link. SDFormat requires the second within a
model, and Gazebo needs the first: it reports every pose by the entity's bare
name, and a model whose link shared its name was reported where its link sits
in it -- at the origin, for ever. So the models, links and joints of a world
share one transcoder, and PartCAD's names are claimed in it before anything
this exporter names itself (a model's '<model>_link', the ground plane), so
that those are the ones told apart. Every joint keeps PartCAD's name, which is
what a validation reads. With ``names_file: true`` -- what the simulation asks
for -- which name each became, where each body and joint is (its model and its
link), and each joint's map back into PartCAD's terms ('kinematics.joint_map')
is written beside the world as '<file>.names.json', and the run reports under
PartCAD's names and in PartCAD's terms by reading it.

**Frames, and where a joint starts.** A link's frame is its body's own, so a
part sits in its link where the body holds it and the inertial is the body's as
PartCAD combined it. SDFormat states no starting value for a joint (see
'Planner.jointed'), so every joint is **re-zeroed** at its start
('kinematics.rezeroed', section 8.13.5 of the design record): every link is
written where the assembly shows it, in its model's frame, and every joint's
frame where that configuration puts it, so that Gazebo's zero of each joint *is*
PartCAD's start. A joint's ``<pose>`` is that frame relative to its child link,
as SDFormat reads it by default; its ``<axis><xyz>`` is expressed in its
*parent* link. Everything stated about a position shifts with the start -- a
bound, a spring's reference, a mimic's offset -- and the joint map in the
sidecar says how to add it back, so a run reports PartCAD's values.

Two things this exporter decides that the file does not say, because a static
arrangement does not say them and a *simulation* needs both:

  * **Whether what is free may move.** A body nothing holds is free, and in
    SDFormat that is a model with no joint to the world. ``static: true`` (the
    default, for a file somebody opens to look at) welds every such body to the
    world instead: a model with nothing moving in it is ``<static>``, as it
    always was -- a floating child included --, and the root of a mechanism
    gets a fixed joint to ``world``, so that the joints *inside* an assembly
    still move -- a pendulum exported on its own still swings on its hinge. A
    body ``fixed`` to the world gets that fixed joint whatever ``static`` says.
    ``static: false`` is what the simulation asks for.
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
import naming  # noqa: E402
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

# What an SDFormat name may be, declared here because it is SDFormat's (see
# PartCAD's 'wrappers/naming.py'). From libsdformat itself ('isReservedName'
# and 'recursiveSiblingNoDoubleColonInNames'): 'world', and every name that
# begins and ends with '__' ('__model__', '__root__'), are reserved for a world,
# a model, a link, a joint and a frame alike, and no element's name may contain
# '::', which is how SDFormat scopes one name inside another. Names are compared
# as written. A model's name must differ from every other frame-defining
# element's in its scope, a link's and a joint's from every other in their model;
# one transcoder for the whole world (see 'Names' above) satisfies all of it.
# The alphabet is narrower than SDFormat's, for Gazebo's sake: a world's and a
# model's names are parts of transport topics ('/world/<world>/pose/info',
# '/world/<world>/model/<model>/joint_state'), where gz-transport refuses
# whitespace, '~', '@', '//' and ':=' and rewrites what it refuses -- so the run
# would listen on a topic the server is not publishing on. ASCII letters,
# digits, '_', '.' and '-' are safe in every one of those places.
SDF_NAMES = naming.Rules(
    "SDFormat",
    allowed=re.compile(r"[A-Za-z0-9_.-]"),
    forbidden=("::",),
    reserved=("world",),
    reserved_patterns=(r"__.*__",),
)

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

# The SDFormat joint each kind of joint 'kinematics.classify()' finds is written
# as: SDFormat's own type, for the five of them Gazebo Harmonic's DART builds
# (gz-physics 7, 'dartsim/src/SDFFeatures.cc', 'ConstructSdfJoint'). DART builds
# anything else -- 'continuous', 'revolute2', 'gearbox' -- as a weld, with a
# 'gzerr' line in the server's log and nothing failing, which is why a turn
# bounded on neither side is a 'revolute' and never a 'continuous'. A floating
# joint is no joint at all: its child is a model of its own (see 'Planner.free').
NATIVE = {
    kinematics.REVOLUTE: "revolute",
    kinematics.PRISMATIC: "prismatic",
    kinematics.SCREW: "screw",
    kinematics.UNIVERSAL: "universal",
    kinematics.BALL: "ball",
}

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
# What a ball's '<axis>' carries. SDFormat's specification gives a ball no axis
# at all; Harmonic's DART applies one's damping, friction, spring and limits to
# each of the three coordinates of its 'BallJoint' ('ConstructBallJoint'). For a
# damper that is a damper about every line through the centre, which is what a
# ball's damping means; for a spring, a friction or a bound it is a box over the
# components of a rotation vector, which is not what any of them means. So the
# damping is written, and the rest is reported (section 8.13.4 of PartCAD's
# simulation design record).
BALL_DYNAMICS = ("damping",)

# The first SDFormat versions with each of these. A screw's pitch is
# '<thread_pitch>' before 1.10 -- radians per metre, positive for a left-handed
# thread, as Gazebo Classic read it and Harmonic still does -- and
# '<screw_thread_pitch>' from 1.10, metres per turn, positive for a right-handed
# one. '<mimic>' is in an axis from 1.11 and nowhere before; Gazebo Harmonic
# reads it and does not enforce it (its DART has no mimic constraint).
SCREW_THREAD_PITCH_SINCE = (1, 10)
MIMIC_SINCE = (1, 11)

FLATTEN_GONE = (
    "'flatten' is no longer an option of this export, and is ignored: which nodes are bodies is PartCAD's "
    "body rule - every element of a scene a body of its own, everything in an assembly attached"
)


class Refused(ValueError):
    """The joints Gazebo has no joint for, which the export will not write as something they are not.

    'joints' is '(name, kind)' for each, every one the world has, found before
    anything is written. The message names them, says why, and gives the
    ways on.
    """

    def __init__(self, joints):
        self.joints = list(joints)
        super().__init__(refusal(self.joints))


def refusal(joints) -> str:
    """Why a world of these joints is not written, and what to do instead (section 8.13.4 of the design record)."""
    named = ", ".join("the %s joint '%s'" % (kind, name) for name, kind in joints)
    return (
        "Gazebo cannot simulate %s: the joints SDFormat has and Gazebo Harmonic's DART builds are revolute, "
        "prismatic, screw, universal and ball, and the only way to write another is through links of a mass nobody "
        "measured, which this export does not invent. Either simulate the mechanism in MuJoCo "
        "(partcad-sim-mujoco), which writes every type; or model the body the joint stands for - for a cylindrical "
        "pair a sleeve and its carriage, for a planar joint the carriage its moves slide, for a composite the "
        "carriage or the ring between its freedoms - and join it by native joints" % named
    )


def _bool_text(value):
    return "true" if value else "false"


def sanitize_name(name, fallback):
    """An SDFormat-safe name derived from a PartCAD name, uniqueness and reservation aside.

    PartCAD names carry package paths and ':' separators ("//pub/examples:logo")
    and the '/' that groups a link's own shapes, none of which belong in an
    SDFormat name -- '::' is its scope separator and a bare '/' has no meaning.
    """
    return naming.transcode(SDF_NAMES, name, fallback)


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
        mesh_name = state["mesh_names"].derive(node.get("label") or link_name, "mesh")
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
    """What PartCAD says about the physics of the object behind one node of the tree.

    Looked up by 'kinematics.properties_of', not by the node's full name: an
    assembly or a scene whose full name a part shares -- the assembly 'nut'
    that holds the part 'nut' -- has an entry of its own under its kind
    ('kinematics.properties_key'), and looked up by name it got the part's
    contact settings. A part is under its full name, as it always was.
    """
    return (kinematics.properties_of(state["properties"], node) or {}).get("physics") or {}


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


def add_ground_plane(world, names=None):
    """The ground every Gazebo world is normally loaded with.

    Its model and link are named out of 'names', the world's transcoder, when
    there is one: Gazebo reports every pose by the bare name, so a body called
    'link' must not share its name with the plane's link.
    """
    model = ElementTree.SubElement(world, "model")
    model.set("name", names.derive("ground_plane") if names is not None else "ground_plane")
    sub(model, "static", "true")
    link = ElementTree.SubElement(model, "link")
    link.set("name", names.derive("link") if names is not None else "link")
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
    return kinematics.per_si(kind)


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


def version_of(text):
    """An SDFormat version as numbers, '1.11' -> (1, 11): what decides which elements a joint is written with."""
    try:
        return tuple(int(part) for part in str(text).strip().split("."))
    except ValueError:
        return tuple(int(part) for part in gazebo_common.SDF_VERSION.split("."))


def _pose_data(pose):
    """A pose as plain data for the sidecar: '[[w, x, y, z], [x, y, z]]', millimetres."""
    return [[float(v) for v in pose[0]], [float(v) for v in pose[1]]]


class LinkPlan:
    """One SDFormat link: one body.

    'frame' is where the link is in its model's frame -- which is its model's
    root body's frame -- with every joint at its start, which is Gazebo's zero
    of every joint (see 'Planner.jointed').
    """

    def __init__(self, name, frame, body):
        self.name = name
        self.frame = frame
        self.body = body

    def __repr__(self):
        return "<LinkPlan %s>" % self.name


class AxisPlan:
    """One '<axis>' or '<axis2>' of a joint: a unit vector in the joint's frame, and the freedom it moves.

    'freedom' is the 'kinematics.RezeroedFreedom' the axis is the coordinate
    of -- its bounds, its start and its coupling in the engine's terms -- or
    None for a ball's '<axis>', which is no coordinate and carries its damping
    alone (see 'BALL_DYNAMICS').
    """

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

    def __init__(self, name, joint_type, parent, child, frame, source=None, rezeroed=None):
        self.name = name
        self.type = joint_type
        # A LinkPlan, or None for the world.
        self.parent = parent
        self.child = child
        # The frame the joint turns or slides in, in the model's frame, at
        # Gazebo's zero -- the joint's start.
        self.frame = frame
        self.axes = []
        # A screw's: the metres a right-handed turn about its '<axis>' moves
        # the child along it.
        self.pitch = None
        # The 'kinematics.Joint' it is written for, whose physics it states,
        # and the same joint re-zeroed at its start ('kinematics.Rezeroed'),
        # whose frames, bounds and offsets are what is written.
        self.source = source
        self.rezeroed = rezeroed

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
    the tests check it against the joints it came from, frame by frame. 'plan'
    raises 'Refused', naming every joint Gazebo has no joint for at once, before
    anything is written.
    """

    def __init__(self, found, names=None, version=None, static=True, warnings=None):
        self.found = found
        self.names = names if names is not None else naming.Transcoder(SDF_NAMES)
        self.version_text = str(version or gazebo_common.SDF_VERSION)
        self.version = version_of(self.version_text)
        self.static = static
        self.warnings = warnings if warnings is not None else []
        self.unsupported = set()
        self.models = []
        # (name, kind) of every joint SDFormat has no type for: 'Refused'.
        self.refused = []
        # Body -> (its ModelPlan, its LinkPlan).
        self._placed = {}
        # PartCAD joint name -> the JointPlan it became: what a 'mimic' looks
        # its leader up in.
        self._written = {}
        # The name a run reports each joint by -> where it is (its model's and
        # its own name), the PartCAD joint it is, and its map back into
        # PartCAD's terms: the sidecar's 'joints'. See 'joint_name'.
        self.reported = {}
        self._mimics = []

    def written(self):
        """The bodies this writes: every one with something in it or under it."""
        return [body for body in self.found if body.parts or body.children]

    def claim_bodies(self):
        """Claim every body's name before anything is derived, so that a derived name is the one told apart."""
        self.names.claim_all((body.name for body in self.written()), "body")

    def plan(self):
        self.claim_bodies()
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
                if body.is_floating:
                    self.free(body)
            elif id(body.parent) in self._placed:
                self.jointed(body)
        if self.refused:
            raise Refused(self.refused)
        for model in self.models:
            self.weld(model)
        self._resolve_mimics()
        return self

    def root(self, body):
        """A model for a body nothing holds, at that body's pose -- for a floating child, where it starts."""
        model = ModelPlan(self.names.claim(body.name, "body", "model"), body.world, body)
        link = LinkPlan(self.names.derive("%s_link" % model.name, "link"), IDENTITY, body)
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
        name = self.names.derive("%s-world" % model.name, "joint")
        model.joints.insert(0, JointPlan(name, "fixed", None, root, root.frame))

    def free(self, body):
        """The child of a floating joint: a model of its own ('root'), and what a run reads its joint by.

        No joint is written -- a link with no joint is SDFormat's free body, and
        DART gives it a 'FreeJoint' -- so the joint is in the sidecar alone: its
        map ('kinematics.joint_map') says which body its pose is read against,
        and 'body' which body it is.
        """
        joint = body.joint
        name = joint.name or "%s-%s" % (body.name, body.joint_parent.name)
        self.reported[name] = {"joint": joint.name, "body": body.name, "map": kinematics.joint_map(joint)}
        if any(freedom.limited for freedom in joint.freedoms):
            self.warnings.append(
                "The floating joint '%s' bounds its freedoms, and a free body has no bounds: not written" % name
            )
        stated = sorted(set(joint.physics))
        if stated:
            self.warnings.append(
                "The floating joint '%s' holds nothing, so there is nothing to state its %s on: not written"
                % (name, ", ".join(stated))
            )
        self._not_one_axis(joint, "free body")

    def jointed(self, body):
        """The joint of 'body' to its parent, re-zeroed at its start, and its link -- or a refusal.

        Re-zeroed rather than written at PartCAD's zero with its start beside
        it, because SDFormat states no starting value: '<axis><initial_position>'
        was in 1.6 and 1.7, deprecated in 1.8 and gone from 1.9 on. Nor does
        Gazebo Harmonic's gz-sim 8 start a joint anywhere a world says
        ('SdfEntityCreator.cc' reads none): the system that would set one from
        a '<model_state>' is gz-sim #2359, an unmerged draft. So the engine's
        zero of every joint is made PartCAD's start instead
        ('kinematics.rezeroed', section 8.13.5 of PartCAD's simulation design
        record): the link where the assembly shows the body, the joint's frame
        where that puts it, every position the joint states -- a bound, a
        spring's reference, a mimic's offset -- shifted by the start, and the
        joint map in the sidecar ('kinematics.joint_map') saying how to shift a
        reading back. The URDF exporter is the one that keeps PartCAD's zero,
        because what reads a URDF takes a joint's zero to be the hardware's; a
        world is PartCAD's own, and only PartCAD's run reads its joints back.
        """
        model, parent = self._placed[id(body.parent)]
        joint = body.joint
        name = joint.name or "%s-%s" % (body.name, body.parent.name)
        kind = joint.kind
        base = parent.frame
        # Where the assembly shows the body: its joint at its start, which is
        # where Gazebo's zero of the joint is.
        child = LinkPlan(self.names.claim(body.name, "body", "link"), kinematics.compose(base, joint.location()), body)
        # Placed even when its joint is refused, so that what hangs off it is
        # planned too, and every refusal is named at once.
        self._placed[id(body)] = (model, child)
        if kind not in NATIVE:
            self.refused.append((name, kind))
            return
        classification = joint.classification
        shifted = kinematics.rezeroed(joint)
        model.links.append(child)
        self.unsupported.update("joint %s" % key for key in set(joint.physics) - JOINT_STATED)
        if joint.soft_limits:
            self.unsupported.add("joint softLimits")

        plan = JointPlan(
            self.joint_name(model, name, joint),
            NATIVE[kind],
            parent,
            child,
            kinematics.compose(base, shifted.origin),
            joint,
            shifted,
        )
        model.joints.append(plan)
        self._written[joint.name] = plan
        if kind == kinematics.SCREW:
            joint_map = self.screw(plan, name)
        elif kind == kinematics.BALL:
            joint_map = self.ball(plan, name, parent, child)
        else:
            # A revolute or a prismatic joint's one axis, a universal joint's
            # two: each its freedom's line in the joint's frame, which
            # re-zeroing leaves where it was (8.13.5).
            plan.axes = [AxisPlan(classification.axes[i], freedom) for i, freedom in enumerate(shifted.freedoms)]
            joint_map = kinematics.joint_map(joint)
        # A start outside the bounds -- which Gazebo would throw against its
        # stop at the first step -- is PartCAD's to report, and is among the
        # problems 'kinematics.bodies()' found ('outside_bounds').
        self.reported[name]["map"] = joint_map
        if kind in (kinematics.UNIVERSAL, kinematics.BALL):
            self._not_one_axis(joint, kind)
        else:
            self._mimic(plan, joint)

    def screw(self, plan, name):
        """A screw: its turn is the joint's coordinate, and its move follows by the thread's pitch.

        Returns its joint map, which says so: the turn the engine's
        coordinate, and the move a follower of it -- 'kinematics.joint_map'
        reads a screw by its turn whichever of its two freedoms leads.
        """
        joint, shifted = plan.source, plan.rezeroed
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
        plan.axes.append(AxisPlan(axis, shifted.freedoms[turn.index], physics=turn.follows is None))
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
        return kinematics.joint_map(joint)

    def ball(self, plan, name, parent, child):
        """A ball: no coordinate a joint state can carry, so read from its two links' poses.

        Returns its joint map, and records beside it -- under 'ball' in the
        sidecar's entry -- the two bodies and the joint's frame in each of
        their links, which is what a run turns their poses into the ball's
        rotation by: the engine's rotation is the child's joint frame against
        the parent's, both at Gazebo's zero where the ball starts.
        """
        joint = plan.source
        if any(key in joint.physics for key in BALL_DYNAMICS):
            plan.axes.append(AxisPlan((0.0, 0.0, 1.0), None))
        dropped = sorted((set(joint.physics) & JOINT_STATED) - set(BALL_DYNAMICS))
        if dropped:
            self.warnings.append(
                "The ball '%s' is given an <axis> for its damping alone, which Gazebo's DART applies to each of its "
                "three coordinates: its %s would be a box over those coordinates, which is not what they mean for a "
                "ball, and are not written" % (name, ", ".join(dropped))
            )
        if joint.turn_limit is not None:
            self.warnings.append(
                "The ball '%s' is bounded at %g deg from its zero, and SDFormat has no bound for a ball (Gazebo's "
                "DART would bound each of its three coordinates, which is a box and not that cone): not written"
                % (name, joint.turn_limit)
            )
        self.reported[name]["ball"] = {
            "parent": parent.body.name,
            "child": child.body.name,
            "in_parent": _pose_data(kinematics.compose(kinematics.inverse(parent.frame), plan.frame)),
            "in_child": _pose_data(kinematics.compose(kinematics.inverse(child.frame), plan.frame)),
        }
        return kinematics.joint_map(joint)

    def joint_name(self, model, name, joint):
        """The SDFormat name of the joint a run reports as 'name' (PartCAD's), recorded for the sidecar."""
        internal = self.names.claim(name, "joint", "joint")
        self.reported[name] = {"path": [model.name, internal], "joint": joint.name}
        return internal

    def bodies(self):
        """The sidecar's 'bodies': where each PartCAD body is, its model and its link, and which parts it is.

        The parts ('kinematics.parts_entry()') are what a run reports as
        'parts', so that the loads it reads on a body can be read as a part's.
        """
        out = {}
        for model in self.models:
            for link in model.links:
                out[link.body.name] = {"path": [model.name, link.name], "parts": kinematics.parts_entry(link.body)}
        return out

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
            leader = self._written.get(str(mimic["joint"]))
            if self.version < MIMIC_SINCE:
                self.warnings.append(
                    "The joint '%s' mimics '%s', and SDFormat %s has no <mimic> to say so (1.11 has): it moves freely"
                    % (name, mimic["joint"], self.version_text)
                )
                continue
            if leader is None or len(leader.axes) != 1 or leader.axes[0].freedom is None or len(plan.axes) != 1:
                self.warnings.append(
                    "The joint '%s' mimics '%s', and SDFormat can say that only of a joint of one axis following "
                    "another: it is not written" % (name, mimic["joint"])
                )
                continue
            leading, following = leader.axes[0].freedom, plan.axes[0].freedom
            multiplier = _number(mimic.get("multiplier"))
            multiplier = 1.0 if multiplier is None else multiplier
            # 'v_f = m v_l + c' in PartCAD's values is 'e_f = m e_l + (c + m s_l - s_f)' in the engine's, both
            # joints re-zeroed at their starts (8.13.5).
            offset = kinematics.rezeroed_offset(
                _number(mimic.get("offset")) or 0.0, multiplier, leading.start, following.start
            )
            plan.axes[0].mimic = (
                leader.name,
                "axis",
                multiplier * per_si(leading.kind) / per_si(following.kind),
                to_si(following.kind, offset),
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
        # one joint; two links of one mechanism that no joint joins -- a part
        # on a turning arm and the frame the arm turns in -- collide only so.
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
    """One axis: its direction, its dynamics, its limits, and what it mimics -- in the engine's terms.

    Everything that is a position of the joint is the re-zeroed joint's
    ('plan.rezeroed'): a bound and a spring's reference are PartCAD's less the
    start, so that they are where PartCAD says they are while Gazebo counts the
    joint from its start (section 8.13.5 of the design record).
    """
    xyz = sub(element, "xyz", _vector(vector))
    xyz.set("expressed_in", plan.parent.name)
    freedom = axis.freedom
    physics = plan.source.physics if (plan.source is not None and axis.physics) else {}

    def number(key):
        value = _number(physics.get(key))
        if value is None and physics.get(key) is not None:
            state["warnings"].append(
                "The joint '%s' states %s as %r, which is not a number" % (plan.name, key, physics[key])
            )
        return value

    def write_dynamics(dynamics):
        if any(value is not None for value in dynamics.values()):
            block = sub(element, "dynamics")
            for tag in ("damping", "friction", "spring_reference", "spring_stiffness"):
                if dynamics.get(tag) is not None:
                    sub(block, tag, _text(dynamics[tag]))

    if freedom is None:
        # A ball's, which is no coordinate: its damping alone (see
        # 'BALL_DYNAMICS'), and nothing that is a position.
        write_dynamics({JOINT_DYNAMICS[key]: number(key) for key in BALL_DYNAMICS})
        return

    kind = freedom.kind
    dynamics = {tag: number(key) for key, tag in JOINT_DYNAMICS.items()}
    if number(SPRING_REFERENCE) is not None:
        dynamics["spring_reference"] = to_si(kind, plan.rezeroed.spring_reference(freedom.index))
    write_dynamics(dynamics)

    effort = number("maxEffort")
    velocity = to_si(kind, number("maxVelocity"))
    lower, upper = to_si(kind, freedom.lower), to_si(kind, freedom.upper)
    if lower is not None and upper is not None:
        lower, upper = min(lower, upper), max(lower, upper)
    if lower is not None or upper is not None or effort is not None or velocity is not None:
        # A side that is open is left out. From SDFormat 1.10 on an omitted
        # '<lower>' is -inf and an omitted '<upper>' +inf, which DART takes as
        # no stop on that side ('CopyStandardJointAxisProperties'); 1.9's
        # default was 1e16, which this used to write out (section 8.13.6).
        limit = sub(element, "limit")
        if lower is not None:
            sub(limit, "lower", _text(lower))
        if upper is not None:
            sub(limit, "upper", _text(upper))
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
    # The scene's own name, 'world' included: PartCAD's default world is called
    # that, and SDFormat reserves it, so the world is written under another
    # name and keeps PartCAD's everywhere a run reports it.
    scene_name = str(request.get("world_name") or root.get("label") or stem)
    scene_names = naming.Transcoder(SDF_NAMES)
    world_name = scene_names.claim(scene_name, "scene", stem)
    mesh_dir_name = request.get("mesh_dir") or "%s_meshes" % stem
    mesh_dir = os.path.join(world_dir, mesh_dir_name)

    warnings = []
    if request.get("flatten") is not None:
        warnings.append(FLATTEN_GONE)
    version = str(request.get("version") or gazebo_common.SDF_VERSION)
    state = {
        # Models, links and joints: one namespace, so that no two of them
        # answer to one name in what Gazebo reports.
        "names": naming.Transcoder(SDF_NAMES),
        # A mesh's name is also its file's, so it answers to the filesystem's
        # rules rather than SDFormat's; 'cube_1.stl' after 'cube.stl'.
        "mesh_names": naming.Transcoder(naming.FILE_NAMES, "_", 1),
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
    #
    # The bodies, parents first, as PartCAD's body rule finds them in the tree;
    # what the rule set aside (a 'fixed' inside an assembly, a joint inside a
    # scene) is said rather than written. Their names are claimed before the
    # ground plane's, which is the one told apart from a body called that.
    found = kinematics.bodies(root, request.get("properties"))
    warnings.extend(found.problems)
    planner = Planner(found, state["names"], version, state["options"]["static"], warnings)
    planner.claim_bodies()
    if request.get("sun", True):
        add_sun(world)
    if request.get("ground_plane", True):
        add_ground_plane(world, state["names"])
    # Planned before anything is written: a refusal ('Refused') leaves nothing
    # behind, not even an empty directory for the meshes.
    planner.plan()
    os.makedirs(mesh_dir, exist_ok=True)
    state["unsupported"].update(planner.unsupported)
    for model in planner.models:
        write_model(world, model, state)
    renamed = naming.renamed_warning(SDF_NAMES, scene_names, state["names"])
    if renamed:
        warnings.append(renamed)

    ElementTree.indent(sdf, space="  ")
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(ElementTree.tostring(sdf, encoding="unicode"))
        f.write("\n")
    if request.get("names_file"):
        naming.write_sidecar(
            path,
            "world",
            naming.merged_mapping(scene_names, state["names"]),
            planner.bodies(),
            dict(planner.reported),
        )
    else:
        naming.remove_sidecar(path)

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
