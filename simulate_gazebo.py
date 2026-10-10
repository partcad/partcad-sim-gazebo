#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""The Gazebo simulation plugin (see 'partcad.yaml' beside this file).

Runs a world under gravity for a while and says where everything ended up.

The world arrives as SDFormat -- PartCAD exported it with this package's own
exporter, with every model free to move -- so all this does is start the server,
watch the poses it publishes, and take the same reading twice: once before
anything has moved and once when the simulated clock reaches ``duration``. That
pair is what a ``simulate:``'s ``validation:`` expression is handed, and it is
the whole of what PartCAD requires a simulation plugin to produce.

Positions are reported in **millimetres**, PartCAD's unit everywhere, not in the
metres SDFormat works in. A validation expression is written by whoever wrote the
part, against the numbers that part is drawn in.

**Where Gazebo comes from.** Not from pip: there is no wheel that carries a
Gazebo, which is the one way this differs from the MuJoCo plugin. So it is found
the same two ways ``pc open --with gazebo`` finds it -- the ``gz`` on this
machine, and otherwise the official image, which ``dockerImage`` in
``partcad.yaml`` names and PartCAD's ``docker`` sandbox is built from. Either way
the binary is on ``PATH`` by the time this runs, and a machine with neither is
told which of the two to arrange rather than shown a traceback.

**Where a ROS installation put it.** ROS installs Gazebo as vendor packages,
with ``gz`` under ``/opt/ros/<distro>/opt/gz_tools_vendor/bin`` and nothing on
``PATH`` until ROS's setup script has run -- which is what the official
``osrf/ros:*-simulation`` images are, and what ``dockerImage`` names. So when no
``gz`` is on ``PATH``, the environment that script leaves is used to look again
and to run it; see 'ros_environment'.

**Snapshots.** PartCAD also asks for two pictures of the world, before and
after, from the viewpoint it names. They are drawn on the CPU by
'snapshot_raster.py' -- Gazebo's own renderer needs a GPU or EGL, which a
sandbox does not have -- out of the world file Gazebo ran, each visual placed
where the pose messages say its model and its link were at the first reading
and at the last; see 'world_layout' and 'arrange'.

**Why the clock is read out of the messages.** ``gz sim -r`` runs at a real-time
factor of about one, so waiting ten wall-clock seconds is *nearly* ten seconds of
simulation -- and "nearly" is not something a validation should depend on. Every
pose message carries the simulated time it was taken at, so this waits for that
to pass ``duration`` instead, and uses wall-clock only as the timeout that stops
a run which is not progressing at all.

**Joints.** A world whose models have joints in them -- a hinge, a slide -- is
also read for where each joint is and how fast it moves, as ``joints`` beside
``bodies``, in the same vocabulary the MuJoCo plugin states them in: degrees
for a turn, millimetres for a move, the terms of the ``motion:`` an interface
declares. Gazebo publishes joint states only for a model that carries its
JointStatePublisher system, so the world that runs is a copy of the one handed
over with that system added; see 'plan_joints' and 'JointStream'.
"""

import collections
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from xml.etree import ElementTree

# 'snapshot_raster' and 'gazebo_common' are this package's, beside this file.
# PartCAD runs this script by path, which puts nothing on sys.path for it.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Millimetres per metre: SDFormat is metres by definition, PartCAD is
# millimetres throughout. Spelled out rather than imported from PartCAD's own
# 'urdf_common' because this script runs in a sandbox that may carry nothing but
# Gazebo -- and because a number this stable is not worth a dependency.
MM_PER_M = 1000.0

# Degrees per radian. Gazebo states every angle in radians; PartCAD states a
# turn in degrees, as the 'motion:' of an interface does, its limits included.
DEG_PER_RAD = 180.0 / math.pi

# The programs that are a Gazebo, newest first, what each one calls the
# subcommand that runs a server, and what it calls the one that prints a topic.
# Both are subcommands of the one program -- 'gz sim', 'gz topic' -- not programs
# of their own. The same table `open:` in 'partcad.yaml' declares for `pc open`,
# for the same reason: one answer to "is there a Gazebo on this machine",
# whichever generation of it is installed.
SERVERS = (
    ("gz", ["sim"], "topic"),
    ("ign", ["gazebo"], "topic"),
)

# What to say when there is neither. Both halves, because both are real answers
# and which one a user wants depends on their machine rather than on this run.
NO_GAZEBO = (
    "No Gazebo found: neither 'gz' nor 'ign' is on PATH. Install one "
    "(https://gazebosim.org/docs) or let PartCAD run this in a container by "
    "using the 'docker' Python sandbox, which is built from the image this "
    "package's 'dockerImage' names."
)


class GazeboMissing(Exception):
    """Raised where no Gazebo could be found, so the message is the whole report."""


# Where ROS keeps the setup scripts that put its Gazebo on PATH, newest
# distribution first by name.
ROS_SETUP = "/opt/ros/*/setup.bash"


def ros_environment():
    """The environment a ROS installation's setup script leaves, or None.

    ROS installs Gazebo as vendor packages and puts ``gz`` on ``PATH`` -- and
    the libraries and plugins it loads on their search paths -- only once its
    setup script has been sourced. A container started from a ROS image by
    anything but its own entrypoint has none of that, so it is asked for here:
    the script is run in a shell and what it leaves in the environment is read
    back, for running the server and the topic tool with.
    """
    for setup in sorted(glob.glob(ROS_SETUP), reverse=True):
        try:
            answer = subprocess.run(
                ["bash", "-c", 'source "$0" >/dev/null 2>&1 && env -0', setup],
                capture_output=True,
                timeout=60,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        if answer.returncode != 0:
            continue
        environment = {}
        for entry in answer.stdout.split(b"\0"):
            name, separator, value = entry.decode("utf-8", "replace").partition("=")
            if separator and name:
                environment[name] = value
        if environment.get("PATH"):
            return environment
    return None


def find_gazebo(path=None):
    """The Gazebo on this machine, as (binary, server args, topic subcommand).

    The server is ``<binary> <server args>`` and the topic tool is
    ``<binary> <topic subcommand>``: one program, two of its subcommands.

    'PC_GZ' overrides the search, for a machine with an installation the name
    does not give away. It names the binary.

    'path' is the ``PATH`` to search instead of this process's own -- the one a
    ROS setup script leaves (see 'ros_environment').
    """
    override = os.environ.get("PC_GZ")
    if override:
        name = os.path.basename(override)
        # The table entry for whichever generation it is, and the current one
        # for a name this table has never heard of: an override is an override,
        # and refusing it because the binary has an unfamiliar name would defeat
        # the point of having one.
        _, args, topic = next((entry for entry in SERVERS if entry[0] == name), SERVERS[0])
        return override, args, topic

    for binary, args, topic in SERVERS:
        found = shutil.which(binary, path=path)
        if found:
            return found, args, topic
    raise GazeboMissing(NO_GAZEBO)


def locate_gazebo():
    """The Gazebo to run and the environment to run it in (None: this one).

    ``PATH`` first, and then whatever a ROS installation's setup script puts on
    it: a Gazebo that is there is used as it is, and one that ROS installed is
    run with the environment ROS says it needs.
    """
    try:
        return find_gazebo(), None
    except GazeboMissing:
        environment = ros_environment()
        if environment is None:
            raise
        return find_gazebo(environment["PATH"]), environment


def world_name(path):
    """The name of the world in an SDFormat file, for the topics it publishes on.

    Gazebo namespaces every topic under the world's name, so this has to be
    right before anything can be read. Parsed with the standard library rather
    than asked of the server, because it is one attribute and asking would mean
    the server had to be up before we knew what to ask it.
    """
    from xml.etree import ElementTree

    root = ElementTree.parse(path).getroot()
    element = root if root.tag == "world" else root.find("world")
    name = element.get("name") if element is not None else None
    if not name:
        raise Exception("The world file names no world, so nothing can be read from it: %s" % path)
    return name


#
# Reading what the server publishes
#
# 'gz topic -e' prints protobuf text format. A 'gz.msgs.Pose_V' is a 'header'
# followed by one 'pose' per entity, and the tool prints one message after
# another with nothing between them -- so a message starts wherever a 'header {'
# appears at the top level, which is what is split on below. Parsing the text
# form rather than the wire form is deliberate: the wire form needs the Gazebo
# protobuf bindings, which are not on PyPI and are not in every image.
#

_FIELD = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?:\{|:\s*(.+?))\s*$")


def parse_message(text):
    """One protobuf text-format message, as nested dicts and lists.

    Repeated fields become lists, which is what 'pose' is and what makes a
    message readable as "every entity at this instant".
    """
    result = {}
    stack = [result]
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "}":
            if len(stack) > 1:
                stack.pop()
            continue
        match = _FIELD.match(line)
        if match is None:
            continue
        key, value = match.group(1), match.group(2)
        if value is None:
            child = {}
            _append(stack[-1], key, child)
            stack.append(child)
        else:
            _append(stack[-1], key, _scalar(value))
    return result


def _append(container, key, value):
    """Add a field, turning it into a list the second time it is seen."""
    if key not in container:
        container[key] = value
    elif isinstance(container[key], list):
        container[key].append(value)
    else:
        container[key] = [container[key], value]


def _scalar(text):
    """A text-format scalar as the Python value it stands for."""
    text = text.strip()
    if len(text) >= 2 and text[0] == '"' and text[-1] == '"':
        return text[1:-1]
    if text in ("true", "false"):
        return text == "true"
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def _as_list(value):
    if value is None:
        return []
    return value if isinstance(value, list) else [value]


def snapshot(message):
    """Where every entity is in one pose message, keyed by the name Gazebo gave it.

    The world itself is left out: it is the frame everything else is stated in
    and it never moves. What is left is the models PartCAD's exporter wrote out
    of the scene, under the names it gave them - which is what makes a
    validation expression readable.

    An entity Gazebo reports with no name at all is kept under its id rather
    than dropped: something moved, and a reading that quietly omits it is worse
    than one that names it awkwardly.

    Its 'joints' are empty: they are published on topics of their own, and
    'fill' fills them in from those. A world with no joints to
    read keeps it empty, so a validation can walk ``after["joints"]`` without
    asking first whether there is one.
    """
    stamp = (message.get("header") or {}).get("stamp") or {}
    seconds = float(stamp.get("sec") or 0) + float(stamp.get("nsec") or 0) / 1e9

    bodies = {}
    for pose in _as_list(message.get("pose")):
        name = pose.get("name")
        if name in (None, "", "default") and pose.get("id") is not None:
            name = "entity_%s" % pose["id"]
        if not name:
            continue
        position = pose.get("position") or {}
        orientation = pose.get("orientation") or {}
        bodies[name] = {
            "pos": [float(position.get(axis) or 0.0) * MM_PER_M for axis in ("x", "y", "z")],
            # 'w x y z', the order PartCAD writes a quaternion in everywhere.
            # Gazebo prints them in the protobuf's own field order, which is not
            # that one, so this is a re-ordering rather than a copy.
            "quat": [float(orientation.get(axis) or 0.0) for axis in ("w", "x", "y", "z")],
        }
    return {"time": seconds, "bodies": bodies, "joints": {}}


def unique_poses(message):
    """Every entity of one pose message whose name nothing else in it has.

    As a ``(q, t)`` pose in millimetres, relative to the entity's parent --
    which is what Gazebo publishes: a model of the world relative to the world,
    a link relative to its model, a visual relative to its link. Names are only
    unique where the world made them so: every visual of a hand-written world
    may well be called "visual", and a name that answers for two entities
    answers for neither, so it is left out and the world file's own pose is
    used for it instead (see 'arrange').
    """
    seen = {}
    repeated = set()
    for pose in _as_list(message.get("pose")):
        name = pose.get("name")
        if not name:
            continue
        if name in seen:
            repeated.add(name)
            continue
        position = pose.get("position") or {}
        orientation = pose.get("orientation") or {}
        seen[name] = (
            tuple(float(orientation.get(axis) or 0.0) for axis in ("w", "x", "y", "z")),
            tuple(float(position.get(axis) or 0.0) * MM_PER_M for axis in ("x", "y", "z")),
        )
    for name in repeated:
        del seen[name]
    return seen


def world_layout(scene_file, warnings):
    """What the world looks like, as the tree its poses are stated along.

    Each model holds its links and its nested models, each link its visuals,
    and each of those its pose relative to its parent -- in millimetres, read by
    the same 'gazebo_common' the world reader uses. A visual is triangles in its
    own frame (a mesh, or a primitive tessellated), or a plane, which is the
    floor and is drawn as one.
    """
    import numpy as np
    from xml.etree import ElementTree

    import gazebo_common
    import snapshot_raster

    root = ElementTree.parse(scene_file).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise Exception("The world file holds no world: %s" % scene_file)
    directory = os.path.dirname(os.path.abspath(scene_file))
    model_paths = [p for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep) if p]
    meshes = {}

    def color(visual):
        for path in ("material/diffuse", "material/ambient"):
            text = visual.findtext(path)
            if text:
                try:
                    values = [float(v) for v in text.split()]
                except ValueError:
                    continue
                if len(values) >= 3:
                    return values[:3]
        return None

    def geometry(visual):
        element = visual.find("geometry")
        if element is None or len(element) == 0:
            return None
        shape = element[0]
        number = lambda path, default: float(shape.findtext(path) or default)  # noqa: E731
        if shape.tag == "mesh":
            uri = shape.findtext("uri")
            path = gazebo_common.resolve_uri(uri, directory, model_paths)
            if path is None or not os.path.isfile(path) or not path.lower().endswith(".stl"):
                warnings.append("a mesh the snapshots cannot draw is left out of them: %s" % uri)
                return None
            if path not in meshes:
                meshes[path] = snapshot_raster.read_stl(path)
            return {"triangles": meshes[path] * gazebo_common.mesh_scale_factor(shape.findtext("scale"), warnings)}
        if shape.tag == "box":
            size = [float(v) * MM_PER_M for v in (shape.findtext("size") or "1 1 1").split()]
            return {"triangles": snapshot_raster.box([v / 2.0 for v in size])}
        if shape.tag == "sphere":
            radius = number("radius", 1) * MM_PER_M
            return {"triangles": snapshot_raster.ellipsoid(radius, radius, radius)}
        if shape.tag == "ellipsoid":
            radii = [float(v) * MM_PER_M for v in (shape.findtext("radii") or "1 1 1").split()]
            return {"triangles": snapshot_raster.ellipsoid(*radii)}
        if shape.tag == "cylinder":
            return {
                "triangles": snapshot_raster.cylinder(
                    number("radius", 1) * MM_PER_M, number("length", 1) * MM_PER_M / 2.0
                )
            }
        if shape.tag == "capsule":
            return {
                "triangles": snapshot_raster.capsule(
                    number("radius", 1) * MM_PER_M, number("length", 1) * MM_PER_M / 2.0
                )
            }
        if shape.tag == "plane":
            normal = np.array([float(v) for v in (shape.findtext("normal") or "0 0 1").split()])
            return {"plane": normal / np.linalg.norm(normal)}
        return None

    def read_link(element):
        visuals = []
        for visual in element.findall("visual"):
            drawn = geometry(visual)
            if drawn is not None:
                drawn["pose"] = gazebo_common.parse_pose(visual.find("pose"))
                drawn["color"] = color(visual)
                visuals.append(drawn)
        return {
            "name": element.get("name"),
            "pose": gazebo_common.parse_pose(element.find("pose")),
            "visuals": visuals,
        }

    def read_model(element):
        return {
            "name": element.get("name"),
            "pose": gazebo_common.parse_pose(element.find("pose")),
            "links": [read_link(link) for link in element.findall("link")],
            "models": [read_model(model) for model in element.findall("model")],
        }

    return [read_model(model) for model in world.findall("model")]


def arrange(layout, poses):
    """The world at one instant, in the terms 'snapshot_raster' draws.

    Every pose is composed down the tree, each one taken from the reading where
    the reading names it and from the world file where it does not -- a static
    model, a name two entities share. A link is one thing, and gets an outline
    of its own.
    """
    import numpy as np

    import snapshot_raster
    import urdf_common

    solids, planes = [], []

    def matrix(q):
        w, x, y, z = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    def place(models, parent):
        for model in models:
            frame = urdf_common.compose(parent, poses.get(model["name"], model["pose"]))
            for link in model["links"]:
                link_frame = urdf_common.compose(frame, poses.get(link["name"], link["pose"]))
                ident = len(solids) + 1
                for visual in link["visuals"]:
                    q, t = urdf_common.compose(link_frame, visual["pose"])
                    rotation = matrix(urdf_common.normalize(q))
                    if "plane" in visual:
                        normal = visual["plane"]
                        helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
                        x = np.cross(helper, normal)
                        x /= np.linalg.norm(x)
                        axes = rotation @ np.column_stack([x, np.cross(normal, x), normal])
                        planes.append({"origin": np.asarray(t), "axes": axes, "color": visual["color"]})
                        continue
                    solids.append(
                        {
                            "triangles": snapshot_raster.place(visual["triangles"], rotation, t),
                            "color": visual["color"],
                            "id": ident,
                        }
                    )
            place(model["models"], frame)

    place(layout, (urdf_common.IDENTITY_Q, urdf_common.IDENTITY_T))
    return {"solids": solids, "planes": planes}


def take_snapshots(path, snapshot, scene_file, first, last, warnings):
    """Draw the pictures PartCAD asked for, and say which were drawn.

    Never a reason for the run to fail: the verdict does not depend on a
    picture, so a picture that cannot be drawn is a warning beside a result
    rather than the loss of one.
    """
    if not snapshot or not path or first is None or last is None:
        return {}
    try:
        import snapshot_raster

        layout = world_layout(scene_file, warnings)
        scenes = {"before": arrange(layout, unique_poses(first)), "after": arrange(layout, unique_poses(last))}
        return snapshot_raster.take(path, snapshot, scenes)
    except Exception as e:  # pylint: disable=broad-except
        warnings.append("the snapshots could not be drawn: %s: %s" % (type(e).__name__, e))
        return {}


def messages(stream):
    """Every complete message on a 'gz topic -e' stream, as it arrives.

    A message is everything from one top-level 'header {' to the next, so the
    one being accumulated is only yielded once the next one starts. That is
    exactly right for what this is used for -- the reading that matters is
    whichever one came before the clock ran out.
    """
    current = []
    for line in stream:
        if line.startswith("header {") and current:
            yield "".join(current)
            current = []
        current.append(line)
    if current:
        yield "".join(current)


#
# Reading the joints
#
# Gazebo states where a model's joints are only for a model that carries its
# JointStatePublisher system, which publishes a 'gz.msgs.Model' every step: the
# model, then one 'joint' per joint, each with an 'axis1' holding its 'position'
# and 'velocity' in radians or metres. A world does not carry it -- PartCAD's
# exporter writes none, and a world somebody else wrote has no reason to -- so
# the world that runs is a copy of the one handed over, with the system added to
# every model that has a joint to read.
#
# Added here rather than by the exporter, for three reasons. What a run needs to
# read is the reader's business: a world written for `pc open --with gazebo`, or
# for anybody else's tooling, should not carry a system only this script listens
# to. It works on whatever world this is handed, whoever wrote it. And Gazebo's
# own defaults survive it: the systems a world runs with when it names none --
# the physics, and the scene broadcaster the poses come from -- are loaded
# unless the world attaches a system *to the world*, and this one is attached to
# a model. (Checked against Harmonic, which says "No systems loaded from SDF,
# loading defaults" with the publisher on every model.)
#
# What the messages do not say is a joint's type -- Harmonic's publisher leaves
# it out -- so that is read out of the world too, while the copy is made.
#

JOINT_STATE_PUBLISHER = {
    "filename": "gz-sim-joint-state-publisher-system",
    "name": "gz::sim::systems::JointStatePublisher",
}

# The SDFormat joint types that are read, as the 'motion:' type PartCAD calls
# them -- which is the same name for each of them, but for a 'revolute' nothing
# limits ('motion_type'). A screw is read as its turn, which is the coordinate
# Gazebo gives it; the move along its axis follows from the thread.
TURNS = ("revolute", "continuous", "screw")
MOVES = ("prismatic",)

# The joint types that are not, and why. Each such joint is named in the
# result's 'warnings' instead: something may have moved there, and a reading
# that quietly leaves it out is worse than one that says it did.
NOT_READ = {
    "ball": "Gazebo publishes two of a ball joint's three coordinates, which is not an orientation",
    "universal": "it turns about two axes, and PartCAD has no motion of that type",
    "revolute2": "it turns about two axes, and PartCAD has no motion of that type",
    "gearbox": "PartCAD has no motion of that type",
}

# SDFormat's own default for a joint limit nobody stated, and so what "no
# limit" looks like once a world has been through a tool that writes them out.
NO_LIMIT = 1e16

# How long to wait, in wall-clock seconds, for a model's joint states to reach
# the instant a pose message was taken at. Both come out of the same step of the
# same server, so they arrive within moments of each other and this is only
# ever spent on a stream that is not coming at all -- once, for a stream that
# has said nothing (see 'JointStream.at').
PATIENCE = 10.0


def note(warnings, message):
    """Add a warning once, however many readings run into it."""
    if message not in warnings:
        warnings.append(message)


class JointModel:
    """One model whose joints are read: what it is called, where they arrive, what they are."""

    def __init__(self, scoped, topic, joints):
        # 'outer::inner' for a nested model, which is how Gazebo scopes one.
        self.scoped = scoped
        # The topic its publisher was told to publish on.
        self.topic = topic
        # {name in the world: (motion type, name reported under, SDFormat type)}
        self.joints = joints


def motion_type(joint, declared):
    """The 'motion:' type of one SDFormat joint of a type that is read.

    Its own type, except that a 'revolute' is one only while something limits
    it: without a limit it is 'continuous', which is the line PartCAD (and
    URDF) draws, and the one the MuJoCo plugin reads a hinge by. So a validation
    reads the same type out of either engine for the same joint, however each
    file had to spell it -- which matters, because Gazebo cannot run one of the
    two spellings (see 'joint_states').
    """
    if declared != "revolute":
        return declared
    for bound in ("axis/limit/lower", "axis/limit/upper"):
        text = joint.findtext(bound)
        try:
            if text is not None and abs(float(text)) < NO_LIMIT:
                return "revolute"
        except ValueError:
            continue
    return "continuous"


def plan_joints(world, prefix, warnings):
    """Add a JointStatePublisher to every model of 'world' that has a joint to read.

    Returns a 'JointModel' for each, in document order, the publisher of the
    n-th told to publish on '<prefix>/<n>' -- a topic of this run's own rather
    than Gazebo's default, so that nothing has to guess how Gazebo names the
    topic of a nested model, and so that two runs on one machine do not read
    each other's joints.

    A fixed joint is not read: nothing moves there. Nor is a joint of a static
    model, or of a model nested in one: the world says nothing of it moves, so
    there is nothing to say. A joint of a type that is not read is named in
    'warnings'.

    A joint is reported under its own name where no other joint read has it,
    and as '<model>::<joint>' where one does -- for every one of them, so that
    neither is the joint the plain name means. Names are only unique within a
    model, and a world with two of the same robot in it has two of every joint.
    """
    found = []

    def visit(model, scope, static):
        scoped = scope + [model.get("name") or "model"]
        static = static or (model.findtext("static") or "").strip().lower() in ("true", "1")
        if not static:
            read = {}
            for joint in model.findall("joint"):
                name = joint.get("name")
                declared = (joint.get("type") or "").strip().lower()
                if not name or declared == "fixed":
                    continue
                if declared in TURNS or declared in MOVES:
                    read[name] = (motion_type(joint, declared), declared)
                else:
                    note(
                        warnings,
                        "joint '%s' of model '%s' is not reported: it is a '%s' joint, and %s"
                        % (
                            name,
                            "::".join(scoped),
                            declared,
                            NOT_READ.get(declared, "this plugin does not know the type"),
                        ),
                    )
            if read:
                found.append((model, "::".join(scoped), read))
        for nested in model.findall("model"):
            visit(nested, scoped, static)

    for model in world.findall("model"):
        visit(model, [], False)

    counts = collections.Counter(name for _model, _scoped, read in found for name in read)
    models = []
    for index, (element, scoped, read) in enumerate(found):
        topic = "%s/%d" % (prefix, index)
        plugin = ElementTree.SubElement(element, "plugin", dict(JOINT_STATE_PUBLISHER))
        ElementTree.SubElement(plugin, "topic").text = topic
        joints = {
            name: (kind, name if counts[name] == 1 else "%s::%s" % (scoped, name), declared)
            for name, (kind, declared) in read.items()
        }
        models.append(JointModel(scoped, topic, joints))
    return models


def prepare_world(scene_file, warnings):
    """The world to run, and the models whose joints are read in it.

    The world as it was handed over when nothing in it has a joint to read -- so
    a world of free models, which is every world PartCAD exports today, runs
    exactly as it always has -- and otherwise a copy of it with the publishers
    added ('plan_joints'), which 'process' removes again when the run is over.
    The copy is written beside the world rather than anywhere else, because a
    world names its meshes relative to where it is.

    Joints of a model the world brings in with '<include>' are not read: they
    are in another file. PartCAD's exporter writes everything into the one.
    """
    try:
        tree = ElementTree.parse(scene_file)
    except ElementTree.ParseError:
        # Not this function's to report: the server reads it next, and says
        # what is wrong with it the way it always has.
        return scene_file, []
    root = tree.getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        return scene_file, []
    models = plan_joints(world, "/partcad/%d/joint_state" % os.getpid(), warnings)
    if not models:
        return scene_file, []
    stem, extension = os.path.splitext(scene_file)
    copy = "%s.joint-states%s" % (stem, extension or ".world")
    try:
        tree.write(copy, encoding="utf-8", xml_declaration=True)
    except OSError as e:
        note(warnings, "the joints are not reported: the world could not be copied to add what publishes them: %s" % e)
        return scene_file, []
    return copy, models


_STAMP = re.compile(r"header \{\s*stamp \{([^}]*)\}")
_SECONDS = re.compile(r"\bsec: (\d+)")
_NANOSECONDS = re.compile(r"\bnsec: (\d+)")


def message_time(text):
    """The simulated time a message was published at, read off its header alone.

    A joint-state message arrives every step and nearly all of them are never
    read, so only the header of each is: the rest is parsed if a reading asks
    for that one. The same sum 'snapshot' makes of the same two fields, so that
    the two clocks compare equal at the same instant.
    """
    match = _STAMP.match(text)
    if match is None:
        return 0.0
    seconds = _SECONDS.search(match.group(1))
    nanoseconds = _NANOSECONDS.search(match.group(1))
    return float(seconds.group(1) if seconds else 0) + float(nanoseconds.group(1) if nanoseconds else 0) / 1e9


class JointStream:
    """The joint states one model publishes, kept for as long as a reading may ask for them.

    Read on a thread of its own ('follow'), because they arrive on a stream of
    their own -- a message every step, where the poses arrive a few dozen times
    a second -- and a stream nobody reads stops the program writing it. Each
    message is kept as text with its time, and parsed only if a reading asks for
    it; 'forget_before' drops what no reading can ask for any more, which keeps
    what is held to the few moments one stream runs ahead of the other.
    """

    def __init__(self, model):
        self.model = model
        self.entries = collections.deque()
        self.newest = None
        self.ended = False
        self.silent = False
        self.condition = threading.Condition()

    def follow(self, stream):
        """Keep every message 'stream' carries, until it ends."""
        try:
            for text in messages(stream):
                stamp = message_time(text)
                with self.condition:
                    self.entries.append((stamp, text))
                    self.newest = stamp
                    self.condition.notify_all()
        except (OSError, ValueError):
            # Closed under it, which is one of the ways a run ends.
            pass
        finally:
            with self.condition:
                self.ended = True
                self.condition.notify_all()

    def at(self, moment, patience=PATIENCE):
        """The message taken at 'moment', as text, or None if there is none at all.

        Waits for the stream to get that far first, since it runs alongside the
        poses rather than in step with them -- for up to 'patience' seconds,
        and only once for a stream that has said nothing in that time, which is
        a publisher that is not there rather than one that is slow.

        The message is the latest one taken at or before 'moment': the one of
        that very step, unless the subscriber missed it. Failing that -- a
        stream that only started after it -- the first one there is.
        """
        with self.condition:
            deadline = time.monotonic() + patience
            while not self.ended and not self.silent and (self.newest is None or self.newest < moment):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.silent = self.newest is None
                    break
                self.condition.wait(remaining)
            chosen = None
            for stamp, text in self.entries:
                if stamp > moment:
                    if chosen is None:
                        chosen = text
                    break
                chosen = text
            return chosen

    def forget_before(self, moment):
        """Drop every message older than the latest one taken at or before 'moment'.

        A reading is never asked for at an instant earlier than a pose message
        already seen, so that one is the earliest any reading can still want.
        """
        with self.condition:
            while len(self.entries) > 1 and self.entries[1][0] <= moment:
                self.entries.popleft()


def joint_states(message, model, warnings):
    """The joints of one model in one of its joint-state messages, in PartCAD's terms.

    The terms of the 'motion:' an interface declares, and the same vocabulary
    the MuJoCo plugin reports in:

      revolute, continuous, screw  'pos' in degrees, 'vel' in degrees per second
      prismatic                    'pos' in millimetres, 'vel' in mm/s

    'pos' is the joint's own coordinate, zero where the world placed the child
    link: a pendulum written out horizontal reads 0 there and 90, one way or
    the other, hanging down.

    There is no 'effort', which the MuJoCo plugin does report. The message has a
    field for it and Harmonic leaves it empty -- it stays empty while a force is
    being applied to the joint -- and a zero nobody measured would be read as a
    measurement.

    A joint that should be here and is not, or that Gazebo ran with no freedom
    to move, is named in 'warnings' rather than reported as standing still.
    """
    states = {}
    published = set()
    for joint in _as_list(message.get("joint")):
        name = joint.get("name")
        if name not in model.joints:
            continue
        published.add(name)
        kind, reported, declared = model.joints[name]
        axis = joint.get("axis1")
        if not isinstance(axis, dict):
            # No coordinate at all: Gazebo built it without a degree of
            # freedom. Which is what Harmonic's DART does with every
            # 'continuous' joint, saying so only in the server's own log.
            note(
                warnings,
                "joint '%s' of model '%s' is not reported: Gazebo ran it with no freedom to move%s"
                % (
                    name,
                    model.scoped,
                    (
                        " (its DART physics builds a 'continuous' joint as a fixed one; a 'revolute'"
                        " with no limits is the same joint, and moves)"
                        if declared == "continuous"
                        else ""
                    ),
                ),
            )
            continue
        scale = DEG_PER_RAD if kind in TURNS else MM_PER_M
        states[reported] = {
            "type": kind,
            "pos": float(axis.get("position") or 0.0) * scale,
            "vel": float(axis.get("velocity") or 0.0) * scale,
        }
    for name in model.joints:
        if name not in published:
            note(
                warnings,
                "joint '%s' of model '%s' is not reported: it is not among the joint states Gazebo published"
                % (name, model.scoped),
            )
    return states


def fill(reading, streams, warnings, patience=PATIENCE):
    """State in 'reading' where the joints of every stream were at its instant.

    'patience' is how long to wait for a stream that has not got that far yet
    (see 'JointStream.at'); nothing is worth waiting for once the server that
    publishes them has gone.
    """
    for stream in streams:
        text = stream.at(reading["time"], patience)
        if text is None:
            note(
                warnings,
                "the joints of model '%s' are not reported: Gazebo published no joint states for it"
                % stream.model.scoped,
            )
            continue
        reading["joints"].update(joint_states(parse_message(text), stream.model, warnings))


def process(path, request):
    (server, server_args, topic_command), environment = locate_gazebo()

    scene_file = request["scene_file"]
    duration = float(request.get("duration") or 10.0)
    samples = int(request.get("samples") or 0)
    timeout = float(request.get("timeout") or 300.0)
    world = request.get("world_name") or world_name(scene_file)
    topic = "/world/%s/pose/info" % world
    warnings = []
    # The world as handed over, or a copy of it that publishes its joints.
    run_file, joint_models = prepare_world(scene_file, warnings)

    command = [server] + list(server_args) + ["-s", "-r", "-v", "1"]
    timestep = request.get("timestep")
    if timestep:
        command += ["-z", str(1.0 / float(timestep))]
    command.append(run_file)

    # The subscriber first. It is the thing that has to be listening before the
    # server starts publishing; the other way round loses the opening reading,
    # which is the one 'before' is.
    watcher = subprocess.Popen(
        [server, topic_command, "-e", "-t", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
        env=environment,
    )
    simulator = None
    # The first message and the last, kept whole for the snapshots: a reading
    # keyed by name is enough for a validation and not for a picture, which
    # has to know which link of which model each pose belongs to.
    first = last = None
    joint_watchers = []
    streams = []
    try:
        # A subscriber for the joints of each model that has any, listening
        # before the server starts for the reason the one above is.
        for model in joint_models:
            joint_watchers.append(
                subprocess.Popen(
                    [server, topic_command, "-e", "-t", model.topic],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                    env=environment,
                )
            )
            streams.append(JointStream(model))
            threading.Thread(target=streams[-1].follow, args=(joint_watchers[-1].stdout,), daemon=True).start()

        simulator = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, env=environment
        )

        before = None
        after = None
        trace = []
        every = duration / (samples + 1) if samples > 0 else None
        next_sample = every
        deadline = time.time() + timeout

        for text in messages(watcher.stdout):
            message = parse_message(text)
            reading = snapshot(message)
            if not reading["bodies"]:
                continue
            if before is None:
                before = reading
                first = message
                fill(before, streams, warnings)
            after = reading
            last = message
            for stream in streams:
                stream.forget_before(reading["time"])
            if next_sample is not None and reading["time"] >= next_sample:
                fill(reading, streams, warnings)
                trace.append(reading)
                next_sample += every
            if reading["time"] >= duration:
                break
            if time.time() > deadline:
                raise Exception(
                    "Gazebo did not reach %.3gs of simulated time within %.0fs of waiting "
                    "(it reached %.3gs). Raise 'timeout' if this machine is simply slow."
                    % (duration, timeout, reading["time"])
                )
            if simulator.poll() is not None:
                break

        if before is None:
            stderr = ""
            if simulator is not None and simulator.poll() is not None and simulator.stderr is not None:
                stderr = simulator.stderr.read() or ""
            raise Exception(
                "Gazebo published no poses for world '%s', so there is nothing to report. "
                "%s%s" % (world, "The server said: " if stderr.strip() else "", stderr.strip())
            )
        # Here rather than as the loop takes it, because which reading is the
        # last is only known once the loop is over -- and here rather than
        # after it, because the server is still up and still publishing, and
        # the joint states of that instant may still be on their way. Unless it
        # is not, in which case what has arrived is all there will be.
        fill(after, streams, warnings, PATIENCE if simulator.poll() is None else 0.0)
    finally:
        for process_handle in [simulator, watcher] + joint_watchers:
            if process_handle is None or process_handle.poll() is not None:
                continue
            process_handle.terminate()
            try:
                process_handle.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process_handle.kill()
        if run_file != scene_file:
            try:
                os.remove(run_file)
            except OSError:
                pass

    result = {
        "success": True,
        "before": before,
        "after": after,
        # Beside the two PartCAD requires: what this run actually was, so that a
        # report of a failed validation says what it was a validation of.
        "simulator": "gazebo",
        "version": _version([server] + list(server_args), environment),
        "world": world,
        "duration": duration,
        "reached": after["time"],
        "gravity": request.get("gravity"),
        "units": "mm",
    }
    if trace:
        result["samples"] = trace
    drawn = take_snapshots(path, request.get("snapshot"), scene_file, first, last, warnings)
    if drawn:
        result["snapshots"] = drawn
    if warnings:
        result["warnings"] = warnings
    return result


def _version(command, environment=None):
    """What this Gazebo calls itself, for the record. Never a reason to fail.

    Asked of the server subcommand ('gz sim --versions'): 'gz --versions' on its
    own answers with the tool's usage text rather than a version.
    """
    try:
        answer = subprocess.run(
            list(command) + ["--versions"], capture_output=True, text=True, timeout=30, env=environment
        )
        printed = (answer.stdout or answer.stderr or "").strip().splitlines()
        return printed[0].strip() if printed else None
    except Exception:  # pylint: disable=broad-except
        return None


if __name__ == "__main__":  # pragma: no cover
    # Not how PartCAD runs this -- it imports the file and calls 'process' --
    # but running it by hand against a world file is how one finds out whether
    # this machine's Gazebo behaves the way the code above expects.
    import json

    print(json.dumps(process(None, {"scene_file": sys.argv[1], "duration": float(sys.argv[2] if len(sys.argv) > 2 else 10)}), indent=2))
