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
the same two ways ``pc ide open --with gazebo`` finds it -- the ``gz`` on this
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
"""

import glob
import os
import re
import shutil
import subprocess
import sys
import time

# 'snapshot_raster' and 'gazebo_common' are this package's, beside this file.
# PartCAD runs this script by path, which puts nothing on sys.path for it.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Millimetres per metre: SDFormat is metres by definition, PartCAD is
# millimetres throughout. Spelled out rather than imported from PartCAD's own
# 'urdf_common' because this script runs in a sandbox that may carry nothing but
# Gazebo -- and because a number this stable is not worth a dependency.
MM_PER_M = 1000.0

# The programs that are a Gazebo, newest first, what each one calls the
# subcommand that runs a server, and what it calls the one that prints a topic.
# Both are subcommands of the one program -- 'gz sim', 'gz topic' -- not programs
# of their own. The same table `open:` in 'partcad.yaml' declares for `pc ide open`,
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
    return {"time": seconds, "bodies": bodies}


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


def process(path, request):
    (server, server_args, topic_command), environment = locate_gazebo()

    scene_file = request["scene_file"]
    duration = float(request.get("duration") or 10.0)
    samples = int(request.get("samples") or 0)
    timeout = float(request.get("timeout") or 300.0)
    world = request.get("world_name") or world_name(scene_file)
    topic = "/world/%s/pose/info" % world

    command = [server] + list(server_args) + ["-s", "-r", "-v", "1"]
    timestep = request.get("timestep")
    if timestep:
        command += ["-z", str(1.0 / float(timestep))]
    command.append(scene_file)

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
    try:
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
            after = reading
            last = message
            if next_sample is not None and reading["time"] >= next_sample:
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
    finally:
        for process_handle in (simulator, watcher):
            if process_handle is None or process_handle.poll() is not None:
                continue
            process_handle.terminate()
            try:
                process_handle.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process_handle.kill()

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
    warnings = []
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
