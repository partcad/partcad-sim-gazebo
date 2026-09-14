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

**Why the clock is read out of the messages.** ``gz sim -r`` runs at a real-time
factor of about one, so waiting ten wall-clock seconds is *nearly* ten seconds of
simulation -- and "nearly" is not something a validation should depend on. Every
pose message carries the simulated time it was taken at, so this waits for that
to pass ``duration`` instead, and uses wall-clock only as the timeout that stops
a run which is not progressing at all.
"""

import os
import re
import shutil
import subprocess
import sys
import time

# Millimetres per metre: SDFormat is metres by definition, PartCAD is
# millimetres throughout. Spelled out rather than imported from PartCAD's own
# 'urdf_common' because this script runs in a sandbox that may carry nothing but
# Gazebo -- and because a number this stable is not worth a dependency.
MM_PER_M = 1000.0

# The programs that are a Gazebo, newest first, and what each one calls the
# subcommand that runs a server. The same table `open:` in 'partcad.yaml'
# declares for `pc open`, for the same reason: one answer to "is there a Gazebo
# on this machine", whichever generation of it is installed.
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


def find_gazebo():
    """The Gazebo on this machine, as (server binary, server args, topic binary).

    'PC_GZ' overrides the search, for a machine with an installation the name
    does not give away. It names the *server* binary; the topic tool is assumed
    to sit beside it under the matching name, which is how every Gazebo release
    has ever laid itself out.
    """
    override = os.environ.get("PC_GZ")
    if override:
        directory = os.path.dirname(override)
        name = os.path.basename(override)
        # The table entry for whichever generation it is, and the current one
        # for a name this table has never heard of: an override is an override,
        # and refusing it because the binary has an unfamiliar name would defeat
        # the point of having one.
        _, args, topic = next((entry for entry in SERVERS if entry[0] == name), SERVERS[0])
        return override, args, os.path.join(directory, topic) if directory else topic

    for binary, args, topic in SERVERS:
        found = shutil.which(binary)
        if found:
            return found, args, topic
    raise GazeboMissing(NO_GAZEBO)


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


def process(path, request):  # pylint: disable=unused-argument
    server, server_args, topic_tool = find_gazebo()

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
        [topic_tool, "-e", "-t", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    simulator = None
    try:
        simulator = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)

        before = None
        after = None
        trace = []
        every = duration / (samples + 1) if samples > 0 else None
        next_sample = every
        deadline = time.time() + timeout

        for text in messages(watcher.stdout):
            reading = snapshot(text)
            if not reading["bodies"]:
                continue
            if before is None:
                before = reading
            after = reading
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
        "version": _version(server),
        "world": world,
        "duration": duration,
        "reached": after["time"],
        "gravity": request.get("gravity"),
        "units": "mm",
    }
    if trace:
        result["samples"] = trace
    return result


def _version(server):
    """What this Gazebo calls itself, for the record. Never a reason to fail."""
    try:
        answer = subprocess.run([server, "--versions"], capture_output=True, text=True, timeout=30)
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
