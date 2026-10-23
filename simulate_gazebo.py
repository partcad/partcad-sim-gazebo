#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""The Gazebo simulation plugin (see 'partcad.yaml' beside this file).

Runs a world under gravity for a while and says where everything ended up.

The world arrives as SDFormat -- PartCAD exported it with this package's own
exporter, with every free body free to move and the joints of every mechanism in
it -- so all this does is start the server, watch the poses it publishes, and
take the same reading twice: once before anything has moved and once when the
simulated clock reaches ``duration``. That pair is what a ``simulate:``'s
``validation:`` expression is handed, and it is the whole of what PartCAD
requires a simulation plugin to produce.

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

**Why the clock is read out of the messages.** Gazebo runs at a real-time
factor of about one, so waiting ten wall-clock seconds is *nearly* ten seconds of
simulation -- and "nearly" is not something a validation should depend on. Every
pose message carries the simulated time it was taken at, so this waits for that
to pass ``duration`` instead, and uses wall-clock only as the timeout that stops
a run which is not progressing at all.

**The first reading is the world as it was loaded.** The server is started
paused, ``before`` is read at t = 0 -- every joint at its start, every body where
the scene put it, every speed nought -- and only then is the world set running,
by its own control service ('unpause'). The step is the world's
``<max_step_size>`` and ``timestep`` writes it ('world_physics'); a run that asks
for more ``samples`` than the sixty pose messages a wall-clock second Gazebo
publishes runs slower than real time to get them ('pace').

**Joints.** A world whose models have joints in them -- a hinge, a slide -- is
also read for where each joint is and how fast it moves, as ``joints`` beside
``bodies``, in the same vocabulary the MuJoCo plugin states them in: one entry
per PartCAD joint, in PartCAD's terms -- degrees for a turn, millimetres for a
move, counted from PartCAD's zero although the world counts every joint from
its start -- made by PartCAD's own 'kinematics.reading' out of the joint map
the exporter wrote into the sidecar. Gazebo publishes joint states only for a
model that carries its JointStatePublisher system, so the world that runs is a
copy of the one handed over with that system added; see 'plan_joints' and
'JointStream'. A ball and a floating child are read from where their bodies
are instead; see 'posed_states'.

**What holds each body, and how hard.** Every body in a reading also says what
acts on it, as ``loads`` beside its pose, in the vocabulary the MuJoCo plugin
states them in: its weight, out of the world's '<inertial>', and what each of
its joints passes it, which Gazebo measures only with a force-torque sensor on
the joint -- so the world that runs carries one on every joint between two
bodies, and the system that reads them; see 'plan_wrenches' and 'fill_loads'.
What Gazebo does not give -- a weld to the world, contacts, accelerations, and
anything at the first reading, taken while the world is paused -- is named in
'warnings' rather than reported as nought.

**Names are PartCAD's, and poses the world's.** A validation reads
``after["bodies"]["arm"]`` and ``after["joints"]["swing"]`` by the names PartCAD
gave them, and a world cannot hold every one of those as it is: SDFormat
reserves 'world', which is what PartCAD's default world is called. The exporter
says which name each became, and which model and link each body is, in a
sidecar beside the world (PartCAD's 'wrappers/naming.py'), and everything here
is reported under the PartCAD name it reads there -- bodies, joints, the world
and the warnings; see 'Names'. Each body's pose is its link's in the *world's*
frame -- its model's pose composed with its link's, which is how Gazebo
publishes them -- so that a link hanging off a mechanism's root reads where it
is, as the MuJoCo plugin reads it, and not where it is in its model. A world
with no sidecar, which PartCAD did not write, is reported as Gazebo publishes
it: every entity under its own name, relative to its parent.

**A partition of its own.** Every run talks to its server in a Gazebo transport
partition nobody else is in, so that two runs on one machine -- two worlds of
the same name, which is every pair of runs of one simulation -- do not read
each other's poses and joints; see 'run_environment'.

**The systems a world runs with.** A world PartCAD exports names no systems
of its own, so the server runs it with the ones its server configuration
lists -- the physics, and the scene broadcaster every pose comes from. Gazebo
keeps that in the home, copying it there the first time a server starts, and
a server started beside the one copying it can read it empty and run with no
systems: alive, paused, and publishing nothing. So a server is pointed at the
installed configuration wherever the home has no copy yet; see
'server_config'.

**Nothing is waited for blindly.** A server that dies says nothing on the topic
its subscriber is waiting on, and neither does a server that hangs while it
starts, or one publishing in a partition nobody is listening in -- the
subscriber just goes on waiting, and so did this script, for ever. So what the
server publishes is read on a thread of its own, and the run is checked while it
waits: a server that has exited fails the run at once, with its exit status and
the end of what it wrote (see 'Programs'); a server that says it loaded no
systems fails it at once too ('no_systems'); poses that do not come to a
running server's subscriber are subscribed to again ('RESUBSCRIBE'); a first
reading that does not come within ``startup_timeout`` fails it saying where it
was listening, how many poses came, and what each joint stream said
('no_first_reading'); and every wait -- for poses, for joint states, for the
version -- is inside the run's budget, ``timeout``, which PartCAD hands every
run and stops it at. This script stops a little before that, so that what it
reports says how far the run got.
Every program it starts is in a process group of its own and is stopped with
everything it started on every way out, and the world copy with it.
"""

import collections
import glob
import math
import os
import queue
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
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


# The variables gz-transport takes its partition from: 'GZ_PARTITION' since
# Garden, 'IGN_PARTITION' in the Ignition years. Both are set, to one name, so
# that whichever generation 'find_gazebo' found reads it. Harmonic ignores the
# 'IGN_' one without a word about it, which was checked rather than assumed.
PARTITION_VARIABLES = ("GZ_PARTITION", "IGN_PARTITION")


def run_environment(environment=None, token=None):
    """The environment to run the server and every subscriber in, and its partition.

    Gazebo's transport finds the other end of a topic by asking the network,
    and answers come from every process in the same *partition* -- which, unless
    something says otherwise, is one per user per machine. A topic is named
    after the world ('/world/<name>/pose/info'), and the world after the scene,
    so two runs of one simulation at once -- 'pc test' running the simulations
    of a package side by side, or a 'pc sim' beside it -- are two servers
    publishing on the same topic, and each run's subscriber hears both. Nothing
    fails: the readings interleave, and a validation passes or fails on poses
    that were never its own.

    So every run gets a partition of its own, and the server and every client
    that reads from it are started in it. Named by a random token, not by
    anything about the run: the run directory is one per object and simulation
    and so is shared by exactly the two runs that collide, and a process id is
    unique on one machine but not across sandbox containers, which have one
    numbering each and may well share a network.

    A partition the user has already set ('GZ_PARTITION', or 'IGN_PARTITION'
    for the generation that reads that one) is kept as a prefix rather than
    used as it is: '<theirs>:partcad-<token>'. Taken as it is, it would put
    every run back into one partition -- the bug this exists to prevent, back
    for exactly the users careful enough to have set one, who set it in a
    shell profile to keep their own traffic apart rather than to share it with
    each run. Nested, their name is still the first thing in it, and the run is
    still apart from everything, theirs included. What that costs is watching
    a run live from a client in their partition, which nothing offers today.

    'environment' is the one to start from, None for this process's own; it is
    copied, never changed. 'token' is for tests.
    """
    result = dict(os.environ if environment is None else environment)
    own = "partcad-%s" % (token or uuid.uuid4().hex[:16])
    theirs = next((result[name] for name in PARTITION_VARIABLES if result.get(name)), None)
    partition = "%s:%s" % (theirs, own) if theirs else own
    for name in PARTITION_VARIABLES:
        result[name] = partition
    return result, partition


# The variable a Gazebo since Garden reads its server configuration's path
# from: the systems a world that names none of its own is run with.
SERVER_CONFIG = "GZ_SIM_SERVER_CONFIG_PATH"

# Where an installation keeps the server configuration it was built with: beside
# the command files Gazebo's tool reads, one directory per release of the
# simulator -- 'gz-sim8' is Harmonic's.
INSTALLED_SERVER_CONFIG = os.path.join("gz-sim*", "server.config")

# Where the tool looks for those when nothing says otherwise, after the
# installation the server binary is in.
SYSTEM_SHARES = ("/usr/share/gz", "/usr/local/share/gz")

# What a server writes when the server configuration it read is no
# configuration at all, and it is left with no systems (see 'server_config').
NO_SYSTEMS = "No <server_config> element found when parsing plugins"


def _release(path):
    """The release a 'gz-sim<N>/server.config' is of, as a number to sort by."""
    digits = re.search(r"(\d+)$", os.path.basename(os.path.dirname(path)))
    return int(digits.group(1)) if digits else -1


def installed_server_config(environment=None, server=None):
    """The server configuration this Gazebo was installed with, or None.

    Looked for in every directory 'GZ_CONFIG_PATH' names -- which is how a ROS
    installation's setup script points Gazebo's tool at its vendor packages,
    and how a workspace built from source does -- then in the 'share/gz' of
    the installation the server binary is in, and in the system's own. The
    newest release in the first that has any: the one 'gz sim' runs.
    """
    environment = os.environ if environment is None else environment
    roots = [root for root in (environment.get("GZ_CONFIG_PATH") or "").split(os.pathsep) if root]
    if server:
        prefix = os.path.dirname(os.path.dirname(os.path.realpath(server)))
        roots.append(os.path.join(prefix, "share", "gz"))
    for root in roots + list(SYSTEM_SHARES):
        found = [path for path in glob.glob(os.path.join(root, INSTALLED_SERVER_CONFIG)) if os.path.isfile(path)]
        if found:
            return max(found, key=_release)
    return None


def home_server_config(environment, installed):
    """The copy of 'installed' a server reads by default: '~/.gz/sim/<release>/server.config'."""
    home = environment.get("HOME") or environment.get("USERPROFILE") or os.path.expanduser("~")
    return os.path.join(home, ".gz", "sim", str(_release(installed)), os.path.basename(installed))


def is_server_config(path):
    """Whether 'path' is a server configuration at all: XML whose root is '<server_config>'."""
    try:
        return ElementTree.parse(path).getroot().tag == "server_config"
    except (OSError, ElementTree.ParseError):
        return False


def server_config(environment, server, warnings):
    """The server configuration to start the server with, or None to leave the choice to Gazebo.

    A world that names no systems of its own -- which is every world PartCAD
    exports -- is run with the ones its server's configuration lists: the
    physics, and the scene broadcaster every pose comes from. Unless
    'GZ_SIM_SERVER_CONFIG_PATH' says where it is, a server reads it from
    '~/.gz/sim/<release>/server.config', and the first server started in a
    home where that is not there yet copies the installed one into it -- in
    place, as 'std::filesystem::copy_file' does: the file is created, empty,
    then filled. A second server started at the same moment finds it there,
    reads nothing, writes "No <server_config> element found when parsing
    plugins", and runs with no systems at all: paused, alive, and publishing
    no pose, ever. A 'pc test' starts a package's simulations together, and
    in a fresh home -- every CI job's -- that is how one or two of the first
    of them hung for the whole of their 'startup_timeout'.

    So where that copy is not there, the server is pointed at the installed
    configuration itself, which it reads where it is: what Gazebo would have
    copied, without the copy, and without writing one into the home either.
    A copy that is there is a configuration somebody may have changed, and is
    read as it always was -- unless it is no configuration at all, empty or
    unreadable, which is what this race leaves behind when a copy is cut
    short. That one is passed over for the installed one and named in
    'warnings'. A configuration the environment already names is that
    environment's choice and is kept, and an installation whose configuration
    cannot be found is left to Gazebo; 'NO_SYSTEMS' is what says so if that
    goes wrong (see 'no_systems').
    """
    if environment.get(SERVER_CONFIG):
        return None
    installed = installed_server_config(environment, server)
    if installed is None:
        return None
    own = home_server_config(environment, installed)
    if not os.path.exists(own):
        return installed
    if is_server_config(own):
        return None
    note(
        warnings,
        "'%s' is not a Gazebo server configuration -- it is empty or unreadable, which is what a server copying it"
        " there leaves when it is cut short -- so this run's server was started with the installed one, '%s'."
        " Remove it, and the next server started without one writes it again" % (own, installed),
    )
    return installed


def no_systems(world, environment, installed_by_us, said):
    """The failure a run whose server read a server configuration with nothing in it fails with."""
    read = environment.get(SERVER_CONFIG)
    if read and installed_by_us:
        which = "the installed one, '%s', which this run started it with" % read
    elif read:
        which = "'%s', which %s names" % (read, SERVER_CONFIG)
    else:
        which = (
            "its copy in '~/.gz/sim/<release>/server.config', which Gazebo copies there the first time a server"
            " starts, and which a server that reads it while another is still writing it finds empty: remove it"
            " if it is, and the next server started writes it again"
        )
    return (
        "Gazebo's server for world '%s' loaded no systems, so it would never publish a pose: a world that names"
        " none of its own runs with the ones its server configuration lists -- the physics, and the scene"
        " broadcaster the poses come from -- and the configuration it read is none: %s. The server wrote:\n%s"
        % (world, which, said.tail(0))
    )


# How long a run may take, in wall-clock seconds, when PartCAD does not say: the
# bound this script always had. PartCAD hands every run its budget as 'timeout'.
DEFAULT_TIMEOUT = 300.0

# How much of that budget is left unused, so that the run stops itself and says
# how far it got before PartCAD stops it and can say only that it did not
# finish: a tenth of it, at least one second and at most thirty.
BUDGET_MARGIN = 0.1
BUDGET_MARGIN_MIN = 1.0
BUDGET_MARGIN_MAX = 30.0

# How long the server may take to publish its first pose, in wall-clock seconds,
# unless 'startup_timeout' says otherwise. Loading a world and its meshes, and
# a first start in a container, take seconds; a server that has said nothing in
# two minutes is one that will not.
STARTUP_TIMEOUT = 120.0

# How long the poses may say nothing before the first reading, in wall-clock
# seconds, before they are subscribed to again -- and twice as long again each
# time after that, inside 'startup_timeout'. gz-transport registers a
# subscriber with the server publishing its topic once, in one UDP datagram,
# when it first hears of that server; a datagram lost -- and a machine running
# several servers at once drops them, every process hearing every other's
# discovery traffic -- leaves a subscriber that is connected and hears nothing,
# for ever, from a server that is running and publishing to nobody. A
# subscriber started again registers again. Starting one while a slow server is
# still loading the world costs nothing but the start: the world is paused
# until the first reading, so no pose is missed.
RESUBSCRIBE = 10.0

# How often a wait looks up from the stream it is reading to check on the
# server, in seconds.
POLL = 0.25

# How long a program is given to stop after being asked, in seconds, before it
# is killed with everything it started.
GRACE = 5.0

# How much of what the server wrote to standard error a failure carries.
STDERR_LINES = 30


def positive(value):
    """'value' as a positive, finite number of seconds, or None."""
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0 else None


class Deadline:
    """The instant this run gives up, from the budget it was handed and when it started."""

    def __init__(self, budget, started=None):
        self.started = time.monotonic() if started is None else started
        self.budget = budget
        margin = min(BUDGET_MARGIN_MAX, max(BUDGET_MARGIN_MIN, BUDGET_MARGIN * budget))
        self.at = self.started + (budget - margin if budget > margin else budget / 2.0)

    def remaining(self):
        return max(0.0, self.at - time.monotonic())

    def passed(self):
        return time.monotonic() >= self.at

    def elapsed(self):
        return time.monotonic() - self.started


class Programs:
    """Every program a run starts, each in a process group of its own, all stopped together.

    A group of its own so that stopping it stops what it started too: 'gz' is a
    front end, and the server it runs or the tool it hands a topic to may well
    be a process of its own. Stopped on every way out of the run -- asked
    first, killed with its whole group after 'GRACE' seconds -- so that nothing
    outlives it to hold the run's output open or a partition busy.
    """

    def __init__(self):
        self.started = []

    def start(self, command, **options):
        if os.name == "nt":
            options.setdefault("creationflags", getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
        else:
            options.setdefault("start_new_session", True)
        process = subprocess.Popen(command, **options)
        self.started.append(process)
        return process

    def stop(self, grace=GRACE):
        running = [p for p in self.started if p.poll() is None]
        for process in self.started:
            # Every group, the leader's exit notwithstanding: what it started
            # may still be in it -- a server that died leaving its own server
            # behind. That can only reach another group if the leader's number
            # was handed out again, which POSIX does not do while anything is
            # still in its group; with nothing left in it, the number is free,
            # and a new group of that number started in the moments since is
            # the risk taken, in exchange for not leaving a live server behind.
            _signal_group(process, signal.SIGTERM)
        deadline = time.monotonic() + grace
        for process in running:
            _wait(process, max(0.0, deadline - time.monotonic()))
        for process in self.started:
            _signal_group(process, getattr(signal, "SIGKILL", signal.SIGTERM))
            _wait(process, 1.0)
        for process in self.started:
            for stream in (getattr(process, "stdout", None), getattr(process, "stderr", None)):
                try:
                    if stream is not None and hasattr(stream, "close"):
                        stream.close()
                except OSError:
                    pass


def _wait(process, seconds):
    """Wait up to 'seconds' for 'process' to end, and reap it if it has."""
    wait = getattr(process, "wait", None)
    if wait is None:
        return
    try:
        wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        pass


def _signal_group(process, sig):
    """Send 'sig' to the process group 'process' leads; on Windows, take its tree."""
    pid = getattr(process, "pid", None)
    if pid is None:
        return
    if os.name == "nt":
        if sig == signal.SIGTERM and process.poll() is None:
            try:
                process.terminate()
            except OSError:
                pass
            return
        try:
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=30,
            )
        except (OSError, subprocess.SubprocessError):
            pass
        return
    try:
        os.killpg(pid, sig)
    except (OSError, ProcessLookupError):
        pass


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class Lines:
    """The end of what a program writes on a stream, read on a thread so the program never blocks on it."""

    def __init__(self, stream, keep=STDERR_LINES):
        self.kept = collections.deque(maxlen=keep)
        self.thread = None
        if stream is not None and not isinstance(stream, (str, bytes)) and hasattr(stream, "__iter__"):
            self.thread = threading.Thread(target=self._read, args=(stream,), daemon=True)
            self.thread.start()

    def _read(self, stream):
        try:
            for line in stream:
                # Gazebo colours its console even into a pipe; a message
                # quoted in a report is read, not displayed.
                self.kept.append(_ANSI.sub("", line.rstrip("\n")))
        except (OSError, ValueError):
            pass

    def tail(self, wait=0.5):
        """What has been kept, once the stream has had 'wait' seconds to finish."""
        if self.thread is not None:
            self.thread.join(wait)
        return "\n".join(line for line in list(self.kept) if line.strip())

    def says(self, text):
        """Whether any line kept so far has 'text' in it."""
        return any(text in line for line in list(self.kept))


# What 'Messages.next' hands back once the stream has ended.
ENDED = object()


class Messages:
    """The messages of a 'gz topic -e' stream, read on a thread of their own.

    So that waiting for the next one is a wait with a bound, rather than a read
    that returns only when the subscriber says something -- which a subscriber
    whose server has died, or never started, never does.
    """

    def __init__(self, stream):
        self.queue = queue.Queue()
        self.count = 0
        self.current = None
        self.follow(stream)

    def follow(self, stream):
        """Read 'stream' from now on, in place of the one before: what that one says after this is dropped,
        its end included, which is the end of a subscriber this run stopped and not of the one it reads."""
        self.current = stream
        threading.Thread(target=self._read, args=(stream,), daemon=True).start()

    def _read(self, stream):
        try:
            for text in messages(stream):
                if stream is not self.current:
                    return
                self.queue.put(text)
        except (OSError, ValueError):
            pass
        finally:
            if stream is self.current:
                self.queue.put(ENDED)

    def next(self, timeout):
        """The next message, ENDED once there will be no more, or None if 'timeout' passed first."""
        try:
            text = self.queue.get(timeout=timeout)
        except queue.Empty:
            return None
        if text is not ENDED:
            self.count += 1
        return text


def ended_how(returncode):
    """How a program ended, in words: its exit status, or the signal that killed it."""
    if returncode is not None and returncode < 0:
        try:
            return "killed by signal %d (%s)" % (-returncode, signal.Signals(-returncode).name)
        except ValueError:
            return "killed by signal %d" % -returncode
    return "exit status %s" % returncode


# The kinds of joint 'kinematics.classify' finds that are read from the poses
# of their bodies ('posed_states'): its BALL and its FLOATING, spelled out
# rather than imported, for the reason MM_PER_M is.
BALL = "ball"
FLOATING = "floating"
POSED = (BALL, FLOATING)


class Names:
    """PartCAD's names for what a world calls its entities, out of the sidecar beside it.

    The sidecar is PartCAD's (see 'naming.read_sidecar' there): its 'bodies'
    say where each PartCAD body is -- the model's name and the link's -- its
    'joints' the same of each reported joint, with the joint's 'map' back into
    PartCAD's terms ('kinematics.joint_map'), and its 'names' which name the
    scene and each body became. Anything it does not mention -- everything in a
    world with no sidecar -- is reported as it is.
    """

    def __init__(self, sidecar=None):
        sidecar = sidecar or {}
        self.bodies = {}
        for name, entry in (sidecar.get("bodies") or {}).items():
            path = entry.get("path") if isinstance(entry, dict) else None
            if isinstance(path, list) and path:
                self.bodies[name] = tuple(str(part) for part in path)
        self.joints = {}
        # (model, SDFormat joint) -> the joint's sidecar entry, where it carries
        # a map: what 'kinematics.reading' makes PartCAD's reading of it from.
        self.maps = {}
        # Reported name -> the entry of every joint read from the poses of its
        # bodies rather than from a joint state (see 'posed_states').
        self.posed = {}
        for name, entry in (sidecar.get("joints") or {}).items():
            if not isinstance(entry, dict):
                continue
            path = entry.get("path")
            mapped = isinstance(entry.get("map"), dict)
            if isinstance(path, list) and len(path) >= 2:
                key = ("::".join(str(part) for part in path[:-1]), str(path[-1]))
                self.joints[key] = name
                if mapped:
                    self.maps[key] = entry
            if mapped and entry["map"].get("kind") in POSED:
                self.posed[name] = entry
        # Part label -> the PartCAD body it is welded into, what it is and
        # where it sits in that body ('naming.parts_of'): what a run reports as
        # 'parts', so that a body's loads can be read as a part's.
        try:
            import naming

            self.parts = naming.parts_of(sidecar)
        except (ImportError, AttributeError):
            # A PartCAD from before a sidecar said which parts a body is.
            self.parts = {}
        names = sidecar.get("names") or {}
        self.scenes = {str(internal): name for name, internal in (names.get("scene") or {}).items()}
        self.models = {str(internal): name for name, internal in (names.get("body") or {}).items()}

    @classmethod
    def beside(cls, scene_file):
        try:
            import naming
        except ImportError:
            # A PartCAD from before names were transcoded, which wrote none.
            return cls()
        return cls(naming.read_sidecar(scene_file))

    def world(self, internal):
        return self.scenes.get(internal, internal)

    def posing(self):
        """The models whose links' poses a run needs every step, by scoped name.

        Each ball's, for its speed, and for each floating child its own model
        and the model of the body it is read against, for the two bodies'
        velocities: what 'kinematics.reading' reads a free body's rates from.
        """
        models = set()
        for entry in self.posed.values():
            if entry["map"].get("kind") == FLOATING:
                for body in (entry.get("body"), entry["map"].get("parent")):
                    path = self.bodies.get(body)
                    if path:
                        models.add("::".join(path[:-1]))
            else:
                models.add("::".join(str(part) for part in (entry.get("path") or [])[:-1]))
        return models

    def model(self, scoped):
        return self.models.get(scoped, scoped)

    def joint(self, scoped, name):
        """PartCAD's name for joint 'name' of the model 'scoped', or None where the sidecar does not say."""
        return self.joints.get((scoped, name))


# Every name as the world has it: a world with no sidecar.
AS_WRITTEN = Names()


def progress(world, before, after, count, duration):
    """What had been received when a run failed, for the message it fails with."""
    if before is None and not count:
        return "No pose of world '%s' had been received." % world
    if before is None:
        return "%d pose messages of world '%s' had been received, and no first reading taken from any of them." % (
            count,
            world,
        )
    return "%d pose messages of world '%s' had been received, the last at t = %.3g s of the %.3g s asked for." % (
        count,
        world,
        after["time"],
        duration,
    )


def no_first_reading(world, topic, partition, waited, read, seen, streams, behind, tail, subscriptions=1):
    """The failure a run with no first reading by its 'startup_timeout' fails with: what came, and what did not.

    'read' is how many pose messages came that named the bodies this run
    reads, 'seen' the instant of the latest of them, and 'behind' each joint
    stream that had only a later state than that to give, by model, with the
    time of it (see 'fill'). With no pose at all, what the joint streams did
    say tells a server that never loaded the world from one that is running
    and publishes no pose: joint states come from the same server, paused or
    not, and poses do not come at all from one with no scene broadcaster.
    'subscriptions' is how many subscribers the poses had (see 'RESUBSCRIBE').
    """
    wrote = (" The server wrote:\n%s" % tail) if tail else ""
    if read:
        return (
            "Gazebo published %d pose messages for world '%s' within %.0f s of being started, the latest at t = %.3g"
            " s, and no first reading could be taken from any of them: %s, so its joints would not have been where"
            " its bodies were.%s"
            % (
                read,
                world,
                waited,
                seen,
                "; ".join(
                    "the joint states of model '%s' had only a later instant to give (t = %.3g s)" % (model, stamp)
                    for model, stamp in sorted(behind.items())
                )
                or "a joint stream had only a later instant to give",
                wrote,
            )
        )
    head = (
        "Gazebo published no poses for world '%s' within %.0f s of being started, so there is nothing to report:"
        " nothing came on '%s' in the transport partition '%s'%s."
        % (
            world,
            waited,
            topic,
            partition,
            " to any of the %d subscribers it had" % subscriptions if subscriptions > 1 else "",
        )
    )
    spoke = [
        stream
        for stream in streams
        if not isinstance(stream.model, Wrench) and not stream.model.links and getattr(stream, "count", 0)
    ]
    if spoke:
        return (
            "%s The server was running all the same: it published %s -- so it had loaded the world, and either runs"
            " without the scene broadcaster poses come from, or never heard this run subscribe to them.%s"
            % (
                head,
                "; ".join(
                    "%d joint states of model '%s', the latest at t = %.3g s"
                    % (stream.count, stream.model.shown, stream.newest or 0.0)
                    for stream in spoke
                ),
                wrote,
            )
        )
    return (
        "%s A server that hangs while it loads the world, or one publishing in another partition, looks like this."
        " Raise 'startup_timeout' if this machine is simply slow to start one.%s" % (head, wrote)
    )


def server_died(simulator, said, world, before, after, count, duration):
    """The failure a run whose server exited fails with."""
    head = (
        "Gazebo published no poses for world '%s': its server stopped before the first one (%s)."
        % (world, ended_how(simulator.poll()))
        if before is None and not count
        else "Gazebo's server stopped before the run was over (%s). %s"
        % (ended_how(simulator.poll()), progress(world, before, after, count, duration))
    )
    tail = said.tail()
    if tail:
        return "%s The end of what the server wrote:\n%s" % (head, tail)
    return "%s The server wrote nothing on standard error." % head


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


# SDFormat's own gravity, in m/s^2: what Gazebo applies to a world that states
# none, which is what PartCAD writes for a scene that states none.
SDF_DEFAULT_GRAVITY = (0.0, 0.0, -9.8)


def world_gravity(path, gravity=None):
    """The gravity the world in 'path' runs under, in m/s^2 -- made 'gravity' first, if one is given.

    The world arrives with the scene's gravity in it, or with none, which is
    Gazebo's own. 'gravity' is what a 'simulate:' passed in 'params' for this
    one run, and it is the explicit answer that beats the scene's -- so it is
    written into the file Gazebo is about to load, which is also the file the
    snapshots are drawn from and the run directory keeps. This package's
    declaration states no default for it, deliberately: a default would be
    passed in every request and replace every scene's.

    What is returned is what the run will actually be under, which is what the
    result reports -- not what was asked for, which is the same number or is
    nothing at all.
    """
    from xml.etree import ElementTree

    tree = ElementTree.parse(path)
    root = tree.getroot()
    element = root if root.tag == "world" else root.find("world")
    if element is None:
        raise Exception("The world file holds no world: %s" % path)
    import gazebo_common

    if gravity is not None:
        vector = gazebo_common.gravity_vector(gravity, "the simulation's 'gravity' for %s" % path)
        node = element.find("gravity")
        if node is None:
            node = ElementTree.SubElement(element, "gravity")
        node.text = " ".join(repr(v) for v in vector)
        tree.write(path, encoding="utf-8", xml_declaration=True)
    text = element.findtext("gravity")
    if not text or not text.strip():
        return list(SDF_DEFAULT_GRAVITY)
    return gazebo_common.gravity_vector(text.split(), "the '<gravity>' of %s" % path)


# Gazebo's own step, in seconds: what a world that states no '<max_step_size>'
# is stepped by.
SDF_DEFAULT_STEP = 0.001

# How many pose messages a second the scene broadcaster publishes -- a second of
# the wall clock's, not the simulation's. Harmonic's SceneBroadcaster throttles
# '/world/<name>/pose/info' to sixty a second whatever its 'dynamic_pose_hertz'
# says (which only paces 'dynamic_pose/info', a topic that leaves out every
# static entity), so a run that wants a reading more often than that, in
# simulated time, has to run slower than real time to get it ('pace').
POSE_MESSAGES_PER_SECOND = 60.0
# How far below the slowest pace that would do a run is set, so that a pose
# message that comes a little late still comes before the next sample is due.
PACE_MARGIN = 0.8


def world_physics(path, timestep=None, real_time_factor=None):
    """Write the step and the pace a run asks for into the world in 'path', and return the step it will run at.

    'timestep' is the integration step, in seconds: SDFormat's
    '<physics><max_step_size>', which is the step Gazebo's DART takes. (It
    used to be handed to the server as '-z 1/timestep', which is an *update
    rate* -- iterations a second of the wall clock -- and changed only how fast
    a run went, never the step: a 'timestep' of 0.0005 ran the world twice as
    fast as real time on its own step of 1 ms.) 'real_time_factor' is
    '<real_time_factor>', how much simulated time passes a second of the wall
    clock's -- 1, Gazebo's own, unless the run asks for readings more often
    than a pose message comes (see 'pace').

    Written into the first '<physics>' the world has, or a new one, as
    'world_gravity' writes the gravity: into the file the server is about to
    load, which the run directory keeps.
    """
    from xml.etree import ElementTree

    tree = ElementTree.parse(path)
    root = tree.getroot()
    element = root if root.tag == "world" else root.find("world")
    if element is None:
        raise Exception("The world file holds no world: %s" % path)
    block = element.find("physics")
    if timestep is not None or real_time_factor is not None:
        if block is None:
            block = ElementTree.Element("physics", {"name": "partcad", "type": "dart"})
            element.insert(0, block)
        for tag, value in (("max_step_size", timestep), ("real_time_factor", real_time_factor)):
            if value is not None:
                node = block.find(tag)
                if node is None:
                    node = ElementTree.SubElement(block, tag)
                node.text = repr(float(value))
        tree.write(path, encoding="utf-8", xml_declaration=True)
    text = block.findtext("max_step_size") if block is not None else None
    try:
        step = float(text) if text is not None and text.strip() else SDF_DEFAULT_STEP
    except ValueError:
        step = SDF_DEFAULT_STEP
    return step if step > 0 else SDF_DEFAULT_STEP


def pace(samples, duration, step):
    """The real-time factor that has Gazebo publish a pose message for every sample asked for, or None for its own.

    A reading is a pose message, and those come sixty times a second of the
    wall clock's ('POSE_MESSAGES_PER_SECOND'): a run in real time has sixty
    readings to give a simulated second, and one that asks for more is run
    slower than real time, just slow enough to give them -- as many pose
    messages a simulated second as there are samples to take, and a margin
    more. Never more than one a step, which is the most Gazebo has to give.
    """
    if samples <= 0:
        return None
    every = max(duration / (samples + 1), step)
    factor = PACE_MARGIN * every * POSE_MESSAGES_PER_SECOND
    return factor if factor < 1.0 else None


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


def snapshot(message, names=AS_WRITTEN):
    """Where every body is in one pose message, keyed by PartCAD's name for it.

    With a sidecar ('names'), exactly the bodies PartCAD's exporter wrote, each
    at its link's pose in the world's frame: the poses along its path -- its
    model's, relative to the world, and its link's, relative to the model --
    composed. A body whose model or link the message does not name once is left
    out of that reading rather than placed at a guess.

    Without one, every entity, keyed by the name Gazebo gave it, relative to
    its parent as Gazebo publishes it; the world itself is left out: it is the
    frame everything else is stated in and it never moves.

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

    if names.bodies:
        return {"time": seconds, "bodies": world_poses(message, names), "joints": {}}

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


def world_poses(message, names):
    """Each PartCAD body of 'names' at its link's pose in the world, from one pose message."""
    import urdf_common

    poses = unique_poses(message)
    bodies = {}
    for name, path in names.bodies.items():
        if not all(part in poses for part in path):
            continue
        frame = poses[path[0]]
        for part in path[1:]:
            frame = urdf_common.compose(frame, poses[part])
        quat, pos = frame
        bodies[name] = {"pos": [float(v) for v in pos], "quat": [float(v) for v in quat]}
    return bodies


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
# model, then one 'joint' per joint, each with an 'axis1' (and an 'axis2', for a
# joint of two) holding its 'position' and 'velocity' in radians or metres. A
# world does not carry it -- PartCAD's exporter writes none, and a world
# somebody else wrote has no reason to -- so the world that runs is a copy of
# the one handed over, with the system added to every model that has a joint to
# read.
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
# it out -- so that is read out of the sidecar, or out of the world, while the
# copy is made.
#
# Two kinds of joint are not read from joint states at all. A ball's are two of
# its three coordinates -- the publisher writes 'axis1' and 'axis2' and DART's
# 'BallJoint' has a third -- which is not an orientation; and the child of a
# floating joint is no joint of the world, but a model of its own. Both are
# read from where their bodies are, which every pose message says (see
# 'posed_states'), and a ball's angular velocity from where its two links are
# a step either side of the reading, which a PosePublisher system on its model
# says every step (see 'LinkStream'): the pose messages come a sixtieth of a
# second apart, which is too far apart to tell a speed by.
#

JOINT_STATE_PUBLISHER = {
    "filename": "gz-sim-joint-state-publisher-system",
    "name": "gz::sim::systems::JointStatePublisher",
}

# Every link's pose in its model, every step, as one 'gz.msgs.Pose_V' on
# '/model/<model>/pose' -- a topic it names itself, after the model.
POSE_PUBLISHER = {
    "filename": "gz-sim-pose-publisher-system",
    "name": "gz::sim::systems::PosePublisher",
}
POSE_PUBLISHER_SETTINGS = (
    ("publish_link_pose", "true"),
    ("publish_visual_pose", "false"),
    ("publish_collision_pose", "false"),
    ("publish_sensor_pose", "false"),
    # The model's own pose in the world too, so that a link's world pose -- and
    # its world velocity, a step either side -- can be had from one message.
    ("publish_model_pose", "true"),
    ("publish_nested_model_pose", "false"),
    ("use_pose_vector_msg", "true"),
    # Every step: nought or less is "as often as the world steps".
    ("update_frequency", "-1"),
)

# The SDFormat joint types that are read in a world PartCAD did not write -- one
# with no sidecar, and so no joint map -- and what each is called in PartCAD,
# which is the same name for each of them but a 'continuous', which is a
# 'revolute' bounded on neither side (PartCAD has no 'continuous', and the
# MuJoCo plugin reports a hinge as a 'revolute' whatever its bounds). A screw is
# read as its turn, which is the coordinate Gazebo gives it; the move along its
# axis follows from the thread.
TURNS = ("revolute", "continuous", "screw")
MOVES = ("prismatic",)
SDF_TYPES = {"continuous": "revolute"}

# The joint types that are not, and why. Each such joint is named in the
# result's 'warnings' instead: something may have moved there, and a reading
# that quietly leaves it out is worse than one that says it did.
NOT_READ = {
    "ball": "Gazebo publishes two of a ball joint's three coordinates, which is not an orientation",
    "universal": "it turns about two axes, and a world PartCAD did not write does not say which turn each is",
    "revolute2": "it turns about two axes, and PartCAD has no motion of that type",
    "gearbox": "PartCAD has no motion of that type",
}

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

    def __init__(self, scoped, topic, joints, shown=None, maps=None, links=False):
        # 'outer::inner' for a nested model, which is how Gazebo scopes one.
        self.scoped = scoped
        # What a warning calls it: PartCAD's name for it, where there is one.
        self.shown = shown or scoped
        # The topic its publisher was told to publish on.
        self.topic = topic
        # {name in the world: (type, name reported under, SDFormat type)}
        self.joints = joints
        # {name in the world: its sidecar entry}, for each joint that has a
        # map back into PartCAD's terms ('kinematics.joint_map').
        self.maps = maps or {}
        # Whether what arrives on 'topic' is its links' poses (a PosePublisher's,
        # for its balls) rather than its joints' states.
        self.links = links


def plan_joints(world, prefix, warnings, names=AS_WRITTEN):
    """Add a JointStatePublisher to every model of 'world' that has a joint to read.

    Returns a 'JointModel' for each, in document order, the publisher of the
    n-th told to publish on '<prefix>/<n>' -- a topic of this run's own rather
    than Gazebo's default, so that nothing has to guess how Gazebo names the
    topic of a nested model. Two runs on one machine use the same topics and
    are kept apart by their partitions (see 'run_environment').

    A joint the sidecar ('names') has a map for is read as the map says,
    under PartCAD's name for it -- but a ball, which is read from its links'
    poses. A model whose links' poses a run needs every step ('Names.posing':
    a ball's, for its speed; a floating child's and the model it is read
    against, for their velocities) gets a PosePublisher, and a 'JointModel'
    with 'links' set. A joint with no map is
    reported under its own name where no other joint read has it, and as
    '<model>::<joint>' where one does -- for every one of them, so that
    neither is the joint the plain name means. Names are only unique within a
    model, and a world with two of the same robot in it has two of every joint.

    A fixed joint is not read: nothing moves there. Nor is a joint of a static
    model, or of a model nested in one: the world says nothing of it moves, so
    there is nothing to say. A joint of a type that is not read is named in
    'warnings'.
    """
    found = []
    posing = []
    wanted = names.posing()

    def visit(model, scope, static):
        scoped = scope + [model.get("name") or "model"]
        static = static or (model.findtext("static") or "").strip().lower() in ("true", "1")
        if not static:
            read, maps = {}, {}
            for joint in model.findall("joint"):
                name = joint.get("name")
                declared = (joint.get("type") or "").strip().lower()
                if not name or declared == "fixed":
                    continue
                entry = names.maps.get(("::".join(scoped), name))
                if entry is not None:
                    kind = entry["map"].get("kind")
                    if kind not in POSED:
                        read[name] = (kind, declared)
                        maps[name] = entry
                elif declared in TURNS or declared in MOVES:
                    read[name] = (SDF_TYPES.get(declared, declared), declared)
                else:
                    note(
                        warnings,
                        "joint '%s' of model '%s' is not reported: it is a '%s' joint, and %s"
                        % (
                            names.joint("::".join(scoped), name) or name,
                            names.model("::".join(scoped)),
                            declared,
                            NOT_READ.get(declared, "this plugin does not know the type"),
                        ),
                    )
            if read:
                found.append((model, "::".join(scoped), read, maps))
            if "::".join(scoped) in wanted:
                posing.append((model, scoped))
        for nested in model.findall("model"):
            visit(nested, scoped, static)

    for model in world.findall("model"):
        visit(model, [], False)

    counts = collections.Counter(name for _model, _scoped, read, _maps in found for name in read)
    models = []
    for index, (element, scoped, read, maps) in enumerate(found):
        topic = "%s/%d" % (prefix, index)
        plugin = ElementTree.SubElement(element, "plugin", dict(JOINT_STATE_PUBLISHER))
        ElementTree.SubElement(plugin, "topic").text = topic
        joints = {
            name: (
                kind,
                names.joint(scoped, name) or (name if counts[name] == 1 else "%s::%s" % (scoped, name)),
                declared,
            )
            for name, (kind, declared) in read.items()
        }
        models.append(JointModel(scoped, topic, joints, names.model(scoped), maps))
    for element, scoped in posing:
        plugin = ElementTree.SubElement(element, "plugin", dict(POSE_PUBLISHER))
        for key, value in POSE_PUBLISHER_SETTINGS:
            ElementTree.SubElement(plugin, key).text = value
        # The topic Harmonic's PosePublisher names after the model it is on.
        topic = "/model/%s/pose" % "/".join(scoped)
        models.append(JointModel("::".join(scoped), topic, {}, names.model("::".join(scoped)), links=True))
    return models


#
# Reading what holds each body
#
# What a joint passes from one body to the other is what Gazebo's force-torque
# sensor measures -- the wrench the ForceTorque system reads off the joint's
# 'JointTransmittedWrench', its constraint and its spring, damper and friction
# together -- and Gazebo makes one only for a joint that carries a
# '<sensor type="force_torque">', in a world that runs the ForceTorque system.
# So, as for the joint states, the world that runs is a copy with a sensor on
# every joint that moves and the system added: once, to a model, for the reason
# the publishers are -- a system attached to the world would turn off the
# defaults. Each sensor measures the wrench the parent applies to the child,
# about the joint's origin and in the child link's orientation ('parent_to_child'
# in the 'child' frame), on a topic of this run's own.
#
# What it does not give: a joint to the world -- a body a scene fixes -- gets no
# sensor (Harmonic makes none for one), and the contacts are not read. Each is
# named in 'warnings' rather than reported as nought, and so is that Gazebo
# publishes no accelerations. A body's weight is the world file's: its link's
# '<inertial>'.

FORCE_TORQUE = {
    "filename": "gz-sim-forcetorque-system",
    "name": "gz::sim::systems::ForceTorque",
}

# What a load from the world names as where it is from.
WORLD = "world"


class Wrench:
    """One joint whose wrench is read: where its sensor publishes, and what it joins."""

    # Read by 'fill', which reads joint states off every stream but these.
    links = False

    def __init__(self, topic, scoped, joint, reported, child, parent, origin):
        self.topic = topic
        # The model and the joint as the world names them.
        self.scoped = scoped
        self.joint = joint
        # PartCAD's name for the joint, where there is one.
        self.reported = reported
        # The PartCAD body of its child link, and of its parent link or 'world'.
        self.child = child
        self.parent = parent
        # The joint's frame in its child link's, '(q, t)' with 't' in mm: where
        # its sensor measures about.
        self.origin = origin


# The rate a force-torque sensor is asked for to measure every step: nought,
# which is SDFormat's "every cycle". Not one over the step, which is the same
# thing said in hertz and is what this was: Harmonic's DART does not take a
# step past t = 0 with a sensor asking for 2000 a second on a 0.5 ms step --
# or for 1999 -- while 1000 runs, and so does nought. A run with a step of
# 1 ms never met it.
EVERY_STEP = 0.0


def plan_wrenches(world, prefix, rate, warnings, names=AS_WRITTEN):
    """Put a force-torque sensor on every joint of 'world' between two PartCAD bodies, and its system on a model.

    Returns a 'Wrench' for each, in document order, the n-th publishing on
    '<prefix>/<n>' at 'rate' a second -- every step, for 'EVERY_STEP'. Only for
    a world PartCAD exported: a body's loads are stated in the
    world's frame, which takes the sidecar's bodies to place. A joint to the
    world, which Gazebo gives no sensor, and a joint of a static model, which
    moves nothing, are passed over; the first is named in 'warnings', and so is
    a joint whose child or parent link is none of the sidecar's bodies, which
    would otherwise vanish from both ends' loads unremarked.
    """
    import gazebo_common

    bodies = {tuple(path): name for name, path in names.bodies.items()}
    if not bodies:
        return []
    found = []
    host = None

    def visit(model, scope, static):
        nonlocal host
        scoped = scope + [model.get("name") or "model"]
        static = static or (model.findtext("static") or "").strip().lower() in ("true", "1")
        for joint in model.findall("joint") if not static else ():
            name = joint.get("name")
            child_link = (joint.findtext("child") or "").strip()
            child = bodies.get(tuple(scoped) + tuple(child_link.split("::")))
            parent_link = (joint.findtext("parent") or "").strip()
            if not name:
                continue
            if child is None:
                note(
                    warnings,
                    "the load of joint '%s' is not reported: its child link, '%s', is none of the bodies the names"
                    " file lists" % (names.joint("::".join(scoped), name) or name, child_link),
                )
                continue
            if parent_link == "world":
                note(
                    warnings,
                    "what holds '%s' to the world is not among its loads: Gazebo makes no force-torque sensor on a"
                    " joint to the world" % child,
                )
                continue
            parent = bodies.get(tuple(scoped) + tuple(parent_link.split("::")))
            if parent is None:
                note(
                    warnings,
                    "the load of joint '%s' is not reported: its parent link, '%s', is none of the bodies the names"
                    " file lists" % (names.joint("::".join(scoped), name) or name, parent_link),
                )
                continue
            pose = joint.find("pose")
            if pose is not None and (pose.get("relative_to") or "").strip() not in ("", joint.findtext("child")):
                note(
                    warnings,
                    "the load of joint '%s' is not reported: its frame is stated relative to '%s', and this plugin"
                    " reads a joint's frame in its child link's" % (name, pose.get("relative_to")),
                )
                continue
            topic = "%s/%d" % (prefix, len(found))
            sensor = ElementTree.SubElement(joint, "sensor", {"name": "partcad_wrench", "type": "force_torque"})
            ElementTree.SubElement(sensor, "always_on").text = "1"
            ElementTree.SubElement(sensor, "update_rate").text = "%g" % rate
            ElementTree.SubElement(sensor, "topic").text = topic
            measured = ElementTree.SubElement(sensor, "force_torque")
            ElementTree.SubElement(measured, "frame").text = "child"
            ElementTree.SubElement(measured, "measure_direction").text = "parent_to_child"
            found.append(
                Wrench(
                    topic,
                    "::".join(scoped),
                    name,
                    names.joint("::".join(scoped), name) or name,
                    child,
                    parent,
                    gazebo_common.parse_pose(pose),
                )
            )
            host = host if host is not None else model
        for nested in model.findall("model"):
            visit(nested, scoped, static)

    for model in world.findall("model"):
        visit(model, [], False)
    if host is not None:
        ElementTree.SubElement(host, "plugin", dict(FORCE_TORQUE))
    return found


def link_inertials(scene_file):
    """Each link's mass, kg, and centre of mass in its frame (mm), by (model, ..., link): a body's weight."""
    import gazebo_common

    root = ElementTree.parse(scene_file).getroot()
    world = root if root.tag == "world" else root.find("world")
    found = {}

    def visit(model, scope):
        scoped = scope + (model.get("name") or "model",)
        for link in model.findall("link"):
            inertial = link.find("inertial")
            if inertial is None or inertial.findtext("mass") is None:
                continue
            try:
                mass = float(inertial.findtext("mass"))
            except ValueError:
                continue
            found[scoped + (link.get("name"),)] = (mass, gazebo_common.parse_pose(inertial.find("pose"))[1])
        for nested in model.findall("model"):
            visit(nested, scoped)

    for model in world.findall("model") if world is not None else ():
        visit(model, ())
    return found


def _vector(data):
    """A protobuf 'Vector3d' as Gazebo prints it, each field nought where it leaves it out."""
    data = data if isinstance(data, dict) else {}
    return [float(data.get(axis) or 0.0) for axis in ("x", "y", "z")]


def fill_loads(reading, streams, inertials, gravity, warnings, patience=PATIENCE, names=AS_WRITTEN, paused=False):
    """State in 'reading' what acts on every body at its instant: its weight, and what each joint passes it.

    In the vocabulary the MuJoCo plugin states it in (its 'body_loads'): per
    body, 'mass' and 'com', and 'loads', each with its 'kind', 'from', 'joint',
    'at' (mm), 'force' (N) and 'torque' (N*m about 'at'), in the world's frame
    and acting on the body. A joint's load is its sensor's wrench, turned from
    the child link's orientation into the world's and stated at the joint's
    origin; the child reports it from its parent and the parent the same
    reversed. A body's weight is its link's mass under the world's gravity, at
    its centre of mass.

    'paused' is the first reading, taken while the world is paused: Gazebo's
    force-torque sensors measure nothing then, so that reading carries the
    weights alone, and says why.

    A joint is stated about its child, so one whose child is not in the reading
    is not reported at all, and one whose parent is not is reported on its child
    alone; either is named in 'warnings' rather than left for a reader to take
    for a joint passing nothing.
    """
    import urdf_common

    bodies = reading["bodies"]
    joints, weights = {}, {}
    for name, body in bodies.items():
        inertial = inertials.get(tuple(names.bodies.get(name) or ()))
        if inertial is None:
            continue
        mass, centre = inertial
        com = [p + v for p, v in zip(body["pos"], urdf_common.rotate_vec(tuple(body["quat"]), centre))]
        body["mass"] = mass
        body["com"] = com
        weights[name] = {
            "kind": "gravity",
            "from": name,
            "at": com,
            "force": [mass * g for g in gravity],
            "torque": [0.0, 0.0, 0.0],
        }
    if paused and streams:
        note(
            warnings,
            "the first reading's loads are its bodies' weights alone: it is taken while the world is paused, and"
            " Gazebo's force-torque sensors measure nothing then",
        )
        streams = ()
    for stream in streams:
        sensor = stream.model
        child = bodies.get(sensor.child)
        if child is None:
            # Its child is where the sensor measures, and the joint is stated
            # at: without it there is nothing to turn the wrench by, on either
            # side.
            note(
                warnings,
                "the load of joint '%s' is not reported: its child body, '%s', is not in the reading"
                % (sensor.reported, sensor.child),
            )
            continue
        text = stream.at(reading["time"], patience)
        if text is None:
            note(
                warnings,
                "the load of joint '%s' is not reported: Gazebo published nothing from its force-torque sensor"
                % sensor.reported,
            )
            continue
        message = parse_message(text)
        rotation = tuple(child["quat"])
        force = list(urdf_common.rotate_vec(rotation, _vector(message.get("force"))))
        torque = list(urdf_common.rotate_vec(rotation, _vector(message.get("torque"))))
        at = [p + v for p, v in zip(child["pos"], urdf_common.rotate_vec(rotation, sensor.origin[1]))]
        joints.setdefault(sensor.child, []).append(
            {
                "kind": "joint",
                "from": sensor.parent,
                "joint": sensor.reported,
                "at": at,
                "force": force,
                "torque": torque,
            }
        )
        if sensor.parent in bodies:
            joints.setdefault(sensor.parent, []).append(
                {
                    "kind": "joint",
                    "from": sensor.child,
                    "joint": sensor.reported,
                    "at": list(at),
                    "force": [-v for v in force],
                    "torque": [-v for v in torque],
                }
            )
        else:
            note(
                warnings,
                "the load of joint '%s' is reported on '%s' alone: its parent body, '%s', is not in the reading"
                % (sensor.reported, sensor.child, sensor.parent),
            )
    for name, body in bodies.items():
        if name in weights or name in joints:
            body["loads"] = joints.get(name, []) + ([weights[name]] if name in weights else [])


def prepare_world(scene_file, warnings, names=AS_WRITTEN, loads=None):
    """The world to run, and the models whose joints are read in it.

    The world as it was handed over when nothing in it has a joint to read -- so
    a world of free models, which is every scene of parts PartCAD exports, runs
    exactly as it always has -- and otherwise a copy of it with the publishers
    added ('plan_joints'), which 'process' removes again when the run is over.
    The copy is written beside the world rather than anywhere else, because a
    world names its meshes relative to where it is -- and under a name of its
    own each time ('<stem>.joint-states.<random><extension>'), because a fixed
    one is shared by every run of the same world at once: each would write it
    over the other's, the first to finish would delete it from under the
    rest, and a file of that name somebody else put there would be lost.
    It is never left behind: one that cannot be written is removed here.

    Joints of a model the world brings in with '<include>' are not read: they
    are in another file. PartCAD's exporter writes everything into the one.

    With 'loads' -- a rate, a second, or 'EVERY_STEP' -- every joint between
    two bodies also gets a force-torque sensor ('plan_wrenches'), and the
    models come back with a 'Wrench' for each of those beside them.
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
    models = plan_joints(world, "/partcad/joint_state", warnings, names)
    if loads is not None:
        # And a force-torque sensor on every joint, read at 'loads' a second
        # (see 'plan_wrenches'), as 'Wrench' entries beside the models.
        models += plan_wrenches(world, "/partcad/wrench", loads, warnings, names)
    if not models:
        return scene_file, []
    stem, extension = os.path.splitext(os.path.basename(scene_file))
    copy = None
    try:
        handle, copy = tempfile.mkstemp(
            prefix="%s.joint-states." % stem,
            suffix=extension or ".world",
            dir=os.path.dirname(os.path.abspath(scene_file)),
        )
        with os.fdopen(handle, "wb") as out:
            tree.write(out, encoding="utf-8", xml_declaration=True)
    except BaseException as e:
        if copy is not None:
            try:
                os.remove(copy)
            except OSError:
                pass
        if not isinstance(e, OSError):
            raise
        note(warnings, "the joints are not reported: the world could not be copied to add what publishes them: %s" % e)
        return scene_file, []
    return copy, models


_STAMP = re.compile(r"header \{\s*stamp \{([^}]*)\}")
_SECONDS = re.compile(r"\bsec: (\d+)")
_NANOSECONDS = re.compile(r"\bnsec: (\d+)")


def message_time(text):
    """The simulated time a message was published at, read off its first header alone.

    A joint-state message arrives every step and nearly all of them are never
    read, so only the header of each is: the rest is parsed if a reading asks
    for that one. The same sum 'snapshot' makes of the same two fields, so that
    the two clocks compare equal at the same instant. The first header is the
    message's own in a joint-state message, and its first pose's in a
    PosePublisher's, where every pose carries the step's.
    """
    match = _STAMP.search(text)
    if match is None:
        return 0.0
    seconds = _SECONDS.search(match.group(1))
    nanoseconds = _NANOSECONDS.search(match.group(1))
    return float(seconds.group(1) if seconds else 0) + float(nanoseconds.group(1) if nanoseconds else 0) / 1e9


def separated(stream):
    """Every message on a 'gz topic -e' stream that prints a blank line after each.

    What a PosePublisher's 'Pose_V' needs, where 'messages' would find no
    message at all: it has no header of its own, only one in each pose.
    """
    current = []
    for line in stream:
        if not line.strip():
            if current:
                yield "".join(current)
                current = []
            continue
        current.append(line)
    if current:
        yield "".join(current)


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
        # How many messages have come, kept or not: what a failure says.
        self.count = 0
        self.ended = False
        self.silent = False
        self.condition = threading.Condition()

    def split(self, stream):
        return messages(stream)

    def follow(self, stream):
        """Keep every message 'stream' carries, until it ends."""
        try:
            for text in self.split(stream):
                stamp = message_time(text)
                with self.condition:
                    self.entries.append((stamp, text))
                    self.newest = stamp
                    self.count += 1
                    self.condition.notify_all()
        except (OSError, ValueError):
            # Closed under it, which is one of the ways a run ends.
            pass
        finally:
            with self.condition:
                self.ended = True
                self.condition.notify_all()

    def _wait(self, reached, patience):
        """Wait, holding the condition, until 'reached()', the stream ends, or 'patience' runs out."""
        deadline = time.monotonic() + patience
        while not self.ended and not self.silent and not reached():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.silent = self.newest is None
                break
            self.condition.wait(remaining)

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
            self._wait(lambda: self.newest is not None and self.newest >= moment, patience)
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


class LinkStream(JointStream):
    """The poses of one model's links, every step, as its PosePublisher prints them: what speeds are read from.

    A pose message comes every sixtieth of a second, and a ball's rotation and
    a free body's pose at a reading's instant are read from it exactly (see
    'posed_states'); a speed is not in it, and a difference over a sixtieth of
    a second is percents off for anything that swings. So the model publishes
    its links every step as well, and a speed is what the last step before the
    reading moved a body by, over the step ('last_step'): Gazebo's DART steps
    by semi-implicit Euler, which moves a body by the speed it ends the step
    with, so that is the speed the engine holds at the reading -- the one its
    joint states report for every other joint -- and not the speed half a step
    on that a difference across the reading would be.
    """

    def split(self, stream):
        return separated(stream)

    def last_step(self, moment, patience=PATIENCE):
        """The message of 'moment' and the one before it, as '(stamp, text)' pairs, or None.

        Where the stream has no message of that very step, or none before it,
        the two nearest it -- one each side where there are -- which is a
        speed a step away at worst.
        """
        with self.condition:
            self._wait(lambda: self.newest is not None and self.newest >= moment, patience)
            earlier = [(stamp, text) for stamp, text in self.entries if stamp < moment]
            at = [(stamp, text) for stamp, text in self.entries if stamp == moment]
            later = [(stamp, text) for stamp, text in self.entries if stamp > moment]
            if at and earlier:
                return earlier[-1], at[0]
            first = earlier[-1] if earlier else (at[0] if at else None)
            second = at[0] if at and earlier else (later[0] if later else None)
            if first is None or second is None:
                return None
            return first, second

    def forget_before(self, moment):
        """Drop every message older than the latest one taken before 'moment', which 'last_step' may still want."""
        with self.condition:
            while len(self.entries) > 1 and self.entries[1][0] < moment:
                self.entries.popleft()


def joint_states(message, model, warnings):
    """The joints of one model in one of its joint-state messages, in PartCAD's terms.

    A joint with a map ('model.maps') is read through it by PartCAD's own
    'kinematics.reading': under the PartCAD joint's name, in PartCAD's units,
    from PartCAD's zero rather than the world's -- the export re-zeroed every
    joint at its start -- and as a number for a joint of one freedom and an
    object keyed by parameter for one of several (a screw's turn and the move
    that follows it, a universal joint's two turns). One without -- a world
    PartCAD did not write -- in the terms of the 'motion:' an interface
    declares, which is the vocabulary the MuJoCo plugin reports in too:

      revolute (and continuous), screw  'pos' in degrees, 'vel' in degrees per second
      prismatic                         'pos' in millimetres, 'vel' in mm/s

    and 'pos' is then the joint's own coordinate, zero where the world placed
    the child link.

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
                    reported,
                    model.shown,
                    (
                        " (its DART physics builds a 'continuous' joint as a fixed one; a 'revolute'"
                        " with no limits is the same joint, and moves)"
                        if declared == "continuous"
                        else ""
                    ),
                ),
            )
            continue
        entry = model.maps.get(name)
        if entry is not None:
            states[reported] = mapped_state(entry, joint)
            continue
        scale = DEG_PER_RAD if declared in TURNS else MM_PER_M
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
                % (model.joints[name][1], model.shown),
            )
    return states


def mapped_state(entry, joint):
    """One joint of a joint-state message, read through its joint map: 'kinematics.reading' of its coordinates.

    One coordinate per axis the map says the joint has -- 'axis1', and
    'axis2' for a universal joint -- in radians or metres as Gazebo states
    them, and nought where the message leaves a field out, as protobuf's text
    format does with every field at its default.
    """
    import kinematics

    count = max(1, len(entry["map"].get("coordinates") or []))
    axes = [joint.get(tag) if isinstance(joint.get(tag), dict) else {} for tag in ("axis1", "axis2")[:count]]
    positions = [float(axis.get("position") or 0.0) for axis in axes]
    velocities = [float(axis.get("velocity") or 0.0) for axis in axes]
    return kinematics.reading([entry], [positions], [velocities])


def _pose_data(data):
    """A pose the sidecar states as '[[w, x, y, z], [x, y, z]]' as a '(q, t)' pose."""
    return tuple(float(v) for v in data[0]), tuple(float(v) for v in data[1])


def _conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def ball_rotation(ball, parent, child):
    """A ball's rotation as Gazebo turned it, from where its two links are.

    'ball' is what the exporter recorded beside the ball's map: the joint's
    frame in each of its two links, which are one frame where the ball starts,
    Gazebo's zero of it. The rotation is the child's against the parent's --
    in the ball's own frame, which is the frame its map ('frame') says the
    engine's rotation is stated in. 'parent' and 'child' are the links' poses
    in any one frame.
    """
    import urdf_common

    on_parent = urdf_common.compose(parent, _pose_data(ball["in_parent"]))
    on_child = urdf_common.compose(child, _pose_data(ball["in_child"]))
    return urdf_common.normalize(urdf_common.quat_mul(_conjugate(on_parent[0]), on_child[0]))


def ball_rate(ball, paths, pair):
    """A ball's angular velocity at a reading, in rad/s about its own moving frame, or None.

    'pair' is the PosePublisher's messages of the step before the reading and
    of the reading's own ('LinkStream.last_step'), and 'paths' the model and
    link of the ball's two bodies (the sidecar's 'bodies'), which is what the
    publisher names a link by: '<model>::<link>'. The rate is the turn between
    the two rotations over the step -- what DART turned it by, at the speed it
    holds at the reading -- about the frame the first one leaves, which is how
    'kinematics.reading' takes it.
    """
    import urdf_common

    rotations = []
    for stamp, text in pair:
        poses = unique_poses(parse_message(text))
        names = ["::".join(paths.get(ball[side]) or ()) for side in ("parent", "child")]
        if not all(name in poses for name in names):
            return None
        rotations.append((stamp, ball_rotation(ball, poses[names[0]], poses[names[1]])))
    (first, earlier), (second, later) = rotations
    turn = _rotation_vector(urdf_common.quat_mul(_conjugate(earlier), later))
    return [v / (second - first) for v in turn]


def _rotation_vector(q):
    """The rotation 'q' as its axis times its angle, in radians: the smaller way round."""
    if q[0] < 0.0:
        q = tuple(-v for v in q)
    along = math.sqrt(sum(v * v for v in q[1:]))
    if along < 1e-15:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(along, q[0])
    return [v / along * angle for v in q[1:]]


def _in_world(poses, path):
    """A link's pose in the world out of one PosePublisher message: its model's, and its own in the model."""
    import urdf_common

    model, link = path[0], "::".join(path)
    if model not in poses or link not in poses:
        return None
    return urdf_common.compose(poses[model], poses[link])


def free_body_twists(entry, paths, links, moment, patience=PATIENCE):
    """A floating child's world velocity and its parent's, at 'moment', as 'kinematics.reading' takes them, or None.

    '(child twist, parent twist)', each '((vx, vy, vz), (wx, wy, wz))': the
    linear velocity of the body's frame origin in m/s and its angular velocity
    in rad/s, both in the world's axes. Gazebo publishes neither -- a link's
    world velocity is a component of the server's, which no topic carries --
    so each is what the last step moved the body by, out of the PosePublisher
    on its model ('LinkStream.last_step'), over the step: DART's semi-implicit
    Euler moves a body by the speed it ends a step with, so that is the speed
    the engine holds at the reading.
    """
    import urdf_common

    twists = []
    for body in (entry.get("body"), entry["map"].get("parent")):
        path = paths.get(body)
        stream = links.get("::".join(path[:-1])) if path else None
        pair = stream.last_step(moment, patience) if stream is not None else None
        if pair is None:
            return None
        poses = [(stamp, _in_world(unique_poses(parse_message(text)), path)) for stamp, text in pair]
        if any(pose is None for _stamp, pose in poses):
            return None
        (first, earlier), (second, later) = poses
        span = second - first
        linear = tuple((b - a) / span / MM_PER_M for a, b in zip(earlier[1], later[1]))
        angular = tuple(v / span for v in _rotation_vector(urdf_common.quat_mul(later[0], _conjugate(earlier[0]))))
        twists.append((linear, angular))
    return tuple(twists)


def _pose_of(body):
    """A body of a reading as a '(q, t)' pose, in millimetres."""
    return tuple(body["quat"]), tuple(body["pos"])


def _metres(body):
    """A body of a reading as a '(q, t)' pose in metres, which is how 'kinematics.reading' takes a free body's."""
    return tuple(body["quat"]), tuple(v / MM_PER_M for v in body["pos"])


# What every body's velocity is at t = 0: a world starts at rest, since SDFormat
# states no starting speed, and neither Gazebo nor PartCAD's exporter gives one.
# Which is what a reading at the start needs no step before it to tell.
AT_REST = (((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))


def posed_states(reading, names, links, warnings, patience=PATIENCE):
    """Every ball and every floating child at a reading's instant, read from where their bodies are.

    A floating joint's child is a model of its own, and its reading is its
    pose against the body it was connected to, which is what its map says to
    read it against ('kinematics.reading' of the two), and its rates the two
    bodies' world velocities ('free_body_twists'). A ball's rotation is its
    two links' turned against each other ('ball_rotation'), and its angular
    velocity is read from its model's PosePublisher over the last step
    ('ball_rate'). A ball or a floating child whose speed cannot be read is
    reported without its 'vel', and says so.
    """
    if not names.posed:
        return
    import kinematics

    bodies = reading["bodies"]
    for name, entry in names.posed.items():
        joint_map = entry["map"]
        if joint_map.get("kind") == FLOATING:
            child, parent = bodies.get(entry.get("body")), bodies.get(joint_map.get("parent"))
            if parent is None:
                note(
                    warnings,
                    "joint '%s' is not reported: the body it is read against, '%s', is not in the reading"
                    % (name, joint_map.get("parent")),
                )
                continue
            if child is None:
                note(
                    warnings,
                    "joint '%s' is not reported: its own body, '%s', is not in the reading" % (name, entry.get("body")),
                )
                continue
            twists = (
                AT_REST
                if reading["time"] <= 0.0
                else free_body_twists(entry, names.bodies, links, reading["time"], patience)
            )
            if twists is None:
                note(
                    warnings,
                    "the floating joint '%s' is reported without its 'vel': Gazebo published no poses of its two "
                    "bodies either side of a reading to tell their speeds by" % name,
                )
            reading["joints"][name] = kinematics.reading(
                [entry], [(_metres(child), _metres(parent))], None if twists is None else [twists]
            )
            continue
        ball = entry.get("ball") or {}
        parent, child = bodies.get(ball.get("parent")), bodies.get(ball.get("child"))
        if child is None or parent is None:
            note(warnings, "the ball '%s' is not reported: its two bodies are not both in the reading" % name)
            continue
        rotation = ball_rotation(ball, _pose_of(parent), _pose_of(child))
        path = entry.get("path") or []
        stream = links.get("::".join(str(part) for part in path[:-1]))
        if reading["time"] <= 0.0:
            rate = [0.0, 0.0, 0.0]
        else:
            pair = stream.last_step(reading["time"], patience) if stream is not None else None
            rate = ball_rate(ball, names.bodies, pair) if pair is not None else None
        if rate is None:
            note(
                warnings,
                "the ball '%s' is reported without its 'vel': Gazebo published no poses of its links either side "
                "of a reading to tell its speed by" % name,
            )
        reading["joints"][name] = kinematics.reading([entry], [rotation], None if rate is None else [rate])


def fill(reading, streams, warnings, patience=PATIENCE, names=AS_WRITTEN, late=None):
    """State in 'reading' where the joints of every stream were at its instant.

    'patience' is how long to wait for a stream that has not got that far yet
    (see 'JointStream.at'); nothing is worth waiting for once the server that
    publishes them has gone. A ball and a floating child are read from the
    reading's own bodies, and their speeds from their models' link streams
    ('posed_states').

    Returns whether every stream had a state of the reading's own instant or
    an earlier one to give: False where one began after it -- its subscriber
    came up a step or two after the server's first -- and gave a later
    step's, which puts those joints a step from where the bodies are. Each
    such stream is put in 'late', a dict, if one is given: its model's name,
    and the time of the state it had to give instead.
    """
    links = {}
    in_step = True
    for stream in streams:
        if isinstance(stream.model, Wrench):
            # A joint's load, which 'fill_loads' reads.
            continue
        if stream.model.links:
            links[stream.model.scoped] = stream
            continue
        text = stream.at(reading["time"], patience)
        if text is None:
            note(
                warnings,
                "the joints of model '%s' are not reported: Gazebo published no joint states for it"
                % stream.model.shown,
            )
            continue
        stamp = message_time(text)
        if stamp > reading["time"]:
            in_step = False
            if late is not None:
                late[stream.model.shown] = stamp
        reading["joints"].update(joint_states(parse_message(text), stream.model, warnings))
    posed_states(reading, names, links, warnings, patience)
    return in_step


def process(path, request):
    started = time.monotonic()
    (server, server_args, topic_command), environment = locate_gazebo()
    # What the server and every subscriber run in: this run's own partition.
    # Not what '_version' runs in, which talks to nobody.
    run_env, partition = run_environment(environment)

    scene_file = request["scene_file"]
    duration = float(request.get("duration") or 10.0)
    samples = int(request.get("samples") or 0)
    # The run's budget: PartCAD's, handed to every run, or this script's own.
    deadline = Deadline(positive(request.get("timeout")) or DEFAULT_TIMEOUT, started)
    startup = positive(request.get("startup_timeout")) or STARTUP_TIMEOUT
    world = request.get("world_name") or world_name(scene_file)
    topic = "/world/%s/pose/info" % world
    # PartCAD's names for what the world calls everything, out of the sidecar
    # beside it, and what the world itself is called there: 'world' for the
    # world PartCAD names that, which SDFormat cannot.
    names = Names.beside(scene_file)
    shown = names.world(world)
    warnings = []
    # Whether every reading says what acts on each body (see 'fill_loads'):
    # yes, unless the 'simulate:' asked for nothing but poses -- and only for a
    # world PartCAD exported, whose sidecar places each body in the world.
    loads = request.get("loads", True) is not False
    if loads and not names.bodies:
        note(warnings, "no loads are reported: they are read for a world PartCAD exported, with its names beside it")
        loads = False
    # The configuration the server reads the systems of a world that names none
    # from -- the physics, the scene broadcaster -- read where it is installed
    # wherever Gazebo would otherwise copy it into the home first, which a
    # server started beside this one could read half-written ('server_config').
    configured = server_config(run_env, server, warnings)
    if configured is not None:
        run_env[SERVER_CONFIG] = configured
    # Before the server is started, because they are written into the file
    # the server loads: see 'world_gravity' and 'world_physics'.
    gravity = world_gravity(scene_file, request.get("gravity"))
    timestep = request.get("timestep")
    if timestep is not None and positive(timestep) is None:
        raise ValueError("'timestep' is Gazebo's integration step, in seconds and more than nought; got %r" % timestep)
    step = world_physics(scene_file, None if timestep is None else float(timestep))
    factor = pace(samples, duration, step)
    if factor is not None:
        world_physics(scene_file, real_time_factor=factor)
    if samples and duration / (samples + 1) < step:
        note(
            warnings,
            "%d samples were asked for over %g s, and Gazebo steps every %g s: it has at most %d to give"
            % (samples, duration, step, int(duration / step)),
        )

    # Paused: 'unpause' sets it running once 'before' is read, at t = 0.
    command = [server] + list(server_args) + ["-s", "-v", "1"]

    programs = Programs()
    simulator = None
    said = Lines(None)
    # The first message and the last, kept whole for the snapshots: a reading
    # keyed by name is enough for a validation and not for a picture, which
    # has to know which link of which model each pose belongs to.
    first = last = None
    streams = []
    run_file = scene_file
    try:

        def subscribe():
            """A 'gz topic -e' for the poses, in this run's partition."""
            return programs.start(
                [server, topic_command, "-e", "-t", topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                env=run_env,
            )

        # The subscriber first. It is the thing that has to be listening before
        # the server starts publishing; the other way round loses the opening
        # reading, which is the one 'before' is.
        watcher = subscribe()
        poses = Messages(watcher.stdout)
        # How many subscribers the poses have had (see 'RESUBSCRIBE').
        subscriptions = 1

        # The world as handed over, or a copy of it that publishes its joints.
        # Made inside the 'try' whose 'finally' removes it, so that nothing
        # that can fail lies between the two and leaves it behind.
        # Every step, which SDFormat spells as a rate of nought; see
        # 'EVERY_STEP'.
        run_file, joint_models = prepare_world(scene_file, warnings, names, EVERY_STEP if loads else None)
        command.append(run_file)
        # Each body's mass and centre of mass, out of the world, for its weight.
        inertials = link_inertials(scene_file) if loads else {}
        if loads:
            note(
                warnings,
                "a body's contacts are not among its loads, and neither is its acceleration: this plugin reads"
                " neither from Gazebo, so a body's loads need not add up to anything",
            )

        # A subscriber for the joints of each model that has any -- and for
        # the links of each that has a ball, whose speed is read from them --
        # listening before the server starts for the reason the one above is.
        for model in joint_models:
            joint_watcher = programs.start(
                [server, topic_command, "-e", "-t", model.topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                env=run_env,
            )
            streams.append((LinkStream if model.links else JointStream)(model))
            threading.Thread(target=streams[-1].follow, args=(joint_watcher.stdout,), daemon=True).start()
        wrenches = [stream for stream in streams if isinstance(stream.model, Wrench)]

        simulator = programs.start(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, env=run_env)
        # Read as it comes, so that a server with a lot to say never blocks on
        # a full pipe, and kept, so that a server that dies is quoted.
        said = Lines(simulator.stderr)

        def patience():
            """How long a joint reading may wait: not past the deadline, and not at all for a dead server."""
            return 0.0 if simulator.poll() is not None else min(PATIENCE, deadline.remaining())

        before = None
        after = None
        trace = []
        every = duration / (samples + 1) if samples > 0 else None
        next_sample = every
        waiting_since = time.monotonic()
        first_by = min(deadline.at, waiting_since + startup)
        # Pose messages that came but named no body this run reads: with a
        # sidecar, ones in which no body's model and link appear once each.
        unread = 0
        # The instant of the latest pose message that did name them, and each
        # joint stream that had only a later state than that to give: what a
        # run with no first reading says it had.
        seen = None
        behind = {}
        resubscribe_at = waiting_since + RESUBSCRIBE

        def out_of_time():
            """The failure a run that has waited as long as it may fails with, or None."""
            if before is None and said.says(NO_SYSTEMS):
                # Not worth waiting for: a server with no systems publishes
                # no pose however long it is given.
                return no_systems(shown, run_env, configured is not None, said)
            if before is None and unread and names.bodies and time.monotonic() >= first_by:
                return (
                    "Gazebo published %d pose messages for world '%s' within %.0f s of being started, and none"
                    " named the model and link of a body the names file beside '%s' lists once each, so there is"
                    " nothing to report. That file does not belong to the world that was loaded, or the world"
                    " names a body's model or link twice; export the world again."
                    % (unread, shown, time.monotonic() - waiting_since, scene_file)
                )
            if before is None and time.monotonic() >= first_by:
                return no_first_reading(
                    shown,
                    topic,
                    partition,
                    time.monotonic() - waiting_since,
                    poses.count - unread,
                    seen,
                    streams,
                    behind,
                    said.tail(0),
                    subscriptions,
                )
            if deadline.passed():
                return (
                    "Gazebo did not reach %.3g s of simulated time within %.0f s of waiting (the run's budget is"
                    " %.0f s). %s Raise 'timeout' if this machine is simply slow."
                    % (
                        duration,
                        deadline.elapsed(),
                        deadline.budget,
                        progress(shown, before, after, poses.count, duration),
                    )
                )
            return None

        while True:
            # On every turn, whatever arrives: a stream of messages that never
            # says anything this run reads is as good as no stream at all.
            late = out_of_time()
            if late:
                raise Exception(late)
            text = poses.next(min(POLL, max(0.01, deadline.remaining())))
            if text is None or text is ENDED:
                # Nothing new: the server is either working, or never will be.
                if simulator.poll() is not None:
                    raise Exception(server_died(simulator, said, shown, before, after, poses.count, duration))
                if text is ENDED:
                    raise Exception(
                        "Gazebo's pose subscriber ('%s %s -e') stopped (%s) while the server was still running. %s"
                        % (
                            server,
                            topic_command,
                            ended_how(watcher.poll()),
                            progress(shown, before, after, poses.count, duration),
                        )
                    )
                if before is None and not poses.count and time.monotonic() >= resubscribe_at:
                    # Not a word from a server that is still running: a
                    # subscriber it may never have registered (see
                    # 'RESUBSCRIBE'). The new one first and then the old one
                    # stopped, so that nothing goes unheard between the two.
                    stale, watcher = watcher, subscribe()
                    poses.follow(watcher.stdout)
                    _signal_group(stale, signal.SIGTERM)
                    resubscribe_at = time.monotonic() + RESUBSCRIBE * 2**subscriptions
                    subscriptions += 1
                continue
            message = parse_message(text)
            reading = snapshot(message, names)
            if not reading["bodies"]:
                unread += 1
                continue
            if before is None:
                seen = reading["time"]
                behind.clear()
                if not fill(reading, streams, warnings, patience(), names, behind):
                    # A joint stream that began after this instant has only a
                    # later step to give, so the joints would be a step from
                    # where the bodies are: 'before' is the first instant
                    # every stream has reached, which the next message is.
                    continue
                if loads:
                    fill_loads(reading, wrenches, inertials, gravity, warnings, 0.0, names, paused=True)
                if subscriptions > 1:
                    note(
                        warnings,
                        "Gazebo published no pose to this run within %.0f s of its server being started, and did"
                        " once the poses were subscribed to again (%d subscribers in all). A server slow to load the"
                        " world looks like that, and so does one that never registered the first subscriber, which"
                        " gz-transport does once, in one UDP datagram that a busy machine can lose"
                        % (time.monotonic() - waiting_since, subscriptions),
                    )
                before = reading
                first = message
                running, answer = unpause(server, world, run_env, min(10.0, max(1.0, deadline.remaining())))
                if not running:
                    raise Exception(
                        "Gazebo's server would not set world '%s' running: it was started paused, so that the first"
                        " reading is the world as loaded, and its control service ('%s service -s /world/%s/control')"
                        " answered: %s" % (shown, server, world, answer or "nothing")
                    )
            after = reading
            last = message
            for stream in streams:
                stream.forget_before(reading["time"])
            if next_sample is not None and reading["time"] >= next_sample and len(trace) < samples:
                fill(reading, streams, warnings, patience(), names)
                if loads:
                    fill_loads(reading, wrenches, inertials, gravity, warnings, patience(), names)
                trace.append(reading)
                # Each sample is the first pose message at or past its instant
                # that no sample before it took, at its own time: a step late,
                # where a message was not published at that very step (the
                # first one after the world is set running often is not), and
                # never one message standing for two.
                next_sample += every
            if reading["time"] >= duration:
                break

        # Here rather than as the loop takes it, because which reading is the
        # last is only known once the loop is over -- and here rather than
        # after it, because the server is still up and still publishing, and
        # the joint states of that instant may still be on their way. Unless it
        # is not, in which case what has arrived is all there will be.
        fill(after, streams, warnings, patience(), names)
        if loads and after is not before:
            fill_loads(after, wrenches, inertials, gravity, warnings, patience(), names)
        if samples and len(trace) < samples:
            note(
                warnings,
                "%d samples were asked for and %d came back: Gazebo published a pose message for no more of "
                "the instants they were due at" % (samples, len(trace)),
            )
    finally:
        programs.stop()
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
        "version": _version([server] + list(server_args), environment, min(30.0, max(1.0, deadline.remaining()))),
        "world": shown,
        "duration": duration,
        "reached": after["time"],
        # What the run was under, out of the file Gazebo loaded. It used to be
        # whatever the request said, which was this package's own default and
        # not a number Gazebo was ever given.
        "gravity": gravity,
        "units": "mm",
    }
    if trace:
        result["samples"] = trace
    if loads and names.parts:
        # Which body each part is welded into and where it sits in it, out of
        # the sidecar: what turns a body's loads into a part's.
        result["parts"] = names.parts
    drawn = take_snapshots(path, request.get("snapshot"), scene_file, first, last, warnings)
    if drawn:
        result["snapshots"] = drawn
    if warnings:
        result["warnings"] = warnings
    return result


def unpause(server, world, environment, timeout):
    """Set the world in the server running, which it was started without; '(it did, what it said)'.

    The server is started paused ('-s' without '-r'), so that the first
    reading is the world as it was loaded -- at t = 0, every joint at its
    start, every body where the scene put it -- and only once that is read is
    it set running, by the world's own control service. The generation that
    calls itself 'ign' names its messages 'ignition.msgs'.
    """
    prefix = "ignition.msgs" if os.path.basename(server).startswith("ign") else "gz.msgs"
    command = [
        server,
        "service",
        "-s",
        "/world/%s/control" % world,
        "--reqtype",
        "%s.WorldControl" % prefix,
        "--reptype",
        "%s.Boolean" % prefix,
        "--timeout",
        "%d" % int(max(1.0, timeout) * 1000),
        "--req",
        "pause: false",
    ]
    try:
        answer = subprocess.run(
            command, capture_output=True, text=True, timeout=max(1.0, timeout) + 5.0, env=environment
        )
    except (OSError, subprocess.SubprocessError) as e:
        return False, "%s: %s" % (type(e).__name__, e)
    said = ((answer.stdout or "") + (answer.stderr or "")).strip()
    # Only an answer of 'true' is a world set running: a call that timed out
    # prints "Service call timed out" and exits with status nought all the
    # same (Harmonic's 'gz service'), and says nothing of what happened.
    return answer.returncode == 0 and re.search(r"\bdata:\s*true\b", said) is not None, said


def _version(command, environment=None, timeout=30.0):
    """What this Gazebo calls itself, for the record. Never a reason to fail.

    Asked of the server subcommand ('gz sim --versions'): 'gz --versions' on its
    own answers with the tool's usage text rather than a version. Within what is
    left of the run's budget, at most 'timeout' seconds.
    """
    try:
        answer = subprocess.run(
            list(command) + ["--versions"], capture_output=True, text=True, timeout=timeout, env=environment
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

    print(
        json.dumps(
            process(None, {"scene_file": sys.argv[1], "duration": float(sys.argv[2] if len(sys.argv) > 2 else 10)}),
            indent=2,
        )
    )
