#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What stops a Gazebo run that will not end, proven with a Gazebo that misbehaves.

The Gazebo here is a fake: a 'gz' that is a few lines of Python, found through
'PC_GZ' exactly as an installation the name does not give away would be. As
'gz topic -e' it prints pose messages in the text format the real one does, or
nothing at all; as 'gz sim' it runs until it is stopped, starting a process of
its own as a server would, or dies -- at once, or after its subscriber has heard
from it. Every program it is run as writes its process id down, so that each
test can check that nothing it started outlived the run.

What each one proves:

* a server that dies before its first pose fails the run at once, quoting it --
  where the run used to wait for that pose for ever;
* a server that hangs while it starts, publishes where nobody is listening, or
  publishes messages that carry nothing, fails the run after 'startup_timeout',
  naming the partition;
* a server that publishes and then dies fails the run saying how far it got;
* a run that will not reach its 'duration' within its budget stops itself;
* a subscriber that never hears a joint state costs a warning and a bounded
  wait, not the run;
* and on every one of those ways out, every program is stopped -- the server,
  what it started, every subscriber -- and the world's copy is removed.

Each test takes a second or two. POSIX only: the fake is a script run by its
'#!' line, and what is checked is process groups.
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# PartCAD's sandbox contract, which 'gazebo_common' is written against and which
# is on 'sys.path' for anything PartCAD runs in a sandbox (see 'test_world.py'):
# from the installed 'partcad', or from PYTHONPATH where there is none -- this
# runs in the simulation's image as well, which carries no PartCAD.
try:
    import partcad as pc

    sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))
except ImportError:
    pass

import simulate_gazebo as sim  # noqa: E402

pytestmark = pytest.mark.skipif(os.name == "nt", reason="the fake Gazebo is a '#!' script; Windows runs none")

FAKE_GZ = r"""#!%(python)s
import os
import resource
import signal
import subprocess
import sys
import time

mode = os.environ.get("FAKE_GZ_MODE", "ok")
args = sys.argv[1:]


def remember(pid):
    with open(os.path.join(os.environ["FAKE_GZ_PIDS"], str(pid)), "w") as f:
        f.write(" ".join(args))


def pose(t, z):
    sys.stdout.write(
        "header {\n  stamp {\n    sec: %%d\n    nsec: %%d\n  }\n}\n"
        "pose {\n  name: \"block\"\n  id: 8\n  position {\n    x: 0\n    y: 0\n    z: %%r\n  }\n"
        "  orientation {\n    w: 1\n  }\n}\n" %% (int(t), int(round((t %% 1) * 1e9)), z)
    )
    sys.stdout.flush()


if args[:1] == ["sim"] and "--versions" in args:
    print("Gazebo Sim, version 8.9.0 (fake)")
    sys.exit(0)
remember(os.getpid())
if args[:1] == ["sim"]:
    if mode == "die":
        sys.stderr.write("[Err] [Server.cc:123] Unable to find or download file\n")
        sys.exit(3)
    # A server of its own, as 'gz sim' may start one.
    remember(subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"]).pid)
    if mode == "publish-then-die":
        time.sleep(0.6)
        sys.stderr.write("[Err] [Physics] the solver failed\n")
        sys.stderr.flush()
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))  # a crash, without a core file
        os.kill(os.getpid(), signal.SIGSEGV)
    time.sleep(600)
elif args[:1] == ["topic"]:
    topic = args[args.index("-t") + 1]
    if topic.endswith("/pose/info"):
        if mode in ("ok", "silent-joints"):
            for step in range(41):
                pose(step * 0.025, 1.0 - step * 0.01)
                time.sleep(0.005)
        elif mode == "publish-then-die":
            for step in range(3):
                pose(step * 0.01, 1.0)
        elif mode == "empty":
            # A world with nothing in it that moves, as far as the poses say.
            for step in range(100000):
                sys.stdout.write("header {\n  stamp {\n    sec: 0\n    nsec: %%d\n  }\n}\n" %% step)
                sys.stdout.flush()
                time.sleep(0.005)
        elif mode == "slow":
            for step in range(100000):
                pose(step * 0.001, 1.0)
                time.sleep(0.05)
    # Then nothing, for ever: what a subscriber does with no publisher.
    time.sleep(600)
"""

WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
  <model name="block"><pose>0 0 1 0 0 0</pose><link name="body">
    <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
  </link></model>
</world></sdf>
"""

JOINTED = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
  <model name="block"><pose>0 0 1 0 0 0</pose>
    <link name="body"/><link name="arm"/>
    <joint name="swing" type="revolute"><parent>body</parent><child>arm</child><axis><xyz>0 1 0</xyz></axis></joint>
  </model>
</world></sdf>
"""


@pytest.fixture
def gazebo(tmp_path, monkeypatch):
    """The fake 'gz', and a way to tell it how to misbehave."""
    binary = tmp_path / "bin" / "gz"
    binary.parent.mkdir()
    binary.write_text(FAKE_GZ % {"python": sys.executable}, encoding="utf-8")
    binary.chmod(0o755)
    pids = tmp_path / "pids"
    pids.mkdir()
    monkeypatch.setenv("PC_GZ", str(binary))
    monkeypatch.setenv("FAKE_GZ_PIDS", str(pids))
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    monkeypatch.delenv("IGN_PARTITION", raising=False)

    def behave(mode):
        monkeypatch.setenv("FAKE_GZ_MODE", mode)

    behave.pids = pids
    return behave


def run(tmp_path, world=WORLD, **request):
    run_dir = tmp_path / "run"
    run_dir.mkdir(exist_ok=True)
    (run_dir / "scene.world").write_text(world, encoding="utf-8")
    request = dict({"scene_file": str(run_dir / "scene.world"), "duration": 0.5, "timeout": 30}, **request)
    started = time.monotonic()
    try:
        return sim.process(str(run_dir), request), None, time.monotonic() - started
    except Exception as e:  # noqa: BLE001 - what the run failed with is what is under test
        return None, e, time.monotonic() - started


def alive(pid):
    """Whether 'pid' is still running: a zombie nobody has reaped yet is not."""
    try:
        with open("/proc/%d/stat" % pid, "rb") as f:
            stat = f.read().decode("utf-8", "replace")
        return stat[stat.rfind(")") + 2 :].split()[0] != "Z"
    except FileNotFoundError:
        return False
    except OSError:
        pass
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def assert_nothing_outlived_the_run(gazebo):
    started = [int(name) for name in os.listdir(gazebo.pids)]
    assert started, "the fake Gazebo was never run"
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and any(alive(pid) for pid in started):
        time.sleep(0.05)
    assert [pid for pid in started if alive(pid)] == []


def test_a_run_of_a_gazebo_that_behaves_reports_and_leaves_nothing_running(gazebo, tmp_path):
    """The fake itself, proven against the plugin's ordinary run."""
    gazebo("ok")

    result, error, _ = run(tmp_path)

    assert error is None
    assert result["reached"] >= 0.5
    assert result["version"] == "Gazebo Sim, version 8.9.0 (fake)"
    assert_nothing_outlived_the_run(gazebo)


def test_a_server_that_dies_before_its_first_pose_fails_the_run_at_once_quoting_it(gazebo, tmp_path):
    """The run used to wait on the subscriber for ever: a dead server says nothing on its topic."""
    gazebo("die")

    _, error, took = run(tmp_path, world=JOINTED)

    assert error is not None and took < 5
    message = str(error)
    assert "published no poses for world 'w'" in message
    assert "exit status 3" in message
    assert "Unable to find or download file" in message
    assert_nothing_outlived_the_run(gazebo)
    assert sorted(os.listdir(tmp_path / "run")) == ["scene.world"]  # the copy with the joint publisher too


def test_a_server_that_hangs_while_it_starts_fails_after_the_startup_timeout(gazebo, tmp_path):
    gazebo("hang")

    _, error, took = run(tmp_path, startup_timeout=1.0)

    assert error is not None and 1.0 <= took < 5
    message = str(error)
    assert "within 1 s of being started" in message
    assert "partition 'partcad-" in message
    assert "startup_timeout" in message
    assert_nothing_outlived_the_run(gazebo)


def test_a_stream_of_messages_that_say_nothing_is_no_first_pose(gazebo, tmp_path):
    """Every turn of the wait checks the clock, not only the turns that read a pose."""
    gazebo("empty")

    _, error, took = run(tmp_path, startup_timeout=1.0)

    assert error is not None and 1.0 <= took < 5
    assert "within 1 s of being started" in str(error)
    assert_nothing_outlived_the_run(gazebo)


def test_a_server_that_publishes_and_then_dies_fails_saying_how_far_it_got(gazebo, tmp_path):
    gazebo("publish-then-die")

    _, error, took = run(tmp_path)

    assert error is not None and took < 5
    message = str(error)
    assert "killed by signal 11 (SIGSEGV)" in message
    # Two of the three: a message is whole only once the next one starts, and
    # the third never had a next.
    assert "2 pose messages of world 'w' had been received, the last at t = 0.01 s" in message
    assert "the solver failed" in message
    assert_nothing_outlived_the_run(gazebo)


def test_a_run_that_will_not_reach_its_duration_within_its_budget_stops_itself(gazebo, tmp_path):
    """A budget of three seconds, of which the run uses two and keeps one to say so."""
    gazebo("slow")

    _, error, took = run(tmp_path, duration=10.0, timeout=3.0)

    assert error is not None and 2.0 <= took < 3.0
    message = str(error)
    assert "did not reach 10 s" in message
    assert "the run's budget is 3 s" in message
    assert "pose messages of world 'w' had been received" in message
    assert_nothing_outlived_the_run(gazebo)


def test_a_joint_subscriber_that_never_hears_costs_a_warning_and_a_bounded_wait(gazebo, tmp_path, monkeypatch):
    gazebo("silent-joints")
    monkeypatch.setattr(sim, "PATIENCE", 0.5)

    result, error, took = run(tmp_path, world=JOINTED)

    assert error is None and took < 5
    assert any("published no joint states" in warning for warning in result["warnings"])
    assert_nothing_outlived_the_run(gazebo)
    assert sorted(os.listdir(tmp_path / "run")) == ["scene.world"]


def test_the_budget_leaves_a_margin_for_saying_so():
    assert sim.Deadline(300, started=0.0).at == 270.0
    assert sim.Deadline(3600, started=0.0).at == 3570.0
    assert sim.Deadline(3, started=0.0).at == 2.0
    assert sim.Deadline(0.5, started=0.0).at == 0.25


@pytest.mark.parametrize("value", [0, -1, "soon", True, float("nan"), None])
def test_a_budget_that_is_not_one_is_none(value):
    assert sim.positive(value) is None


#
# In a real Gazebo
#
# Skipped without one, which CI's runners do not have; run it in the image the
# simulation uses, with PartCAD's sandbox modules on the path:
#
#     docker run --rm -v "$PWD:/w" -v "<partcad>/src/partcad/wrappers:/wrappers:ro" -w /w \
#         -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONPATH=/wrappers --entrypoint bash \
#         ghcr.io/partcad/partcad-sim-gazebo:latest -c \
#         'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest numpy \
#          && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_safety.py'
#

REFUSED = """<?xml version="1.0"?>
<sdf version="1.9"><world name="refused"><model name="m"><link name="l"/><link name="l"/></model></world></sdf>
"""


@pytest.fixture
def real_gazebo(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    try:
        sim.locate_gazebo()
    except sim.GazeboMissing:
        pytest.skip("no Gazebo here; see above for running it in the simulation's image")


def test_in_gazebo_a_world_the_server_refuses_fails_at_once_quoting_the_server(real_gazebo, tmp_path):
    """The known hang: Harmonic refuses a world with two links of one name, says
    so, and exits -- with status 0 -- before it publishes anything, and the
    subscriber waited for its first pose for ever."""
    _, error, took = run(tmp_path, world=REFUSED, timeout=60)

    assert error is not None and took < 30
    message = str(error)
    assert "published no poses for world 'refused'" in message
    assert "link with name[l] already exists" in message
    assert "\x1b[" not in message
