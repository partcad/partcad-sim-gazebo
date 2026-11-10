#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""That two runs at once do not read each other (see 'run_environment' in 'simulate_gazebo.py').

Gazebo's topics are named after the world, and two runs of one simulation are
two worlds of one name. In one transport partition each run's subscriber hears
both servers, and nothing fails: the readings interleave, and a validation is
handed poses that were not its own. So every run gets a partition of its own.

The environment that does it is checked without a Gazebo. The last test runs two
same-named worlds side by side in a real one and checks that each reports only
itself; it is skipped without a Gazebo, and run in the image the simulation
uses, as CI runs it (see 'Tests' in the README):

    container/test.sh ghcr.io/partcad/partcad-sim-gazebo:latest test_partition.py
"""

import concurrent.futures
import math
import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# PartCAD's sandbox contract, which a run imports ('gazebo_common' is written
# against it), from the installed 'partcad' or from PYTHONPATH where there is
# none -- as 'test_safety.py' finds it.
try:
    import partcad as pc

    sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))
except ImportError:
    pass

import simulate_gazebo as sim  # noqa: E402

#
# The environment a run starts its programs in
#


def test_a_run_has_a_partition_of_its_own_in_both_spellings():
    """'GZ_' for the current Gazebo and 'IGN_' for the generation before, alike."""
    environment, partition = sim.run_environment({"PATH": "/bin"}, token="abc")

    assert partition == "partcad-abc"
    assert environment["GZ_PARTITION"] == environment["IGN_PARTITION"] == "partcad-abc"
    assert environment["PATH"] == "/bin"


def test_no_two_runs_share_one():
    """Not even two runs of one simulation in one process: the token is random,
    rather than the run directory both share or a process id two sandbox
    containers may both have."""
    partitions = {sim.run_environment({})[1] for _ in range(50)}

    assert len(partitions) == 50


def test_a_partition_the_user_set_is_kept_as_the_prefix_of_the_runs_own():
    """Taken as it is, it would put every run back into one partition."""
    environment, partition = sim.run_environment({"GZ_PARTITION": "lab:me"}, token="abc")

    assert partition == "lab:me:partcad-abc"
    assert environment["GZ_PARTITION"] == environment["IGN_PARTITION"] == partition


def test_the_older_spelling_is_kept_too_when_it_is_the_one_the_user_set():
    _environment, partition = sim.run_environment({"IGN_PARTITION": "fortress"}, token="abc")

    assert partition == "fortress:partcad-abc"


def test_an_empty_partition_is_none_at_all():
    """Which is also what gz-transport makes of one."""
    _environment, partition = sim.run_environment({"GZ_PARTITION": ""}, token="abc")

    assert partition == "partcad-abc"


def test_the_environment_started_from_is_not_changed():
    """It may be this process's own, or the one a ROS setup script left."""
    given = {"GZ_PARTITION": "lab", "PATH": "/ros/bin"}

    sim.run_environment(given, token="abc")

    assert given == {"GZ_PARTITION": "lab", "PATH": "/ros/bin"}


def test_with_no_environment_given_it_is_this_processs_own_plus_the_partition(monkeypatch):
    monkeypatch.setenv("PC_PARTITION_TEST", "kept")
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    monkeypatch.delenv("IGN_PARTITION", raising=False)

    environment, partition = sim.run_environment(token="abc")

    assert environment["PC_PARTITION_TEST"] == "kept"
    assert partition == "partcad-abc"
    assert "GZ_PARTITION" not in os.environ


def test_a_partition_is_a_name_gz_transport_accepts():
    """No '@', '~' or whitespace, and no '//': gz-transport refuses those in a
    partition, which is validated the way a topic name is."""
    _environment, partition = sim.run_environment({"GZ_PARTITION": "host:user"})

    assert not any(c in partition for c in "@~ \t\n")
    assert "//" not in partition


#
# The discovery ports a run's programs are started with
#


def test_a_run_has_discovery_ports_of_its_own_in_both_spellings():
    environment, _partition = sim.run_environment({"PATH": "/bin"}, token="abc")

    topics, services = int(environment["GZ_DISCOVERY_MSG_PORT"]), int(environment["GZ_DISCOVERY_SRV_PORT"])
    assert environment["IGN_DISCOVERY_MSG_PORT"] == str(topics)
    assert environment["IGN_DISCOVERY_SRV_PORT"] == str(services)
    assert services == topics + 1
    assert sim.DISCOVERY_PORTS[0] <= topics < services <= sim.DISCOVERY_PORTS[1]
    # Not gz-transport's own, 10317 and 10318, which every other process uses.
    assert topics > 10318


def test_runs_have_ports_of_their_own():
    """2268 pairs to pick from: 50 runs share some pair now and then, never most of them."""
    pairs = {sim.run_environment({})[0]["GZ_DISCOVERY_MSG_PORT"] for _ in range(50)}
    assert len(pairs) >= 45


def test_a_pair_something_holds_is_passed_over():
    """A concurrent run's server holds its pair as gz-transport binds it, with 'SO_REUSEADDR'."""
    first = sim.discovery_ports({}, "abc")
    held = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    held.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        held.bind(("", first[1]))
        assert sim.discovery_ports({}, "abc") not in (None, first)
    finally:
        held.close()


def test_with_every_pair_held_the_ports_are_left_as_they_are(monkeypatch):
    """What every run did before it had ports of its own: its partition still keeps it apart."""
    monkeypatch.setattr(sim, "_unbound", lambda port: False)

    environment, partition = sim.run_environment({}, token="abc")

    assert not any(name in environment for names in sim.DISCOVERY_PORT_VARIABLES for name in names)
    assert environment["GZ_PARTITION"] == partition == "partcad-abc"


@pytest.mark.parametrize("name", sim.DISCOVERY_VARIABLES)
def test_discovery_the_user_set_up_is_left_as_it_is(name):
    """Theirs to have set, whichever part of it: the ports, the group, a relay to another machine."""
    environment, _partition = sim.run_environment({name: "kept"}, token="abc")

    assert environment[name] == "kept"
    assert all(environment.get(other) in (None, "kept") for names in sim.DISCOVERY_PORT_VARIABLES for other in names)


def test_a_failure_says_where_the_runs_discovery_was():
    environment, _partition = sim.run_environment({}, token="abc")

    said = sim.discovery_said(environment, "/usr/bin/gz")

    assert said == " (discovery on ports %s and %s)" % (
        environment["GZ_DISCOVERY_MSG_PORT"],
        environment["GZ_DISCOVERY_SRV_PORT"],
    )
    assert sim.discovery_said(environment, "/usr/bin/ign") == said
    assert sim.discovery_said({}, "/usr/bin/gz") == ""


@pytest.mark.parametrize(
    "spelling, server, said",
    [
        ("IGN", "/usr/bin/ign", " (discovery on ports 11317 and 11318)"),
        ("GZ", "/usr/bin/gz", " (discovery on ports 11317 and 11318)"),
        # The other generation's spelling: this run is on its defaults, not on these.
        ("IGN", "/usr/bin/gz", ""),
        ("GZ", "/usr/bin/ign", ""),
    ],
)
def test_a_failure_names_a_users_ports_only_for_the_generation_that_reads_them(spelling, server, said):
    environment, _partition = sim.run_environment(
        {spelling + "_DISCOVERY_MSG_PORT": "11317", spelling + "_DISCOVERY_SRV_PORT": "11318"}, token="abc"
    )

    assert sim.discovery_said(environment, server) == said


def test_the_server_and_every_subscriber_are_started_in_it(monkeypatch, tmp_path):
    """All of them, or the ones left out hear nothing at all; checked by
    starting nothing and recording what would have been started."""
    scene = tmp_path / "scene.world"
    scene.write_text(
        '<sdf version="1.9"><world name="w"><model name="rig"><link name="a"/><link name="b"/>'
        '<joint name="j" type="revolute"><parent>a</parent><child>b</child></joint></model></world></sdf>',
        encoding="utf-8",
    )
    monkeypatch.setattr(sim, "locate_gazebo", lambda: (("gz", ["sim"], "topic"), {"PATH": "/bin"}))
    started = []

    class Recorded:
        def __init__(self, command, env=None, **_kwargs):
            started.append((command, env))
            self.stdout = iter(())
            self.stderr = None

        def poll(self):
            return 0

    monkeypatch.setattr(sim.subprocess, "Popen", Recorded)

    with pytest.raises(Exception, match="published no poses"):
        sim.process(str(tmp_path), {"scene_file": str(scene), "duration": 0.1})

    # The pose subscriber, the joint subscriber and the server, in that order.
    assert [command[:2] for command, _env in started] == [["gz", "topic"], ["gz", "topic"], ["gz", "sim"]]
    partitions = {env["GZ_PARTITION"] for _command, env in started}
    assert len(partitions) == 1 and partitions.pop().startswith("partcad-")
    ports = {(env.get("GZ_DISCOVERY_MSG_PORT"), env.get("GZ_DISCOVERY_SRV_PORT")) for _command, env in started}
    assert len(ports) == 1
    assert all(env["PATH"] == "/bin" for _command, env in started)


#
# In a real Gazebo
#

G = 9.81

# One world name for both runs, which is the case that collides: a block falling
# from a height of its own and a pendulum on a hinge. 'swinging' decides whether
# the pendulum is written out horizontal, and swings, or hanging, and stays.
TWIN = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="twin">
    <gravity>0 0 -%(g)s</gravity>
    <model name="block">
      <pose>0 0 %(height)s 0 0 0</pose>
      <link name="body">
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
    </model>
    <model name="rig">
      <pose>3 0 2 0 0 0</pose>
      <link name="base">
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
      <joint name="anchor" type="fixed"><parent>world</parent><child>base</child></joint>
      <link name="arm">
        <pose>%(arm)s</pose>
        <inertial><mass>1</mass><inertia><ixx>4e-5</ixx><iyy>4e-5</iyy><izz>4e-5</izz></inertia></inertial>
      </link>
      <joint name="swing" type="revolute">
        <parent>base</parent><child>arm</child>
        <pose>%(hinge)s</pose>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>
"""


def twin(directory, height, swinging):
    arm, hinge = ("0.5 0 0 0 0 0", "-0.5 0 0 0 0 0") if swinging else ("0 0 -0.5 0 0 0", "0 0 0.5 0 0 0")
    (directory / "scene.world").write_text(
        TWIN % {"g": G, "height": height, "arm": arm, "hinge": hinge}, encoding="utf-8"
    )
    return {"scene_file": str(directory / "scene.world"), "duration": 1.0, "samples": 5}


@pytest.fixture(scope="module")
def gazebo():
    try:
        sim.locate_gazebo()
    except sim.GazeboMissing:
        pytest.skip("no Gazebo here; see this file's docstring for running it in the simulation's image")


def test_in_gazebo_two_runs_of_one_world_at_once_each_report_only_themselves(gazebo, tmp_path):
    """Every reading of each run is its own world's: its block where its own fall
    puts it, its pendulum swinging or not as its own was written. In one
    partition each would hear both servers, and the readings would interleave."""
    low, high = tmp_path / "low", tmp_path / "high"
    low.mkdir()
    high.mkdir()
    requests = {"low": twin(low, 1.0, swinging=True), "high": twin(high, 5.0, swinging=False)}

    with concurrent.futures.ThreadPoolExecutor(2) as pool:
        running = {name: pool.submit(sim.process, str(tmp_path / name), request) for name, request in requests.items()}
        results = {name: future.result() for name, future in running.items()}

    for name, height in (("low", 1000.0), ("high", 5000.0)):
        readings = [results[name]["before"]] + results[name]["samples"] + [results[name]["after"]]
        assert len(readings) >= 6, name
        for reading in readings:
            t = reading["time"]
            fallen = height - 0.5 * G * t * t * 1000.0
            assert reading["bodies"]["block"]["pos"][2] == pytest.approx(fallen, abs=20.0), (name, t)
            swing = reading["joints"]["swing"]
            if name == "high":
                assert swing["pos"] == pytest.approx(0.0, abs=1e-3), (name, t)
            elif t >= 0.1:
                # Swinging, and by as much as its own fall says: what it lost in
                # height it has as speed. To within 2% of what the whole fall
                # is worth, which is a scale that holds at the far end of the
                # swing too, where both are nearly nothing.
                fallen = G * 0.5 * math.sin(math.radians(swing["pos"]))
                moving = 0.5 * (0.25 + 4e-5) * math.radians(swing["vel"]) ** 2
                assert moving == pytest.approx(fallen, abs=0.02 * G * 0.5), (name, t)
        assert results[name]["after"]["time"] >= 1.0, name


def test_in_gazebo_runs_at_once_each_discover_on_ports_of_their_own(gazebo, tmp_path, monkeypatch):
    """Six runs of one world side by side, as 'pc qual test' starts a package's simulations: each on a pair of
    discovery ports no other run has, and each reading only its own world -- its block falling from its own height.

    Two runs may pick one pair before either server binds it, which only shares their discovery, as every run did
    before; that is not what is under test here, so the pairs handed out are kept distinct, each still picked and
    probed by 'discovery_ports' itself."""
    for name in sim.DISCOVERY_VARIABLES:
        monkeypatch.delenv(name, raising=False)
    picked = sim.discovery_ports
    handed = set()
    lock = threading.Lock()

    def distinct(environment, token):
        with lock:
            for attempt in range(20):
                pair = picked(environment, "%s-%d" % (token, attempt))
                if pair is not None and pair not in handed:
                    handed.add(pair)
                    return pair
        return None

    monkeypatch.setattr(sim, "discovery_ports", distinct)
    environments = []
    started = sim.run_environment

    def recorded(*args, **kwargs):
        environment, partition = started(*args, **kwargs)
        environments.append(environment)
        return environment, partition

    monkeypatch.setattr(sim, "run_environment", recorded)
    heights = {"run%d" % k: 1.0 + k for k in range(6)}
    requests = {}
    for name, height in heights.items():
        (tmp_path / name).mkdir()
        requests[name] = dict(twin(tmp_path / name, height, swinging=False), duration=0.5, samples=0)

    with concurrent.futures.ThreadPoolExecutor(len(requests)) as pool:
        running = {name: pool.submit(sim.process, str(tmp_path / name), request) for name, request in requests.items()}
        results = {name: future.result() for name, future in running.items()}

    pairs = {(e["GZ_DISCOVERY_MSG_PORT"], e["GZ_DISCOVERY_SRV_PORT"]) for e in environments}
    assert len(environments) == len(pairs) == len(requests)
    for name, height in heights.items():
        assert results[name]["before"]["bodies"]["block"]["pos"][2] == pytest.approx(height * 1000.0, abs=1.0), name
        assert results[name]["reached"] >= 0.5, name
