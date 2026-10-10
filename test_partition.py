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
itself; it is skipped without a Gazebo, which CI's runners do not have. Run it in
the image the simulation uses:

    docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 \\
        --entrypoint bash \\
        ghcr.io/partcad/partcad-sim-gazebo:latest -c \\
        'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest \\
         && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_partition.py'
"""

import concurrent.futures
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
