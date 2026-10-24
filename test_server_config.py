#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""That a server never runs with a half-written server configuration (see 'server_config' in 'simulate_gazebo.py').

A world PartCAD exports names no systems of its own, so its server runs it with
the ones its server configuration lists -- the physics, and the scene
broadcaster every pose comes from. Gazebo reads that from
'~/.gz/sim/<release>/server.config', and the first server started in a home
without one copies the installed one there, in place. A second server started
at that moment reads the file empty, writes "No <server_config> element found
when parsing plugins", and runs with no systems: paused, alive, and publishing
no pose. That is what hung CI's first runs, one or two a job, for the two minutes
of 'startup_timeout' each.

So a server is pointed at the installed configuration wherever the home has no
copy, and reads it where it is; a copy that is there is read as it is, unless
it is no configuration at all. The choice is checked without a Gazebo, with
directories that stand for an installation and a home. The last tests run a
real Gazebo, and are skipped without one, which CI's runners do not have; run
them in the image the simulation uses:

    docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 \\
        --entrypoint bash \\
        ghcr.io/partcad/partcad-sim-gazebo:latest -c \\
        'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest \\
         && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_server_config.py'
"""

import os
import sys

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

CONFIG = """<server_config>
  <plugins>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
  </plugins>
</server_config>
"""


def installation(root, *releases):
    """A 'share/gz' with a server configuration for each release, as an installation lays them out."""
    for release in releases:
        (root / ("gz-sim%s" % release)).mkdir(parents=True)
        (root / ("gz-sim%s" % release) / "server.config").write_text(CONFIG, encoding="utf-8")
    return root


@pytest.fixture
def nowhere_else(monkeypatch):
    """No system installation to find, whatever this machine has: only what a test lays out."""
    monkeypatch.setattr(sim, "SYSTEM_SHARES", ())


#
# Where the installed configuration is
#


def test_it_is_found_where_gz_config_path_points_gazebos_tool(tmp_path, nowhere_else):
    """Which is how ROS's setup script points the tool at its vendor packages."""
    share = installation(tmp_path / "opt" / "gz_sim_vendor" / "share" / "gz", 8)
    environment = {"GZ_CONFIG_PATH": os.pathsep.join([str(tmp_path / "elsewhere"), str(share)])}

    assert sim.installed_server_config(environment) == str(share / "gz-sim8" / "server.config")


def test_the_newest_release_is_the_one_gz_sim_runs(tmp_path, nowhere_else):
    """By its number, not its spelling: 'gz-sim10' is newer than 'gz-sim8'."""
    share = installation(tmp_path / "share" / "gz", 7, 8, 10)

    found = sim.installed_server_config({"GZ_CONFIG_PATH": str(share)})

    assert found == str(share / "gz-sim10" / "server.config")


def test_without_gz_config_path_it_is_found_in_the_installation_the_server_is_in(tmp_path, nowhere_else):
    """An installation of Gazebo's own packages: 'bin/gz' beside 'share/gz'."""
    installation(tmp_path / "usr" / "share" / "gz", 8)
    (tmp_path / "usr" / "bin").mkdir()
    (tmp_path / "usr" / "bin" / "gz").write_text("", encoding="utf-8")

    found = sim.installed_server_config({}, str(tmp_path / "usr" / "bin" / "gz"))

    assert found == str(tmp_path / "usr" / "share" / "gz" / "gz-sim8" / "server.config")


@pytest.mark.skipif(os.name == "nt", reason="a symbolic link to a binary is how Homebrew installs one; not on Windows")
def test_a_linked_binary_is_looked_beside_where_it_was_found_too(tmp_path, nowhere_else):
    """Homebrew's way: 'bin/gz' and 'share/gz' linked into its prefix, the binary itself in a keg of the tool's
    own that holds no simulator."""
    keg = tmp_path / "Cellar" / "gz-tools2" / "bin"
    keg.mkdir(parents=True)
    (keg / "gz").write_text("", encoding="utf-8")
    installation(tmp_path / "prefix" / "share" / "gz", 8)
    (tmp_path / "prefix" / "bin").mkdir()
    (tmp_path / "prefix" / "bin" / "gz").symlink_to(keg / "gz")

    found = sim.installed_server_config({}, str(tmp_path / "prefix" / "bin" / "gz"))

    assert found == str(tmp_path / "prefix" / "share" / "gz" / "gz-sim8" / "server.config")


@pytest.mark.skipif(os.name == "nt", reason="an executable here is a '#!' script; Windows finds programs by extension")
def test_a_binary_named_by_its_name_alone_is_the_one_on_path(tmp_path, nowhere_else, monkeypatch):
    """'PC_GZ=gz' names a program on PATH, which is what runs; not a 'gz' beside wherever this happens to run."""
    installation(tmp_path / "usr" / "share" / "gz", 8)
    (tmp_path / "usr" / "bin").mkdir()
    (tmp_path / "usr" / "bin" / "gz").write_text("#!/bin/sh\n", encoding="utf-8")
    (tmp_path / "usr" / "bin" / "gz").chmod(0o755)
    monkeypatch.chdir(tmp_path)

    found = sim.installed_server_config({"PATH": str(tmp_path / "usr" / "bin")}, "gz")

    assert found == str(tmp_path / "usr" / "share" / "gz" / "gz-sim8" / "server.config")


def test_an_installation_with_none_is_none(tmp_path, nowhere_else):
    """And the choice is then left to Gazebo (see the tests below)."""
    (tmp_path / "share" / "gz").mkdir(parents=True)

    assert sim.installed_server_config({"GZ_CONFIG_PATH": str(tmp_path / "share" / "gz")}) is None


#
# Which configuration a server is started with
#


def test_a_home_with_no_copy_yet_gets_the_installed_one_itself(tmp_path, nowhere_else):
    """What makes two servers started at once safe: neither copies anything for the other to read half-written."""
    share = installation(tmp_path / "share" / "gz", 8)
    environment = {"GZ_CONFIG_PATH": str(share), "HOME": str(tmp_path / "home")}
    warnings = []

    assert sim.server_config(environment, None, warnings) == str(share / "gz-sim8" / "server.config")
    assert warnings == []
    # And nothing is written into the home by choosing it.
    assert not (tmp_path / "home").exists()


def test_a_copy_in_the_home_is_read_as_it_is(tmp_path, nowhere_else):
    """It is somebody's configuration, and Gazebo reads it without copying anything: nothing to race."""
    share = installation(tmp_path / "share" / "gz", 8)
    own = tmp_path / "home" / ".gz" / "sim" / "8"
    own.mkdir(parents=True)
    (own / "server.config").write_text(CONFIG, encoding="utf-8")
    warnings = []

    assert sim.server_config({"GZ_CONFIG_PATH": str(share), "HOME": str(tmp_path / "home")}, None, warnings) is None
    assert warnings == []


@pytest.mark.parametrize("content", ["", "<server_con", "<sdf/>"])
def test_a_copy_that_is_no_configuration_is_passed_over_and_named(tmp_path, nowhere_else, content):
    """An empty copy is what this race leaves behind when a copy is cut short, and every server after it would
    read it and run with nothing."""
    share = installation(tmp_path / "share" / "gz", 8)
    own = tmp_path / "home" / ".gz" / "sim" / "8"
    own.mkdir(parents=True)
    (own / "server.config").write_text(content, encoding="utf-8")
    warnings = []

    chosen = sim.server_config({"GZ_CONFIG_PATH": str(share), "HOME": str(tmp_path / "home")}, None, warnings)

    assert chosen == str(share / "gz-sim8" / "server.config")
    (warning,) = warnings
    assert str(own / "server.config") in warning and "is not a Gazebo server configuration" in warning


def test_a_configuration_the_environment_names_is_kept(tmp_path, nowhere_else):
    """'GZ_SIM_SERVER_CONFIG_PATH' set by whoever runs this is their choice, and Gazebo reads it without copying."""
    share = installation(tmp_path / "share" / "gz", 8)
    environment = {"GZ_CONFIG_PATH": str(share), "HOME": str(tmp_path), sim.SERVER_CONFIG: "/theirs.config"}

    assert sim.server_config(environment, None, []) is None


def test_with_no_installed_configuration_the_choice_is_left_to_gazebo(tmp_path, nowhere_else):
    """Nothing to point a server at: it reads its home's copy, as it always has."""
    assert sim.server_config({"HOME": str(tmp_path)}, None, []) is None


def test_the_server_is_started_with_it(monkeypatch, tmp_path, nowhere_else):
    """And in the run's partition, as every program is; checked by starting nothing and recording what would have
    been started."""
    share = installation(tmp_path / "share" / "gz", 8)
    scene = tmp_path / "scene.world"
    scene.write_text(
        '<sdf version="1.9"><world name="w"><model name="m"><link name="l"/></model></world></sdf>', encoding="utf-8"
    )
    environment = {"PATH": "/bin", "GZ_CONFIG_PATH": str(share), "HOME": str(tmp_path / "home")}
    monkeypatch.setattr(sim, "locate_gazebo", lambda: (("gz", ["sim"], "topic"), environment))
    started = []

    class Recorded:
        def __init__(self, command, env=None, **_kwargs):
            """Record what would have been started, and in what environment."""
            started.append((command, env))
            self.stdout = iter(())
            self.stderr = None

        def poll(self):
            """Every program has already exited, which ends the run at its first wait."""
            return 0

    monkeypatch.setattr(sim.subprocess, "Popen", Recorded)

    with pytest.raises(Exception, match="published no poses"):
        sim.process(str(tmp_path), {"scene_file": str(scene), "duration": 0.1})

    (server,) = [env for command, env in started if command[:2] == ["gz", "sim"]]
    assert server[sim.SERVER_CONFIG] == str(share / "gz-sim8" / "server.config")
    assert server["GZ_PARTITION"].startswith("partcad-")
    assert sim.SERVER_CONFIG not in environment


#
# In a real Gazebo
#

WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
  <model name="block"><pose>0 0 1 0 0 0</pose><link name="body">
    <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
  </link></model>
</world></sdf>
"""


@pytest.fixture
def gazebo(monkeypatch, tmp_path):
    """A real Gazebo, run in a home of its own that it has never written to."""
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.delenv(sim.SERVER_CONFIG, raising=False)
    try:
        sim.locate_gazebo()
    except sim.GazeboMissing:
        pytest.skip("no Gazebo here; see this file's docstring for running it in the simulation's image")
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    return home


def run(tmp_path, **request):
    """'process' on a block in free fall, half a second of it."""
    (tmp_path / "scene.world").write_text(WORLD, encoding="utf-8")
    return sim.process(str(tmp_path), dict({"scene_file": str(tmp_path / "scene.world"), "duration": 0.5}, **request))


def test_in_gazebo_a_run_in_a_fresh_home_copies_nothing_into_it(gazebo, tmp_path):
    """The server read the installed configuration where it is: there was no copy for anything to read half-made."""
    result = run(tmp_path)

    assert result["after"]["time"] >= 0.5
    assert result["after"]["bodies"]["block"]["pos"][2] < 1000.0
    assert not list(gazebo.glob(".gz/sim/*/server.config"))


def test_in_gazebo_an_empty_copy_in_the_home_no_longer_hangs_the_run(gazebo, tmp_path):
    """The file a server reads when it starts beside the one copying it: it used to run with no systems, and the
    run waited 'startup_timeout' for a pose that never came, and said only that none had."""
    installed = sim.installed_server_config(sim.locate_gazebo()[1])
    own = gazebo / ".gz" / "sim" / str(sim._release(installed))
    own.mkdir(parents=True)
    (own / "server.config").write_text("", encoding="utf-8")

    result = run(tmp_path, startup_timeout=30)

    assert result["after"]["time"] >= 0.5
    assert any("is not a Gazebo server configuration" in warning for warning in result["warnings"])
