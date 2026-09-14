#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What can be checked about the simulation plugin without a Gazebo in the room.

Gazebo is a native program rather than a wheel, so a contributor and a CI runner
alike may not have one. That rules out an end-to-end run, and it does *not* rule
out the parts most likely to be quietly wrong -- which here is nearly all of it,
because what this plugin does is read what a program says and turn it into what
PartCAD reads:

* the protobuf text-format parser, which is the one thing that fails silently
  rather than loudly: a field it drops becomes a body that did not move;
* splitting one stream into messages, which decides which reading is 'after';
* the quaternion, whose component order in the message is not the order PartCAD
  states one in, so a copy rather than a re-ordering is wrong by a rotation;
* the unit conversion, which is a factor of a thousand and looks plausible in
  either direction until something is compared against a part's own dimensions;
* and that a machine with no Gazebo is told which of the two ways to get one
  rather than shown a traceback.

Run it with `pytest`. It needs nothing but the standard library.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate_gazebo as sim  # noqa: E402


#
# Finding a Gazebo
#


def test_a_machine_with_no_gazebo_is_told_both_ways_to_get_one(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda _name: None)

    with pytest.raises(sim.GazeboMissing) as raised:
        sim.find_gazebo()
    message = str(raised.value)
    # The two answers, because which one applies is a fact about the machine.
    assert "gazebosim.org" in message
    assert "docker" in message


def test_the_newest_generation_present_is_the_one_used(monkeypatch):
    """'gz sim' today, 'ign gazebo' before it. A machine may well have both."""
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name: "/usr/bin/" + name)

    binary, args, topic = sim.find_gazebo()
    assert binary == "/usr/bin/gz"
    assert args == ["sim"]
    assert topic == "topic"


def test_an_older_generation_is_run_the_way_it_spells_itself(monkeypatch):
    monkeypatch.delenv("PC_GZ", raising=False)
    monkeypatch.setattr(sim.shutil, "which", lambda name: "/usr/bin/ign" if name == "ign" else None)

    binary, args, _topic = sim.find_gazebo()
    assert binary == "/usr/bin/ign"
    assert args == ["gazebo"]


def test_an_override_points_at_the_tools_beside_it(monkeypatch):
    """An installation PATH does not know about is still one installation."""
    monkeypatch.setenv("PC_GZ", "/opt/gz/bin/gz")

    binary, args, topic = sim.find_gazebo()
    assert (binary, args) == ("/opt/gz/bin/gz", ["sim"])
    # Beside the server, not whatever 'gz' PATH happens to hold: the point of
    # the override is that this installation is not the one PATH finds.
    assert topic == os.path.join("/opt/gz/bin", "topic")


#
# Reading what the server publishes
#

POSE_MESSAGE = """header {
  stamp {
    sec: 3
    nsec: 500000000
  }
}
pose {
  name: "block"
  id: 8
  position {
    x: 0.1
    y: -0.02
    z: 0.5
  }
  orientation {
    x: 0
    y: 0
    z: 0.7071068
    w: 0.7071068
  }
}
"""


def test_a_pose_is_read_in_millimetres():
    """SDFormat is metres and PartCAD is millimetres, everywhere and always."""
    reading = sim.snapshot(sim.parse_message(POSE_MESSAGE))

    assert reading["bodies"]["block"]["pos"] == pytest.approx([100.0, -20.0, 500.0])


def test_the_quaternion_is_stated_the_way_partcad_states_one():
    """'w x y z'. The message prints 'x y z w', which is a different rotation."""
    reading = sim.snapshot(sim.parse_message(POSE_MESSAGE))

    assert reading["bodies"]["block"]["quat"] == pytest.approx([0.7071068, 0.0, 0.0, 0.7071068])


def test_the_simulated_clock_comes_out_of_the_message():
    """Which is what a run waits on, rather than on the wall clock."""
    assert sim.snapshot(sim.parse_message(POSE_MESSAGE))["time"] == pytest.approx(3.5)


def test_a_field_the_message_omits_reads_as_zero():
    """Protobuf text format prints no field whose value is the default."""
    reading = sim.snapshot(
        sim.parse_message('header {\n}\npose {\n  name: "still"\n  position {\n  }\n  orientation {\n    w: 1\n  }\n}\n')
    )

    assert reading["time"] == 0.0
    assert reading["bodies"]["still"]["pos"] == [0.0, 0.0, 0.0]
    assert reading["bodies"]["still"]["quat"] == [1.0, 0.0, 0.0, 0.0]


def test_an_entity_with_no_name_is_kept_under_its_id():
    """Something moved; a reading that omits it is worse than an odd name."""
    reading = sim.snapshot(sim.parse_message('header {\n}\npose {\n  id: 12\n  position {\n    z: 1\n  }\n}\n'))

    assert list(reading["bodies"]) == ["entity_12"]


def test_every_entity_in_one_message_is_one_reading():
    """A reading is where everything was at one instant, not one body's history."""
    text = POSE_MESSAGE + 'pose {\n  name: "ground"\n  id: 4\n  position {\n  }\n}\n'

    assert sorted(sim.snapshot(sim.parse_message(text))["bodies"]) == ["block", "ground"]


#
# One stream, many messages
#


def test_a_stream_is_split_where_the_next_message_starts():
    """'gz topic -e' prints one message after another with nothing between."""
    stream = (POSE_MESSAGE + POSE_MESSAGE.replace("sec: 3", "sec: 9")).splitlines(keepends=True)

    times = [sim.snapshot(sim.parse_message(text))["time"] for text in sim.messages(stream)]
    assert times == pytest.approx([3.5, 9.5])


def test_the_last_message_is_not_lost_at_the_end_of_the_stream():
    """It is the one 'after' is, so losing it loses the whole run."""
    assert len(list(sim.messages(POSE_MESSAGE.splitlines(keepends=True)))) == 1


def test_a_stream_that_said_nothing_is_no_messages_rather_than_one_empty_one():
    assert list(sim.messages([])) == []


#
# The world's name, which is what its topics are named after
#


def test_the_world_name_is_read_out_of_the_file(tmp_path):
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><world name="warehouse"/></sdf>', encoding="utf-8")

    assert sim.world_name(str(path)) == "warehouse"


def test_a_file_that_names_no_world_says_so_rather_than_reading_nothing(tmp_path):
    """Every topic is namespaced under the name, so a wrong one reads silence."""
    path = tmp_path / "scene.world"
    path.write_text('<sdf version="1.9"><model name="block"/></sdf>', encoding="utf-8")

    with pytest.raises(Exception, match="names no world"):
        sim.world_name(str(path))
