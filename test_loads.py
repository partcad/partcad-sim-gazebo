#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""What a run says acts on each body, and how the world is made to say it (see 'fill_loads').

Gazebo reports what a joint passes from one body to the other through a
force-torque sensor, which a world carries only where somebody put one, in a
world that runs the ForceTorque system. So the run puts one on every joint
between two PartCAD bodies ('plan_wrenches') and reads each sensor's wrench
back into the vocabulary the MuJoCo plugin states loads in: in the world's frame,
at the joint, on the body, from the body at the other end.

No Gazebo is needed: the world is XML and the messages are text, as in
'test_joints.py'. That a real Gazebo measures what is read here -- the weight of
an arm held level by the spring of its hinge, and the moment of it -- was checked
against Harmonic, and the README says with what.
"""

import math
import os
import sys
from xml.etree import ElementTree

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate_gazebo as sim  # noqa: E402

pc = pytest.importorskip("partcad")
sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))

RIG = """
<sdf version="1.9"><world name="rig">
  <model name="rig">
    <pose>0 0 1 0 0 0</pose>
    <link name="frame"><inertial><mass>3</mass></inertial></link>
    <joint name="weld" type="fixed"><parent>world</parent><child>frame</child></joint>
    <link name="arm">
      <pose>0.5 0 0 0 0 0</pose>
      <inertial><pose>0.1 0 0 0 0 0</pose><mass>1</mass></inertial>
    </link>
    <joint name="hold" type="revolute">
      <pose>-0.5 0 0 0 0 0</pose>
      <parent>frame</parent><child>arm</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>
  </model>
  <model name="shelf">
    <static>true</static>
    <link name="board"/><link name="door"/>
    <joint name="hinge" type="revolute"><parent>board</parent><child>door</child></joint>
  </model>
</world></sdf>
"""

NAMES = sim.Names(
    {
        "bodies": {
            "frame": {"path": ["rig", "frame"], "parts": [{"label": "rig/frame", "part": "//p:f", "placement": None}]},
            "arm": {"path": ["rig", "arm"], "parts": [{"label": "rig/arm", "part": "//p:a", "placement": None}]},
            "board": {"path": ["shelf", "board"]},
            "door": {"path": ["shelf", "door"]},
        },
        "joints": {"knee": {"path": ["rig", "hold"], "joint": "knee"}},
    }
)


def world():
    return ElementTree.fromstring(RIG).find("world")


def test_every_joint_between_two_bodies_is_given_a_sensor_and_the_world_the_system_once():
    element = world()
    warnings = []

    (wrench,) = sim.plan_wrenches(element, "/partcad/wrench", 1000.0, warnings, NAMES)

    assert (wrench.topic, wrench.scoped, wrench.joint, wrench.reported) == ("/partcad/wrench/0", "rig", "hold", "knee")
    assert (wrench.child, wrench.parent) == ("arm", "frame")
    # The joint's frame in its child link's: where the sensor measures about.
    assert wrench.origin[1] == pytest.approx((-500.0, 0.0, 0.0))
    sensor = element.find("model[@name='rig']/joint[@name='hold']/sensor")
    assert sensor.get("type") == "force_torque"
    assert (sensor.findtext("topic"), sensor.findtext("update_rate")) == ("/partcad/wrench/0", "1000")
    assert sensor.findtext("force_torque/frame") == "child"
    assert sensor.findtext("force_torque/measure_direction") == "parent_to_child"
    # Once, and on a model: a system attached to the world turns Gazebo's defaults off.
    plugins = element.findall(".//plugin")
    assert [(p.get("filename"), p.get("name")) for p in plugins] == [
        ("gz-sim-forcetorque-system", "gz::sim::systems::ForceTorque")
    ]
    assert element.find("plugin") is None
    # Nothing on a joint of a static model, which moves nothing; and a joint to
    # the world, which Gazebo makes no sensor for, is said rather than left out.
    assert element.find("model[@name='shelf']/joint/sensor") is None
    assert any("what holds 'frame' to the world is not among its loads" in w for w in warnings)


def test_a_world_with_no_names_beside_it_is_given_nothing():
    element = world()
    assert sim.plan_wrenches(element, "/p", 1000.0, [], sim.AS_WRITTEN) == []
    assert element.find(".//sensor") is None


def test_a_sensor_asked_for_every_step_is_asked_for_a_rate_of_nought(tmp_path):
    """SDFormat's "every cycle": a rate of one over a 0.5 ms step stops Gazebo at t = 0."""
    scene = tmp_path / "rig.world"
    scene.write_text(RIG, encoding="utf-8")

    run_file, models = sim.prepare_world(str(scene), [], NAMES, loads=sim.EVERY_STEP)

    try:
        assert [model.joint for model in models if isinstance(model, sim.Wrench)] == ["hold"]
        assert ElementTree.parse(run_file).getroot().find(".//sensor").findtext("update_rate") == "0"
    finally:
        os.remove(run_file)


def test_the_world_that_runs_is_a_copy_with_the_sensors_in_it(tmp_path):
    scene = tmp_path / "rig.world"
    scene.write_text(RIG, encoding="utf-8")

    run_file, models = sim.prepare_world(str(scene), [], NAMES, loads=500.0)

    try:
        assert run_file != str(scene)
        wrenches = [model for model in models if isinstance(model, sim.Wrench)]
        assert [w.joint for w in wrenches] == ["hold"]
        copied = ElementTree.parse(run_file).getroot()
        assert copied.find(".//sensor").findtext("update_rate") == "500"
        # The world handed over is left as it was.
        assert ElementTree.parse(str(scene)).getroot().find(".//sensor") is None
    finally:
        os.remove(run_file)


def test_a_bodys_weight_is_its_links_mass_at_its_centre_of_mass(tmp_path):
    scene = tmp_path / "rig.world"
    scene.write_text(RIG, encoding="utf-8")
    inertials = sim.link_inertials(str(scene))
    assert inertials[("rig", "frame")] == (3.0, pytest.approx((0.0, 0.0, 0.0)))
    assert inertials[("rig", "arm")] == (1.0, pytest.approx((100.0, 0.0, 0.0)))


class Stream:
    """A force-torque sensor's stream that has one message to give."""

    def __init__(self, model, text):
        self.model = model
        self.text = text

    def at(self, moment, patience=None):
        return self.text


def wrench_message(force, torque):
    """A 'gz.msgs.Wrench' as 'gz topic -e' prints it: a field a line, nought ones left out."""

    def fields(vector):
        return "".join("  %s: %r\n" % (axis, value) for axis, value in zip("xyz", vector) if value)

    return "header {\n  stamp {\n    sec: 3\n  }\n}\nforce {\n%s}\ntorque {\n%s}\n" % (fields(force), fields(torque))


def reading(arm_quat):
    """The rig at an instant: the frame where the world put it, the arm 600 mm along +X turned by 'arm_quat'."""
    return {
        "time": 3.0,
        "bodies": {
            "frame": {"pos": [0.0, 0.0, 1000.0], "quat": [1.0, 0.0, 0.0, 0.0]},
            "arm": {"pos": [500.0, 0.0, 1000.0], "quat": list(arm_quat)},
        },
        "joints": {},
    }


def test_a_joints_wrench_is_stated_in_the_worlds_frame_at_the_joint_on_both_bodies():
    # The arm turned 90 degrees about +Z: a force along its own +X is along the
    # world's +Y, and the joint, half a metre behind the arm's frame along the
    # arm's own -X, is half a metre behind it along the world's -Y.
    turned = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    seen = reading(turned)
    wrench = sim.Wrench("/t/0", "rig", "hold", "knee", "arm", "frame", ((1.0, 0.0, 0.0, 0.0), (-500.0, 0.0, 0.0)))
    stream = Stream(wrench, wrench_message([2.0, 0.0, 9.81], [0.0, -5.0, 0.0]))
    warnings = []

    sim.fill_loads(seen, [stream], {("rig", "arm"): (1.0, (100.0, 0.0, 0.0))}, [0.0, 0.0, -9.81], warnings, 0.0, NAMES)

    arm = seen["bodies"]["arm"]
    held, weight = arm["loads"]
    assert (held["kind"], held["from"], held["joint"]) == ("joint", "frame", "knee")
    assert held["at"] == pytest.approx([500.0, -500.0, 1000.0])
    assert held["force"] == pytest.approx([0.0, 2.0, 9.81])
    assert held["torque"] == pytest.approx([5.0, 0.0, 0.0])
    assert (weight["kind"], weight["force"]) == ("gravity", pytest.approx([0.0, 0.0, -9.81]))
    assert arm["mass"] == 1.0 and arm["com"] == pytest.approx([500.0, 100.0, 1000.0])
    assert weight["at"] == pytest.approx(arm["com"])
    # The frame feels the same joint the other way. It has no weight to report:
    # the world says nothing of what it weighs.
    (back,) = seen["bodies"]["frame"]["loads"]
    assert (back["from"], back["joint"]) == ("arm", "knee")
    assert back["force"] == pytest.approx([0.0, -2.0, -9.81])
    assert back["torque"] == pytest.approx([-5.0, 0.0, 0.0])
    assert warnings == []


def test_the_first_reading_carries_the_weights_alone_and_says_why():
    seen = reading((1.0, 0.0, 0.0, 0.0))
    wrench = sim.Wrench("/t/0", "rig", "hold", "knee", "arm", "frame", ((1.0, 0.0, 0.0, 0.0), (-500.0, 0.0, 0.0)))
    warnings = []

    sim.fill_loads(
        seen,
        [Stream(wrench, wrench_message([0, 0, 9.81], [0, 0, 0]))],
        {("rig", "arm"): (1.0, (100.0, 0.0, 0.0))},
        [0.0, 0.0, -9.81],
        warnings,
        0.0,
        NAMES,
        paused=True,
    )

    assert [load["kind"] for load in seen["bodies"]["arm"]["loads"]] == ["gravity"]
    assert any("while the world is paused" in w for w in warnings)


def test_a_sensor_that_said_nothing_is_named_rather_than_read_as_nought():
    seen = reading((1.0, 0.0, 0.0, 0.0))
    wrench = sim.Wrench("/t/0", "rig", "hold", "knee", "arm", "frame", ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    warnings = []

    sim.fill_loads(seen, [Stream(wrench, None)], {}, [0.0, 0.0, -9.81], warnings, 0.0, NAMES)

    assert "loads" not in seen["bodies"]["arm"]
    assert any("the load of joint 'knee' is not reported" in w for w in warnings)


def test_a_run_says_which_body_each_part_is_in():
    import naming

    if not hasattr(naming, "parts_of"):
        pytest.skip("this PartCAD cannot say which parts a body is (partcad/partcad#823)")
    assert NAMES.parts == {
        "rig/frame": {"part": "//p:f", "body": "frame", "placement": None},
        "rig/arm": {"part": "//p:a", "body": "arm", "placement": None},
    }


#
# In a real Gazebo: skipped where there is none, and run in the simulation's
# image the way 'test_joints.py' says.
#

HELD = RIG.replace(
    "<axis><xyz>0 1 0</xyz></axis>",
    "<axis><xyz>0 1 0</xyz><dynamics><spring_stiffness>400</spring_stiffness><damping>20</damping></dynamics></axis>",
).replace('<sdf version="1.9"><world name="rig">', '<sdf version="1.9"><world name="rig"><gravity>0 0 -9.81</gravity>')


@pytest.fixture(scope="module")
def held(tmp_path_factory):
    try:
        sim.locate_gazebo()
    except sim.GazeboMissing:
        pytest.skip("no Gazebo here; see 'test_joints.py' for running it in the simulation's image")
    import naming

    directory = tmp_path_factory.mktemp("loads")
    scene = directory / "scene.world"
    scene.write_text(HELD, encoding="utf-8")
    naming.write_sidecar(
        str(scene),
        "world",
        {"scene": {"rig": "rig"}},
        {
            "frame": {"path": ["rig", "frame"], "parts": [{"label": "rig/frame", "part": "//p:f", "placement": None}]},
            "arm": {"path": ["rig", "arm"], "parts": [{"label": "rig/arm", "part": "//p:a", "placement": None}]},
        },
        {"knee": {"path": ["rig", "hold"], "joint": "knee"}},
    )
    return sim.process(str(directory), {"scene_file": str(scene), "duration": 3.0})


def test_in_gazebo_a_hinge_holds_an_arm_with_its_weight_and_the_moment_of_it(held):
    """An arm of 1 kg, its centre 600 mm out, held level by its hinge's spring: 9.81 N up, and m g d about +y."""
    after = held["after"]
    arm = after["bodies"]["arm"]
    (load,) = [load for load in arm["loads"] if load["kind"] == "joint"]
    assert (load["from"], load["joint"]) == ("frame", "knee")
    assert load["at"] == pytest.approx([0.0, 0.0, 1000.0], abs=0.01)
    assert load["force"] == pytest.approx([0.0, 0.0, 9.81], abs=1e-3)
    lever = (arm["com"][0] - load["at"][0]) / 1000.0
    assert load["torque"] == pytest.approx([0.0, -9.81 * lever, 0.0], abs=1e-3)
    # The frame feels it the other way; nothing reports what holds the frame up.
    (back,) = [load for load in after["bodies"]["frame"]["loads"] if load["kind"] == "joint"]
    assert back["force"] == pytest.approx([0.0, 0.0, -9.81], abs=1e-3)
    assert any("to the world is not among its loads" in w for w in held["warnings"])
    import naming

    # Which body a part is in, where this PartCAD can say (see above).
    assert held.get("parts", {}).get("rig/arm", {}).get("body") == ("arm" if hasattr(naming, "parts_of") else None)
