#!/usr/bin/env bash
#
# What PartCAD's `docker` sandbox does with the image, and the three things the
# plugin needs of it: a Python that can make a virtual environment, a Gazebo
# that ROS's setup script puts on PATH, and that Gazebo running a world -- its
# physics stepping a body and its scene broadcaster publishing where the body
# is, which is all a run reads.
#
# One script, run by every job that builds the image, on every architecture it
# is built for -- the image a pull request checks and the image `main` pushes
# are built separately, and the base image can move in between. The world is
# run here because a Gazebo that answers '--versions' can still fail to load
# its physics engine on a platform.
#
# Usage: container/check.sh <image>
set -euo pipefail
docker run --rm --entrypoint bash "$1" -c '
  set -e
  python3 -m venv /tmp/sandbox
  /tmp/sandbox/bin/pip --version
  source /opt/ros/*/setup.bash
  gz sim --versions

  # A box a metre up, dropped: a few seconds of it, and a pose message of it
  # lower than it started.
  cat > /tmp/fall.sdf <<WORLD
<?xml version="1.0"?>
<sdf version="1.11">
  <world name="fall">
    <model name="box">
      <pose>0 0 1 0 0 0</pose>
      <link name="body">
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
    </model>
  </world>
</sdf>
WORLD
  export GZ_PARTITION="check-$$"
  timeout 60 gz topic -e -t /world/fall/pose/info -n 60 > /tmp/poses.txt &
  watcher=$!
  sleep 1
  timeout 60 gz sim -s -r --iterations 3000 /tmp/fall.sdf
  wait "$watcher" || true
  python3 - <<READ
import re
text = open("/tmp/poses.txt").read()
heights = [float(z) for z in re.findall(r"name: \"box\"\s+id: \d+\s+position \{[^}]*z: ([-0-9.e]+)", text)]
assert heights, "Gazebo published no pose of the box: %r" % text[:500]
print("the box fell to %.4f m in %d pose messages" % (heights[-1], len(heights)))
assert heights[-1] < 0.99, "the box did not fall: %r" % heights[-5:]
READ
'
