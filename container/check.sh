#!/usr/bin/env bash
#
# What PartCAD's `docker` sandbox does with the image, and the two things the
# plugin needs of it: a Python that can make a virtual environment, and a Gazebo
# that ROS's setup script puts on PATH.
#
# One script, run by every job that builds the image -- the image a pull request
# checks and the image `main` pushes are built separately, and the base image
# can move in between.
#
# Usage: container/check.sh <image>
set -euo pipefail
docker run --rm --entrypoint bash "$1" -c '
  set -e
  python3 -m venv /tmp/sandbox
  /tmp/sandbox/bin/pip --version
  source /opt/ros/*/setup.bash
  gz sim --versions
'
