# The image the Gazebo simulation runs in when the machine has no Gazebo of its
# own -- `dockerImage` on the `simulation:` entry in `partcad.yaml`.
#
# ROS Jazzy's simulation stack, which is Gazebo Harmonic, plus the one thing it
# lacks to be a PartCAD sandbox: a Python that can make a virtual environment.
# PartCAD's `docker` sandbox builds an environment inside the image it is given,
# with `python3 -m venv`, and Ubuntu ships the `ensurepip` that needs as a
# package of its own, which the ROS image does not install.
#
# Built for linux/amd64 and linux/arm64, each natively (see
# `.github/workflows/image.yml`). So it is built here the way OSRF builds
# `osrf/ros:jazzy-simulation` -- the official `ros:jazzy-ros-base` and the
# `ros-jazzy-simulation` packages on it, which is that image's own recipe --
# rather than on top of it: OSRF publishes that image for amd64 alone, while
# `ros:jazzy-ros-base` and the packages are published for both.
#
# Nothing else is added. Gazebo stays where ROS put it, off PATH, and
# `simulate_gazebo.py` finds it through ROS's setup script -- the same way it
# finds it on a machine with ROS installed.
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-simulation python3-venv \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.source="https://github.com/partcad/partcad-sim-gazebo"
LABEL org.opencontainers.image.description="Gazebo Harmonic, for running PartCAD's Gazebo simulation plugin in a sandbox"
