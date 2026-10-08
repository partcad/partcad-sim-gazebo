# The image the Gazebo simulation runs in when the machine has no Gazebo of its
# own -- `dockerImage` on the `simulation:` entry in `partcad.yaml`.
#
# OSRF's own ROS image with the simulation stack in it, which is Gazebo Harmonic,
# plus the one thing it lacks to be a PartCAD sandbox: a Python that can make a
# virtual environment. PartCAD's `docker` sandbox builds an environment inside
# the image it is given, with `python3 -m venv`, and Ubuntu ships the `ensurepip`
# that needs as a package of its own, which the ROS image does not install.
#
# Nothing else is added. Gazebo stays where ROS put it, off PATH, and
# `simulate_gazebo.py` finds it through ROS's setup script -- the same way it
# finds it on a machine with ROS installed.
FROM osrf/ros:jazzy-simulation

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.source="https://github.com/partcad/partcad-sim-gazebo"
LABEL org.opencontainers.image.description="Gazebo Harmonic, for running PartCAD's Gazebo simulation plugin in a sandbox"
