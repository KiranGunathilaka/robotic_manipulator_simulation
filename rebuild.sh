#!/usr/bin/env bash
set -e

WS=~/development/ros/robotic_manipulator_simulation

echo ">>> Cleaning build, install, log in $WS"
rm -rf "$WS/build" "$WS/install" "$WS/log"

echo ">>> Sourcing base ROS (adjust distro if needed)"
source /opt/ros/jazzy/setup.sh

echo ">>> Building workspace (symlink install)"
cd "$WS"
colcon build --symlink-install

echo ">>> Sourcing workspace overlay"
source "$WS/install/setup.sh"

echo ">>> Setting Gazebo model path"
PREFIX=$(ros2 pkg prefix gp7_robot)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PREFIX/share

echo ">>> Gazebo path now:"
echo "$GZ_SIM_RESOURCE_PATH"

echo ">>> Done. You're now in: $WS"
