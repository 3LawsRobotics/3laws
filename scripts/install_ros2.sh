#!/usr/bin/env bash

set -e

ROS_DISTRO=$1
if [ -z "$ROS_DISTRO" ]; then
  ROS_DISTRO="jazzy"
fi

mkdir -p "/tmp/${ROS_DISTRO}_ws/src"
sudo mkdir -p "/opt/ros/${ROS_DISTRO}"
cd "/tmp/${ROS_DISTRO}_ws"
wget "https://github.com/ros2/ros2/raw/${ROS_DISTRO}/ros2.repos"
vcs import src <ros2.repos
sudo rosdep update
sudo rosdep install --rosdistro "${ROS_DISTRO}" --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
cd src
git clone -b 2.0.2 https://github.com/ros-drivers/ackermann_msgs
cd ..
colcon build --event-handlers console_cohesion+ --merge-install --install-base "/opt/ros/${ROS_DISTRO}" --cmake-args -DCMAKE_BUILD_TYPE=Release
rm -rf "/tmp/${ROS_DISTRO}_ws"
