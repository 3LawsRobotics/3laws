#!/bin/bash

# Exit on errors
set -e

HAS_ROS1=0
if command -v rosversion -d &>/dev/null; then
  ROS1_VERSION=$(rosversion -d)
  HAS_ROS1=1
fi
HAS_ROS2=0
if command -v ros2 &>/dev/null; then
  ROS2_VERSION="$ROS_DISTRO"
  HAS_ROS2=1
fi

echo "starting scan"
lsb_release -a 2>/dev/null 1>ros_network_info.txt
echo >>ros_network_info.txt
# shellcheck disable=SC2129
# Get the list of topics
if [ "$HAS_ROS1" -eq 1 ]; then
  if rostopic list &>/dev/null; then
    {
      topicsRos1=$(rostopic list)

      echo "================================================================================" >>ros_network_info.txt
      echo "===================================== ROS1 =====================================" >>ros_network_info.txt
      echo "================================================================================" >>ros_network_info.txt
      echo "Ros version: ${ROS1_VERSION}" >>ros_network_info.txt
      echo -e "\n\n===================================  TOPICS  ===================================" >>ros_network_info.txt
      # Iterate through each topic
      for topic in $topicsRos1; do
        echo "exploring topic: $topic"
        echo -e "\nTopic: $topic" >>ros_network_info.txt
        echo "----------------------------------------" >>ros_network_info.txt

        # Get the info for the current topic
        rostopic info "$topic" >>ros_network_info.txt

        echo "========================================" >>ros_network_info.txt
      done

      echo -e "\n\n====================================  NODES  ===================================\n" >>ros_network_info.txt
      # Get the list of topics
      nodes=$(rosnode list)

      # Iterate through each topic
      for node in $nodes; do
        echo "exploring node: $node"
        # Get the info for the current topic
        rosnode info "$node" >>ros_network_info.txt
      done
    } || {
      echo "Failed to discover ROS1 topics, passing to ROS2"
    }
  else
    echo "ros master doesn't seem to be up"
  fi
else
  echo "ROS1 not found, passing to ROS2"
fi

# shellcheck disable=SC2129
# Get the list of ros2 topics
if [ "$HAS_ROS2" -eq 1 ]; then
  {
    topicsRos2=$(ros2 topic list) &>/dev/null

    echo -e "\n\n\n================================================================================" >>ros_network_info.txt
    echo "===================================== ROS2 =====================================" >>ros_network_info.txt
    echo "================================================================================" >>ros_network_info.txt
    echo "Ros version: ${ROS2_VERSION}" >>ros_network_info.txt
    echo -e "\n\n===================================  TOPICS  ===================================\n" >>ros_network_info.txt
    # Iterate through each topic
    for topic in $topicsRos2; do
      echo "exploring topic: $topic"
      echo -e "\nTopic: $topic" >>ros_network_info.txt
      echo "----------------------------------------" >>ros_network_info.txt

      # Get the info for the current topic
      ros2 topic info -v "$topic" >>ros_network_info.txt

      echo "========================================" >>ros_network_info.txt
    done

    echo -e "\n\n====================================  NODES  ===================================" >>ros_network_info.txt
    # Get the list of topics
    nodes=$(ros2 node list)

    # Iterate through each topic
    for node in $nodes; do
      echo "exploring node: $node"
      echo -e "\n----------------------------------------" >>ros_network_info.txt

      # Get the info for the current topic
      ros2 node info "$node" >>ros_network_info.txt
    done
  } || {
    echo "Failed to discover ROS2 topics"
  }
else
  echo "ROS2 not found"
fi

echo "End of scan, results saved in ros_network_info.txt"
