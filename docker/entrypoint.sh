#!/bin/bash

set -e # Exit immediately if a command exits with a non-zero status

source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091 &

sleep 1

/opt/3laws/control_panel/control-panel-backend 8000 /opt/3laws/control_panel/build/ &
PID_CP=$!

sleep 1
ros2 launch lll_supervisor supervisor.launch.py

wait $PID_CP

echo "Control Panel backend exited, shutting down."
