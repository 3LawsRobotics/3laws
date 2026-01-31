#!/bin/bash

set -e # Exit immediately if a command exits with a non-zero status

screen -dmS lll_rosbridge bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091"

screen -dmS lll_control_panel /opt/3laws/control_panel/control-panel-backend 8000 /opt/3laws/control_panel/build/

source /opt/ros/humble/setup.bash && exec "$@"
