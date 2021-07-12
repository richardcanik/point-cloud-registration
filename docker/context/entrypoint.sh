#!/bin/bash
set -e

# setup ros environment
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
CATKIN_WS_SETUP="/catkin_ws/devel/setup.bash"

if [ -f "$ROS_SETUP" ]; then
  . "$ROS_SETUP"
fi

if [ -f "$CATKIN_WS_SETUP" ]; then
  . "$CATKIN_WS_SETUP"
fi

roscore &
exec "$@"
