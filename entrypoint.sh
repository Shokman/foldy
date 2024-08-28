#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

if [ -f /root/ws/install/setup.bash ]
then
  source /root/ws/install/setup.bash
fi

exec "$@"
