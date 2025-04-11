#!/bin/bash
# Source ROS and the virtual environment

source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/local_setup.bash
source /root/myenv/bin/activate

#Execute the command
exec "$@"
