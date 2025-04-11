#!/bin/bash

# 激活ROS环境
source /opt/ros/$ROS_DISTRO/setup.bash

# 启动ROS桥接服务
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# 启动JupyterLab
jupyter-lab --ip=0.0.0.0 --port=8888 --no-browser --allow-root &

# 保持容器运行
exec "$@"
