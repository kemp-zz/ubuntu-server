#!/bin/bash
ENV ROS2_WS=/opt/ros2_ws
# 激活Python虚拟环境（路径改为/opt/venv）
source /opt/venv/bin/activate

# 加载ROS环境
source /opt/ros/humble/setup.bash

# 加载自定义ROS工作空间（根据实际需要选择是否启用）
if [ -f "${ROS2_WS}/install/local_setup.bash" ]; then
    source ${ROS2_WS}/install/local_setup.bash
fi

# 设置Turtlebot3型号（根据实际需求保留）
# export TURTLEBOT3_MODEL=burger

# 启动JupyterLab服务
JUPYTER_CMD="jupyter-lab --ip 0.0.0.0 --port=8888 --no-browser --allow-root --NotebookApp.token='' --NotebookApp.password=''"
nohup ${JUPYTER_CMD} > /var/log/jupyter.log 2>&1 &

# 启动rosbridge（根据实际需要选择是否启用）
# nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /var/log/rosbridge.log 2>&1 &


echo "容器初始化完成，进入交互终端..."
exec /bin/bash
