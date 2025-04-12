#!/bin/bash

# 重载udev规则并触发设备事件
echo "正在应用USB设备规则..."
if udevadm control --reload-rules && udevadm trigger; then
    echo "设备规则加载成功"
else
    echo "警告：设备规则加载失败，相机可能无法正常工作" >&2
fi

# 等待设备初始化完成
sleep 0.5

# 激活ROS环境
source "/opt/ros/humble/setup.bash"

# 加载工作空间环境
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source "/root/ros2_ws/install/setup.bash"
fi

# 启动ROS桥接服务
echo "启动ROS桥接服务..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# 启动JupyterLab服务
echo "启动JupyterLab服务..."
jupyter-lab --ip=0.0.0.0 --port=8888 --no-browser --allow-root --NotebookApp.token='' &

# 保持容器运行并进入交互模式
echo "容器初始化完成，进入交互终端..."
exec /bin/bash
