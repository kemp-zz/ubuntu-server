FROM ubuntu:22.04

# 基础系统配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    ROS_PYTHON_VERSION=3

# 初始化ROS环境变量
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages" \
    LD_LIBRARY_PATH="/opt/ros/humble/lib" \
    AMENT_PREFIX_PATH="/opt/ros/humble" \
    CMAKE_PREFIX_PATH="/opt/ros/humble" \
    PATH="/opt/ros/humble/bin:$PATH"

RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 安装ROS2
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init && rosdep update

# 安装相机驱动依赖
RUN apt-get update && apt-get install -y \
    libgflags-dev \
    ros-humble-image-geometry \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev

# 安装libuvc
RUN apt-get update && apt-get install -y \
    git cmake wget \
    && git clone https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cd /tmp/libuvc \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && make install \
    && ldconfig

# 安装Blender及依赖
# 安装Blender及Web依赖（带版本锁定）
RUN apt-get update && apt-get install -y \
    blender=3.3.0+dfsg-2 \  # Ubuntu 22.04官方源版本
    python3-pip \
    nginx \
    && pip3 install flask==3.0.2 \
    && { [ ! -f /usr/bin/blender ] || rm /usr/bin/blender; } \
    && ln -sfv /usr/games/blender /usr/bin/blender \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# 下载源码
RUN git clone https://github.com/orbbec/ros2_astra_camera.git src/ros2_astra_camera \
    && git clone https://github.com/emanuelbuholzer/ros2_blender.git src/ros2_blender

# 安装USB规则
RUN bash src/ros2_astra_camera/astra_camera/scripts/install.sh

# 构建工作空间
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && colcon build --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# 添加工作空间环境变量
ENV PYTHONPATH="$PYTHONPATH:/ros2_ws/install/lib/python3.10/site-packages" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/ros2_ws/install/lib" \
    AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:/ros2_ws/install" \
    CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/ros2_ws/install" \
    PATH="$PATH:/ros2_ws/install/bin"

# 最终配置
ENV ROS_DOMAIN_ID=38
EXPOSE 3000

# 启动命令
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && \
    ros2 launch astra_camera astra.launch.xml & \
    ros2 launch ros2_blender blender.launch.py"]
