# Stage 1: 使用官方基础镜像代替自定义镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 系统级配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 2: ROS2 安装
FROM base AS ros-installer
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep && \
    rosdep init && \
    rosdep update

# Stage 3: Isaac ROS 构建
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble

RUN apt-get update && \
    apt-get install -y git cmake build-essential libopencv-dev

WORKDIR /isaac_ws/src
RUN git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common && \
    git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox && \
    git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam

RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install

# Stage 4: 最终镜像
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws

USER appuser
WORKDIR /home/appuser

CMD ["bash"]
