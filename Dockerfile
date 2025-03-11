# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 系统配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y ros-humble-desktop ros-humble-ros-base python3-rosdep && \
    rosdep init && \
    rosdep update

# Stage 3: Isaac ROS 构建
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake build-essential \
    python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

# 配置rosdep源
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update

# 克隆仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/${repo}.git || exit 1; \
    done

# 构建
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
