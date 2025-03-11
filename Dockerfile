# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata ca-certificates && \
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
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-rosdep \
    python3-pytest \
    python3-ament-python && \
    rosdep init && \
    rosdep update --include-eol-distros

# Stage 3: Isaac ROS 构建
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

# 安装核心依赖
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    libopencv-dev \
    libeigen3-dev \
    python3-venv \
    python3-colcon-common-extensions && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 配置官方ROS源
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    rosdep init && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update

# 克隆官方兼容仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch humble \
        https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 手动处理依赖
RUN mkdir -p /isaac_ws/src/isaac_ros_deps && \
    cd /isaac_ws/src/isaac_ros_deps && \
    git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git

# 构建
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

# Stage 4: 最终镜像
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser
USER appuser
WORKDIR $HOME

CMD ["bash"]
