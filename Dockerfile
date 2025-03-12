# Stage 1: 基础镜像（保持原样）
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

RUN apt-get update && \
    apt-get install -y locales && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS2 Humble 安装（新增NVIDIA仓库）
FROM base AS ros-installer

# 配置APT仓库（新增NVIDIA官方源）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    # 添加ROS仓库
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    # 新增NVIDIA仓库（关键修复）
    curl -sSL https://nvidia-isaac-ros.s3.us-east-2.amazonaws.com/ros.key -o /usr/share/keyrings/isaac-ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] https://nvidia-isaac-ros.s3.us-east-2.amazonaws.com/ros2-humble jammy main" > /etc/apt/sources.list.d/isaac-ros.list

# 安装ROS核心组件（合并新包列表）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # 新增基础开发工具 ↓
    devscripts \
    dh-make \
    fakeroot \
    python3-bloom \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosinstall-generator \
    python3-vcstool \
    quilt \
    # 新增ROS组件 ↓
    ros-humble-angles \
    ros-humble-apriltag \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-cv-bridge \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-diagnostics \
    ros-humble-diagnostic-aggregator \
    ros-humble-diagnostic-updater \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-graceful-controller \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbridge-suite \
    ros-humble-rosx-introspection \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-robot-monitor \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-vision-msgs-rviz-plugins \
    && rosdep init \
    && rosdep update --include-eol-distros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 升级setuptools并安装Python依赖
RUN python3 -m pip install --upgrade --force-reinstall --target=/usr/lib/python3/dist-packages setuptools==65.7.0 \
    && python3 -m pip install -U --no-cache-dir \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    matplotlib \
    pandas \
    rosbags \
    boto3


FROM base AS isaac-builder

# 继承ROS安装（新增关键依赖）
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/
COPY --from=ros-installer /etc/apt/sources.list.d/isaac-ros.list /etc/apt/sources.list.d/
COPY --from=ros-installer /usr/share/keyrings/isaac-ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/


# 安装核心构建工具链（强化开发依赖）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    python3-rosdep \
    python3-venv \
    python3-pytest \
    python3-pytest-mock \
    libopencv-dev \
    libeigen3-dev \
    # 新增构建必需组件
    devscripts \
    dh-make \
    fakeroot \
    quilt \
    python3-bloom \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 配置rosdep源（优先级优化）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    curl -sSL https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/docker/rosdep/extra_rosdeps.yaml \
        -o /etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml && \
    # 调整优先级顺序（关键修复）
    printf "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml\nyaml file:///etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml\n" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update --include-eol-distros --rosdistro humble




# 克隆并构建Isaac ROS核心仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 构建优化参数
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=80"


# 构建指令（新增环境变量）
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    # 设置CUDA架构（关键修复）
    export CMAKE_CUDA_ARCHITECTURES=80 && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "isaac_ros_test python3-pytest" --rosdistro humble \
    && colcon build \
        --symlink-install \
        --parallel-workers $(($(nproc) * 2)) \
        --cmake-args $CMAKE_ARGS


# Stage 4: 最终运行时镜像
FROM base

# 继承运行时组件
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

# 创建非root用户
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser
USER appuser
WORKDIR $HOME

CMD ["bash"]
