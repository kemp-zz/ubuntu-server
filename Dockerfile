# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 新增Python警告抑制（来自colcon问题#454）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

# 时区与locale配置
RUN apt-get update && \
    apt-get install -y locales && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer

# 配置官方ROS源（带APT缓存）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

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
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

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
    bloom \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 配置rosdep源（优先级优化）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    curl -sSL https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/docker/rosdep/extra_rosdeps.yaml \
        -o /etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml && \
    printf "yaml file:///etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml\nyaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml\n" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update --include-eol-distros --rosdistro humble

# 安装修补后的ROS核心组件
# 1. 安装negotiated接口
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_WORKSPACE}/src && cd ${ROS_WORKSPACE}/src \
    && git clone https://github.com/osrf/negotiated -b master \
    && source /opt/ros/humble/setup.bash \
    && cd negotiated_interfaces \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb \
    && rm -rf src build log

# 2. 修复image_proc的resize节点
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_WORKSPACE}/src && cd ${ROS_WORKSPACE}/src \
    && git clone https://github.com/ros-perception/image_pipeline.git \
    && cd image_pipeline \
    && git checkout 55bf2a38c327b829c3da444f963a6c66bfe0598f \
    && git cherry-pick 969d6c763df99b42844742946f7a70c605a72a15 \
    && cd image_proc \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb

# 3. 修补rclcpp多线程执行器
COPY patches/rclcpp-disable-tests.patch /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_WORKSPACE}/src && cd ${ROS_WORKSPACE}/src \
    && git clone https://github.com/ros2-gbp/rclcpp-release.git \
    && cd rclcpp-release \
    && patch -i /tmp/rclcpp-disable-tests.patch \
    && git cherry-pick 232262c02a1265830c7785b7547bd51e1124fcd8 \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb

# 安装MoveIt 2完整套件
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    ros-humble-moveit* \
    ros-humble-ur* \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 克隆并构建Isaac ROS核心仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 构建优化参数
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=80"

# 构建Isaac组件（启用并行编译）
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "isaac_ros_test" \
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
