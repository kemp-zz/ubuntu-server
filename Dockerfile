# ====================== Stage 0: 全局配置 ======================
ARG ROS_DISTRO=humble
ARG CUDA_VERSION=12.8.0
ARG UBUNTU_VERSION=22.04

# ====================== Stage 1: 基础环境 ======================
FROM nvidia/cuda:${CUDA_VERSION}-base-ubuntu${UBUNTU_VERSION} AS base

# 系统级环境变量定义（必须显式声明）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=${ROS_DISTRO} \
    ISAAC_WS=/isaac_ws \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}"

# 基础系统配置（网页3安全实践）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        locales tzdata ca-certificates && \
    ln -snf "/usr/share/zoneinfo/$TZ" /etc/localtime && \
    echo "$TZ" > /etc/timezone && \
    locale-gen en_US.UTF-8 && \
    update-ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 2: ROS2 核心 ======================
FROM base AS ros-core

# APT源配置（网页6最佳实践）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        curl gnupg2 && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# ROS核心安装修正（网页1安全优化）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        python3-colcon-common-extensions \ 
        ros-${ROS_DISTRO}-rcl-logging-spdlog \
        ros-${ROS_DISTRO}-ament-cmake-auto \
        ros-${ROS_DISTRO}-rclcpp \
        libspdlog-dev \
        python3-rosdep && \  
    ln -sf /usr/bin/python3 /usr/bin/python && \
    pip3 install --upgrade setuptools wheel && \
    rosdep init || true && \
    rosdep update --include-eol-distros && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 3: 构建环境 ======================
FROM base AS builder

# 继承ROS核心（网页4多阶段构建实践）
COPY --from=ros-core /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}
COPY --from=ros-core /usr/lib/python3/dist-packages /usr/lib/python3/dist-packages
COPY --from=ros-core /usr/bin/colcon /usr/bin/colcon

# 构建工具链安装（网页5缓存优化）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        libssl-dev \
        ros-${ROS_DISTRO}-ament-cmake \
        cmake git libopencv-dev ccache && \
    ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++ && \
    rm -rf /var/lib/apt/lists/*

WORKDIR $ISAAC_WS/src

# 代码克隆优化（网页1子模块管理）
RUN git clone --branch main --recurse-submodules --depth 1 \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
    && { [ -f "isaac_ros_common/isaac_ros_common/package.xml" ] && \
         [ -d "isaac_ros_common/isaac_common" ]; } || \
       { echo "Missing core files"; exit 1; } \
    && git clone --branch main --recurse-submodules --depth 1 \
        https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# 构建参数配置（网页6编译优化）
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                -DBUILD_TESTING=OFF \
                -DCMAKE_CUDA_ARCHITECTURES='80-real;86-real;89-virtual' \
                -DCMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
                -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog"

# 构建执行（网页2分层优化）
RUN --mount=type=cache,target=/root/.cache/ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd $ISAAC_WS && \
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --cmake-args $CMAKE_ARGS && \
    find . -name '*.o' -delete

# ====================== Stage 4: 运行时镜像 ======================
FROM base

# 运行时依赖（网页1最小化镜像）
COPY --from=ros-core /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}
COPY --from=builder $ISAAC_WS/install $ISAAC_WS/install

# 安全配置（网页4非特权用户）
RUN useradd -m -d /home/appuser -s /bin/bash -u 1001 appuser && \
    chown -R appuser:appuser $ISAAC_WS /opt/ros/${ROS_DISTRO} && \
    chmod 750 $ISAAC_WS /opt/ros/${ROS_DISTRO}

# 环境隔离（网页3路径配置）
ENV PATH="/home/appuser/.local/bin:/opt/ros/${ROS_DISTRO}/bin:$PATH" \
    LD_LIBRARY_PATH="/opt/ros/${ROS_DISTRO}/lib:$LD_LIBRARY_PATH" \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:$PYTHONPATH"

USER appuser
WORKDIR /home/appuser
EXPOSE 5000/tcp 5005/udp
HEALTHCHECK --interval=30s --timeout=5s --retries=3 \
    CMD ros2 node list || exit 1

CMD ["bash"]
