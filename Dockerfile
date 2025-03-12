# ====================== Stage 1: 基础环境 ======================
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 系统级配置（网页1推荐）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=humble \
    VPI_VERSION=2.1.5 \
    ISAAC_WS=/isaac_ws

# 时区与语言配置（网页7实践）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 2: ROS2 核心 ======================
FROM base AS ros-core

# APT源配置（网页5安全实践）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    software-properties-common \
    curl gnupg2 && \
    # ROS源
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    # CUDA源
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | gpg --dearmor > /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64 /" > /etc/apt/sources.list.d/cuda.list

# ROS核心安装（网页3分层优化）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-desktop \
    python3-rosdep \
    python3-pip \
    libspdlog-dev && \
    # VPI安装与配置
    apt-get install -y libvpi2=$VPI_VERSION vpi3-dev && \
    echo "/opt/nvidia/vpi2/lib64" >> /etc/ld.so.conf.d/vpi.conf && \
    ldconfig && \
    # ROS初始化
    rosdep init && \
    rosdep update --include-eol-distros

# ====================== Stage 3: 构建环境 ======================
FROM base AS builder

# 继承ROS核心
COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=ros-core /usr/share/keyrings/* /usr/share/keyrings/
COPY --from=ros-core /etc/apt/sources.list.d/* /etc/apt/sources.list.d/

# 构建依赖（网页6最佳实践）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    ccache && \
    apt-get clean

# 代码克隆与验证（网页2安全实践）
WORKDIR $ISAAC_WS/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git && \
        [ -f "${repo}/package.xml" ] || { echo "Missing package.xml in ${repo}"; exit 1; }; \
    done

# 构建参数优化（网页4 CUDA适配）
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CUDA_ARCHITECTURES='80-real;86-real;89-virtual' \
                -Dvpi_DIR=/opt/nvidia/vpi3/lib/cmake/vpi \
                -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
                -DOPENSSL_ROOT_DIR=/usr/include/openssl"

# 构建执行（网页9缓存优化）
RUN --mount=type=cache,target=/root/.cache/ccache \
    . /opt/ros/humble/setup.sh && \
    cd $ISAAC_WS && \
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --cmake-args $CMAKE_ARGS

# ====================== Stage 4: 运行时镜像 ======================
FROM base

# 继承运行时组件
COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=builder $ISAAC_WS/install $ISAAC_WS/install
COPY --from=ros-core /opt/nvidia/vpi2 /opt/nvidia/vpi2

# 安全配置（网页10用户隔离）
RUN useradd -m -d /home/appuser -s /bin/bash appuser && \
    chown -R appuser:appuser $ISAAC_WS /opt/ros/humble

# 环境变量
ENV PATH="/home/appuser/.local/bin:$PATH" \
    LD_LIBRARY_PATH="/opt/nvidia/vpi2/lib64:$LD_LIBRARY_PATH" \
    HOME=/home/appuser

USER appuser
WORKDIR $HOME

CMD ["bash"]
