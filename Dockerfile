# ====================== Stage 1: 基础环境 ======================
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 系统级配置（最小化环境变量）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=humble \
    ISAAC_WS=/isaac_ws

# 时区与语言配置（安全加固）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    locales tzdata ca-certificates && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    locale-gen en_US.UTF-8 && \
    update-ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 2: ROS2 核心 ======================
FROM base AS ros-core

# APT源配置（安全镜像源）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    curl gnupg2 && \
    # ROS源密钥环加固
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    # CUDA源签名验证
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | \
    gpg --dearmor > /usr/share/keyrings/cuda-archive-keyring.gpg

# ROS核心安装（最小化依赖）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3.10 python3-pip python3.10-venv \  
        python3-colcon-common-extensions \   
        python3-rosdep python3-rosinstall-generator \
        python3-dev \
        libssl-dev \
        libspdlog-dev && \
    # Python 版本管理
    ln -sf /usr/bin/python3.10 /usr/bin/python && \
    ln -sf /usr/bin/python3.10 /usr/bin/python3 && \
    # ROS 初始化
    rosdep init || true && \
    rosdep update --include-eol-distros && \
    # 清理阶段
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 3: 构建环境 ======================
FROM base AS builder

# 继承ROS核心（安全复制）
COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=ros-core /usr/share/keyrings/ /usr/share/keyrings/
COPY --from=ros-core /etc/apt/sources.list.d/ /etc/apt/sources.list.d/

# 构建依赖（缓存优化）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        libssl-dev libffi-dev \
        python3-pip python3-setuptools \
        ros-humble-ament-cmake \
        cmake git libopencv-dev ccache && \ 
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \   
    ln -s /usr/bin/ccache /usr/local/bin/gcc && \
    ln -s /usr/bin/ccache /usr/local/bin/g++

# 代码克隆与验证（完整性检查）
WORKDIR $ISAAC_WS/src
RUN git clone --depth 1 --branch main --shallow-submodules \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
    [ -f "isaac_ros_common/isaac_common/package.xml" ] || { echo "Missing package.xml"; exit 1; }

RUN git clone --depth 1 --branch main \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    [ -f "isaac_ros_nvblox/isaac_ros_nvblox/package.xml" ] || { echo "Missing nvblox package"; exit 1; }

RUN git clone --depth 1 --branch main \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git && \
    [ -f "isaac_ros_visual_slam/isaac_ros_visual_slam/package.xml" ] || { echo "Missing visual_slam package"; exit 1; }

# 构建参数优化（安全编译选项）
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CUDA_ARCHITECTURES='80-real;86-real;89-virtual' \
                -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
                -DOPENSSL_ROOT_DIR=/usr/lib/x86_64-linux-gnu \
                -DOPENSSL_CRYPTO_LIBRARY=/usr/lib/x86_64-linux-gnu/libcrypto.so \
                -DOPENSSL_INCLUDE_DIR=/usr/include/openssl"

# 构建执行（安全缓存隔离）
RUN --mount=type=cache,target=/root/.cache/ccache \
    . /opt/ros/humble/setup.sh && \
    cd $ISAAC_WS && \
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --cmake-args $CMAKE_ARGS && \
    # 构建后清理
    find . -name '*.o' -delete && \
    rm -rf /var/lib/apt/lists/*

# ====================== Stage 4: 运行时镜像 ======================
FROM base

# 安全继承运行时组件
COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=builder $ISAAC_WS/install $ISAAC_WS/install

# 非特权用户配置（网页5/7安全实践）
RUN useradd -m -d /home/appuser -s /bin/bash -u 1001 appuser && \
    chown -R appuser:appuser $ISAAC_WS /opt/ros/humble && \
    chmod 700 $ISAAC_WS /opt/ros/humble

# 环境变量隔离
ENV PATH="/home/appuser/.local/bin:$PATH" \
    HOME=/home/appuser \
    LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"

# 安全启动配置
USER appuser
WORKDIR $HOME
EXPOSE 5000/tcp 5005/udp
HEALTHCHECK --interval=30s --timeout=5s --retries=3 \
    CMD ros2 node list || exit 1

CMD ["bash"]
