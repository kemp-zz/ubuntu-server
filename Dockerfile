# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 环境配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    ROS_PYTHON_VERSION=3 \
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

# 系统基础配置
RUN apt-get update && \
    apt-get install -y locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer

# 安装基础工具并配置所有源
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        software-properties-common \
        curl gnupg2 lsb-release && \
    mkdir -p /usr/share/keyrings && \
    # ROS源
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    # CUDA源
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | gpg --dearmor > /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    # VPI源
    curl -fsSL https://repo.download.nvidia.com/ubuntu2204/x86_64/7fa2af80.pub | gpg --dearmor > /usr/share/keyrings/nvidia-vpi-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-vpi-keyring.gpg] https://repo.download.nvidia.com/ubuntu2204/x86_64 /" > /etc/apt/sources.list.d/nvidia-vpi.list && \
    apt-get clean

# 安装ROS核心组件和VPI
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    build-essential \
    cmake \
    git \
    vpi2-dev=2.1.5 \
    vpi2-libnvvpi=2.1.5 \
    libvpi2=2.1.5 \
    libspdlog-dev \
    python3-rosdep \
    python3-pip \
    libssl-dev \
    pkg-config \
    devscripts && \
    # VPI路径配置
    echo "/opt/nvidia/vpi2/lib64" >> /etc/ld.so.conf.d/vpi.conf && \
    ldconfig && \
    # ROS初始化
    mkdir -p /isaac_ws/src && \
    rosdep init && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths /isaac_ws/src --ignore-src -y && \
    apt-get clean

# Python虚拟环境配置
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir \
        setuptools==65.7.0 \
        flake8-blind-except \
        flake8-builtins \
        matplotlib \
        pandas \
        rosbags \
        boto3
ENV PATH="/opt/venv/bin:$PATH"

# Stage 3: Isaac构建环境
FROM base AS isaac-builder

# 继承配置
COPY --from=ros-installer /etc/apt/sources.list.d/* /etc/apt/sources.list.d/
COPY --from=ros-installer /usr/share/keyrings/* /usr/share/keyrings/
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /opt/nvidia/vpi2 /opt/nvidia/vpi2

# 安装构建依赖
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

# 代码克隆
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 构建参数
ENV CMAKE_ARGS="-Dvpi_DIR=/opt/nvidia/vpi2/lib/cmake/vpi \
                -Dspdlog_DIR=/usr/lib/cmake/spdlog \
                -DOPENSSL_ROOT_DIR=/usr \
                -DCMAKE_CUDA_ARCHITECTURES=80 \
                -DCMAKE_BUILD_TYPE=Release"

# 执行构建
RUN --mount=type=cache,target=/root/.cache/ccache \
    . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    colcon build \
        --symlink-install \
        --parallel-workers $(($(nproc) * 2)) \
        --cmake-args $CMAKE_ARGS

# Stage 4: 最终镜像
FROM base

# 继承运行时组件
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install
COPY --from=ros-installer /opt/venv /opt/venv

# 创建非特权用户
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble /opt/venv

ENV PATH="/opt/venv/bin:$PATH" \
    HOME=/home/appuser
USER appuser
WORKDIR $HOME

CMD ["bash"]
