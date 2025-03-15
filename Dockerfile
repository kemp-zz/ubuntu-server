FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 环境变量配置（适配ROS2 Humble）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    NVIDIA_VISIBLE_DEVICES=all

# 配置NVIDIA CUDA官方仓库（关键修复步骤）
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
    mkdir -p /usr/share/keyrings && \
    curl -fsSL --retry 3 --retry-delay 2 https://nvidia.github.io/libnvidia-container/gpgkey | \
    gpg --batch --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
    curl -fsSL --retry 3 --retry-delay 2 https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 安装ROS2核心组件（2025-Q2稳定版）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装完整版ROS2及编译工具
RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools && \
    rosdep init && \
    rosdep update --include-eol-distros

# 安装nvblox依赖（CUDA 12.8适配版）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libpcl-dev \
        libopencv-dev \
        libeigen3-dev \
        libcudnn9-cuda-12 \ 
        libcudnn9-dev-cuda-12 \ 
        libcublas-12-8 \
        libcusparse-12-8 \
    && rm -rf /var/lib/apt/lists/*

# 构建nvblox工作空间
RUN mkdir -p /ros_ws/src && \
    git clone -b ros2 https://github.com/ethz-asl/nvblox.git /ros_ws/src/nvblox

# 配置构建参数（启用CUDA加速）
RUN cd /ros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DNVBLX_WITH_CUDA=ON \
            -DCMAKE_CUDA_ARCHITECTURES=80 

# 配置运行时环境
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# 启动命令（支持实时点云处理）
CMD ["bash", "-c", "source ~/.bashrc && ros2 launch nvblox_ros nvblox.launch.py"]
