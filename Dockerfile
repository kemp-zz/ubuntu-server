FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS builder

# 环境变量设置
ENV ROS_DISTRO=humble
ENV ISAAC_ROS_WS=/opt/isaac_ros_ws
WORKDIR $ISAAC_ROS_WS

# 安装核心工具链（2025年Ubuntu 22.04适配版本）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    git \
    software-properties-common \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 安装ROS 2 Humble（官方指定版本）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ament-cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 安装vcs工具（2025.03版）
RUN pip3 install -U "git+https://github.com/dirk-thomas/vcstool@2023.10.0"

# ============== 阶段2：Isaac ROS Common安装 ==============
RUN mkdir -p src && \
    curl -sSL https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/ros2.repos -o ros2.repos && \
    vcs import src < ros2.repos

# 安装Common依赖（官方2025年最新依赖列表）
RUN apt-get update && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -y \
      --from-paths \
        src/ros-visualization/ \
        src/uxr/client/ \
        src/isaac_ros_common/ \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# 构建Common组件（优化编译参数）
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --merge-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES=75  # 根据实际GPU架构调整


RUN curl -sSL https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/ros2.repos -o nvblox.repos && \
    vcs import src < nvblox.repos

# 安装nvBlox专用依赖（包含CUDA加速库）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libgflags-dev \
    libgoogle-glog-dev \
    libspdlog-dev \
    cuda-nvcc-12-8 \
    && rm -rf /var/lib/apt/lists/*

# 构建nvBlox组件（启用CUDA加速）
RUN . install/setup.sh && \
    colcon build \
      --merge-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES=75 \
        -DNVBLOX_USE_CUDA=ON


FROM nvidia/cuda:12.8.0-base-ubuntu22.04

ENV ROS_DISTRO=humble
ENV ISAAC_ROS_WS=/opt/isaac_ros_ws
COPY --from=builder $ISAAC_ROS_WS/install $ISAAC_ROS_WS/install


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    libspdlog-dev \
    && rm -rf /var/lib/apt/lists/*

# 配置环境变量（持久化设置）
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $ISAAC_ROS_WS/install/setup.bash" >> ~/.bashrc

WORKDIR $ISAAC_ROS_WS
CMD ["/bin/bash"]
