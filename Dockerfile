# ============== 构建阶段 ==============
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS builder

# 环境预配置
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai \
    ROS_DISTRO=humble \
    ISAAC_ROS_WS=/opt/isaac_ros_ws

# 时区配置（网页2推荐写法）
RUN echo "Asia/Shanghai" > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

# 核心工具链安装（网页5最佳实践）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    git \
    software-properties-common \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble安装（网页6官方方案）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ament-cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 安装精确版本vcstool（网页3要求）
RUN python3 -m pip install "vcstool==0.3.0" --no-cache-dir

# ============== Isaac ROS Common安装 ==============
WORKDIR $ISAAC_ROS_WS

# 增强YAML处理流程（替代yamllint方案）
RUN python3 -m pip install --no-cache-dir "ruamel.yaml>=0.17.32"

# 带哈希校验的仓库下载（网页5优化方案）
RUN mkdir -p src && \
    curl --retry 5 --retry-all-errors --retry-delay 10 -sSL \
    https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/ros2.repos -o ros2.repos && \
    python3 -c "import hashlib; h=hashlib.sha256(open('ros2.repos','rb').read()).hexdigest(); assert h == 'a1b2c3d4...', 'Checksum mismatch'" && \
    python3 -c "from ruamel.yaml import YAML; YAML(typ='safe').load(open('ros2.repos'))" && \
    vcs import src < ros2.repos

# CUDA 12.8专用依赖（网页3关键配置）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libgflags-dev \
    libgoogle-glog-dev \
    libspdlog-dev \
    cuda-nvcc-12-8=12.8.0.1-1 \
    cuda-toolkit-12-8=12.8.0.1-1 \
    && rm -rf /var/lib/apt/lists/*

# 依赖安装优化（网页7解决方案）
RUN apt-get update && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -y \
      --from-paths \
        src/ros-visualization/ \
        src/uxr/client/ \
        src/isaac_ros_common/ \
      --ignore-src \
    --skip-keys "libopencv-dev libopencv-contrib-dev" && \
    apt-get clean

# 构建参数优化（网页4最佳实践）
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --merge-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES=75 \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc

# ============== 运行时阶段 ==============
FROM nvidia/cuda:12.8.0-base-ubuntu22.04

ENV ROS_DISTRO=humble \
    ISAAC_ROS_WS=/opt/isaac_ros_ws \
    CUDA_HOME=/usr/local/cuda-12.8 \
    PATH=/usr/local/cuda-12.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH

COPY --from=builder $ISAAC_ROS_WS/install $ISAAC_ROS_WS/install

# 最小运行时依赖（网页1精简方案）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    libspdlog-dev \
    && rm -rf /var/lib/apt/lists/*

# 环境持久化配置（网页2推荐方法）
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $ISAAC_ROS_WS/install/setup.bash" >> ~/.bashrc

WORKDIR $ISAAC_ROS_WS
CMD ["/bin/bash"]
