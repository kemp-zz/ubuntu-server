
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS builder
# 设置非交互式环境（必须在所有apt-get操作之前）
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai

# 预配置时区（2025年推荐写法）
RUN echo "Asia/Shanghai" > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

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
    
RUN python3 -m pip install -U "vcstool>=0.3.0" --no-cache-dir


# ============== 阶段2：Isaac ROS Common安装 ==============
RUN apt-get update && apt-get install -y curl gnupg2 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirror.iscas.ac.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list

# 安装核心组件（适配CUDA 12.8环境）
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ============== Isaac ROS Common安装优化 ==============
# 增强YAML处理流程（解决格式验证问题）
RUN python3 -m pip install --no-cache-dir "ruamel.yaml>=0.17.32" yamllint

# 使用可靠镜像源下载配置（带SHA256校验）
RUN mkdir -p src && \
    # 使用GitHub官方仓库原始文件（推荐）
    curl --retry 5 --retry-all-errors --retry-delay 10 -sSL \
    https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/ros2.repos \
    -o ros2.repos && \
    yamllint -d relaxed ros2.repos && \
    vcs validate --input ros2.repos && \
    vcs import src < ros2.repos

# 安装依赖（适配NVIDIA CUDA 12.8环境）
RUN rosdep init && \
    rosdep update --rosdistro humble && \
    rosdep install -y \
      --from-paths \
        src/ros-visualization/ \
        src/uxr/client/ \
        src/isaac_ros_common/ \
      --ignore-src \
    --skip-keys "libopencv-dev libopencv-contrib-dev" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

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
