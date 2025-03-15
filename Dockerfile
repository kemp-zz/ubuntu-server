# 使用NVIDIA官方CUDA基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 设置环境变量
ENV LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    ISAAC_ROS_WS=/workspaces/isaac_ros-dev

# 安装基础工具链
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# 配置ROS官方仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS核心组件
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 初始化rosdep数据库
RUN rosdep init && \
    rosdep update

# 创建工作空间目录
WORKDIR ${ISAAC_ROS_WS}/src

# 克隆NVIDIA官方仓库
RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 配置rosdep源
COPY nvidia-isaac.yaml /etc/ros/rosdep/sources.list.d/
RUN echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" > /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list && \
    rosdep update

# 安装NVIDIA核心库
RUN mkdir -p /tmp/keys && \
    wget -P /tmp/keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    apt-key add /tmp/keys/jetson-ota-public.asc && \
    add-apt-repository "deb http://repo.download.nvidia.com/jetson/x86_64/ $(lsb_release -cs) r36.3 main" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libnvvpi3 \
    vpi3-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/keys

# 安装CV-CUDA（下载到/tmp）
RUN wget -P /tmp https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i /tmp/nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    wget -P /tmp https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i /tmp/nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb && \
    rm /tmp/*.deb

# 安装PyTorch官方包
RUN pip3 install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch \
    torchvision \
    torchaudio

# 克隆Nvblox仓库
WORKDIR ${ISAAC_ROS_WS}/src
RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# 构建工作空间
WORKDIR ${ISAAC_ROS_WS}
RUN rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

# 配置CUDA环境
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH} \
    PATH=/usr/local/cuda/bin:${PATH}

# 设置默认工作目录
WORKDIR ${ISAAC_ROS_WS}
CMD ["/bin/bash"]
