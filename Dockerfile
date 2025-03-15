FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 设置环境变量
ENV LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    ISAAC_ROS_WS=/workspaces/isaac_ros-dev

# 安装基础依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 设置ROS 2仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2核心组件
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 初始化rosdep
RUN rosdep init && \
    rosdep update

# 创建工作空间
WORKDIR ${ISAAC_ROS_WS}/src
RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 配置NVIDIA rosdep源
COPY nvidia-isaac.yaml /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml
RUN echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" > /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list && \
    rosdep update

# 安装NVIDIA核心库
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    add-apt-repository "deb http://repo.download.nvidia.com/jetson/x86_64/ $(lsb_release -cs) r36.3 main" && \
    apt-get update && apt-get install -y --no-install-recommends \
    libnvvpi3 \
    vpi3-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装CV-CUDA
RUN wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb && \
    rm *.deb

# 安装PyTorch
RUN pip3 install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch \
    torchvision \
    torchaudio

# 克隆Nvblox仓库
WORKDIR ${ISAAC_ROS_WS}/src
RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# 安装依赖并构建
WORKDIR ${ISAAC_ROS_WS}
RUN rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# 设置环境变量
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH} \
    PATH=/usr/local/cuda/bin:${PATH}

# 设置默认工作目录
WORKDIR ${ISAAC_ROS_WS}
CMD ["/bin/bash"]
