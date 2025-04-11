# 使用NVIDIA CUDA基础镜像
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    SHELL=/bin/bash \
    PYTHONVER=3.10

# 安装基础系统工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    ninja-build

# 设置ROS2仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS2核心组件
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep

# 初始化rosdep
RUN rosdep init && rosdep update

# 安装Python工具链
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y \
    python$PYTHONVER \
    python$PYTHONVER-dev \
    python3-pip \
    python3-venv

# 创建Python虚拟环境
RUN python$PYTHONVER -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# 安装Python依赖
RUN pip install --upgrade pip setuptools && \
    pip install \
    jupyterlab \
    numpy==1.24.4 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools

# 安装TinyCUDA-NN
RUN pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 安装图形相关依赖
RUN apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-6 \
    libxext6 \
    libxrender1 \
    ffmpeg \
    freeglut3-dev

# 配置JupyterLab
RUN mkdir -p /root/.jupyter/lab/workspaces && \
    echo "c.ServerApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.allow_root = True" >> /root/.jupyter/jupyter_server_config.py

# 设置ROS环境
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 启动脚本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
