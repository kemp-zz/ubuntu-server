# 基础镜像 (明确指定 CUDA 11.8 + Ubuntu 22.04 开发环境)
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置全局环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61

# 第一阶段：安装基础系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git \
    cmake \
    ninja-build \
    libboost-dev \
    libeigen3-dev \
    libopenmpi-dev \
    python3-pip \
    python3-venv \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 第二阶段：安装 ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# 移除可能冲突的系统Python包（如blinker）
RUN apt-get update && apt-get remove -y python3-blinker \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# 第三阶段：一次性安装所有 Python 依赖（解决 NumPy 版本冲突）
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir \
    numpy==1.23.5 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    jupyterlab \
    nerfstudio \
    pypose \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    TCNN_CUDA_ARCHITECTURES=$TCNN_CUDA_ARCHITECTURES \
    pip3 install --no-cache-dir \
    git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 配置 ROS 环境
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}" \
    PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

# 设置工作目录
WORKDIR /workspace

# 启动 Jupyter Lab
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
