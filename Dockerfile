# 基础镜像 (明确指定ubuntu 22.04)
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# 第一阶段：安装系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git \
    curl \
    wget \
    python3-pip \
    python3-venv \
    cmake \
    libopenmpi-dev \
    zlib1g-dev \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 第二阶段：安装 ROS 2 (官方源版本)
RUN apt-get update && apt-get install -y curl gnupg2 \
    # 添加官方ROS仓库
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    # 初始化 rosdep
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# 第三阶段：配置 Python 环境
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir \
    jupyterlab \
    nerfstudio \
    pypose \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
    # 显式安装 tiny-cuda-nn 的 PyTorch 绑定
    git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 配置 ROS 环境变量
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages:$PYTHONPATH \
    PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH

# 设置工作目录
WORKDIR /workspace

# 启动命令
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root", "--NotebookApp.token=''"]
