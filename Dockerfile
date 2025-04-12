# 基础镜像
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置环境变量和非交互模式
ENV DEBIAN_FRONTEND=noninteractive

# 更新系统并安装基本工具
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    git \
    curl \
    wget \
    python3-pip \
    python3-venv \
    cmake \
    libopenmpi-dev \
    zlib1g-dev && \
    rm -rf /var/lib/apt/lists/*

# 安装 ROS 2 (Humble)
ARG ROS_DISTRO=humble
RUN apt-get update && apt-get install -y gnupg2 lsb-release && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-core && \
    rm -rf /var/lib/apt/lists/*

# 设置 ROS 环境变量
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PATH="/opt/ros/${ROS_DISTRO}/bin:$PATH"
ENV PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages:$PYTHONPATH"

# 安装 Python 包：JupyterLab、PyPose、PyTorch 等
RUN pip3 install --upgrade pip && pip3 install \
    jupyterlab \
    nerfstudio \
    pypose \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
    tiny-cuda-nn

# 设置工作目录并启动 JupyterLab
WORKDIR /workspace
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root"]
