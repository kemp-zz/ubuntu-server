# syntax=docker/dockerfile:1

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 创建非特权用户（提前声明可写目录）
RUN mkdir -p /opt/venv /app ${ROS2_WS:-/opt/ros2_ws}/src && \
    groupadd -g 1000 appuser && \
    useradd -u 1000 -g appuser -m appuser && \
    chown -R appuser:appuser /opt/venv /app ${ROS2_WS:-/opt/ros2_ws}

# 环境变量配置
ENV DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/opt/ros2_ws \
    SHELL=/bin/bash \
    TCNN_CUDA_ARCHITECTURES=61

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    gnupg2 \
    lsb-release \
    python3.10 \
    python3.10-dev \
    python3.10-venv \
    libopenmpi-dev \
    libboost-dev \
    libeigen3-dev \
    libcudnn8-dev \
    cuda-toolkit-11-8 \      
    libcublas-dev-11-8 \     
    libcurand-dev-11-8 \      
    && rm -rf /var/lib/apt/lists/*

# 配置ROS 2仓库（root权限操作）
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS 2基础组件
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# 配置Python环境（保持root权限操作）
RUN python3.10 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip setuptools wheel

# 切换用户
USER appuser

# 安装Python依赖
COPY --chown=appuser:appuser requirements.txt /tmp/
RUN /opt/venv/bin/pip install -r /tmp/requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cu118 \
    && /opt/venv/bin/pip install \
    "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# 配置ROS工作空间环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ~/.bashrc

# 配置入口点
COPY --chown=appuser:appuser entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 卷和目录配置
VOLUME ["${ROS2_WS}/src", "/app", "/home/appuser/.jupyter/lab"]
WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
