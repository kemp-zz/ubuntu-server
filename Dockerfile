# 使用NVIDIA CUDA基础镜像
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \  # RTX 1070对应计算能力6.1
    SHELL=/bin/bash \
    PYTHONVER=3.10 \
    PATH="/opt/venv/bin:$PATH"

# 安装系统依赖（合并图形和编译工具）
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    ninja-build \
    zlib1g-dev \                # tcnn编译必需
    libtcmalloc-minimal4 \       # 内存优化
    libgl1-mesa-glx \           # OpenGL支持
    libgl1-mesa-dri \
    libx11-6 \
    libxext6 \
    libxrender1 \
    ffmpeg \                    # 视频处理
    freeglut3-dev && \
    rm -rf /var/lib/apt/lists/*

# 设置ROS2仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS2核心组件
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep && \
    rosdep init && rosdep update

# 安装Python工具链
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y \
    python$PYTHONVER \
    python$PYTHONVER-dev \
    python3-pip \
    python3-venv && \
    python$PYTHONVER -m venv /opt/venv

# 安装Python依赖（优化分步安装顺序）
RUN pip install --upgrade pip setuptools wheel
RUN pip install --extra-index-url https://download.pytorch.org/whl/cu118 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118
RUN pip install \
    jupyterlab \
    nerfstudio \
    pypose \
    numpy==1.24.4 \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools
RUN pip install "tcnn @ git+https://github.com/NVlabs/tiny-cuda-nn@master#subdirectory=bindings/torch"

# 配置JupyterLab
RUN mkdir -p /root/.jupyter/lab/workspaces && \
    { echo "c.ServerApp.ip = '0.0.0.0'"; \
      echo "c.ServerApp.allow_root = True"; } > /root/.jupyter/jupyter_server_config.py

# 设置ROS环境
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 启动脚本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
