# 基础镜像 (明确指定 CUDA 11.8 + cuDNN + Ubuntu 22.04 开发环境)
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# 设置全局环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    NVIDIA_VISIBLE_DEVICES=all

# 设置 CUDA 环境变量
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

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
    gcc-9 g++-9 \
    nvidia-driver-535 \
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

# 第三阶段：创建 Python 虚拟环境
RUN python3 -m venv /workspace/myenv

# 激活虚拟环境并安装依赖
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --upgrade pip setuptools wheel"

# 安装 PyTorch (CUDA 11.8)
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --no-cache-dir \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118"

# 步骤1：单独安装 JupyterLab
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --no-cache-dir jupyterlab"

# 步骤2：安装 NeRFStudio（需验证 CUDA 兼容性）
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --no-cache-dir nerfstudio || echo 'NeRFStudio 安装失败，检查 CUDA 兼容性'"

# 步骤 3：安装 PyPose（注意 OpenCV 依赖）
RUN apt-get update && apt-get install -y --no-install-recommends libopencv-dev && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --no-cache-dir pypose"

# 安装 tiny-cuda-nn
RUN /bin/bash -c "source /workspace/myenv/bin/activate \
    && pip install --no-cache-dir numpy==1.24.4"

WORKDIR /workspace
RUN git clone --recursive https://github.com/NVlabs/tiny-cuda-nn.git

WORKDIR /workspace/tiny-cuda-nn
RUN mkdir build

WORKDIR /workspace/tiny-cuda-nn/build
RUN cmake .. -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CXX_COMPILER=g++-9 -DCMAKE_C_COMPILER=gcc-9

RUN cmake --build . --config RelWithDebInfo -j

WORKDIR /workspace/tiny-cuda-nn/bindings/torch
RUN /bin/bash -c "source /workspace/myenv/bin/activate && python setup.py install"

# 配置 ROS 环境
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}"
ENV PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

# 设置工作目录
WORKDIR /workspace

# 复制测试脚本并使其可执行
COPY nerfstudio_test.sh /usr/local/bin/nerfstudio_test.sh
RUN chmod +x /usr/local/bin/nerfstudio_test.sh

# 启动 Jupyter Lab
CMD ["bash", "-c", "source /workspace/myenv/bin/activate && /usr/local/bin/nerfstudio_test.sh && jupyter-lab --ip=0.0.0.0 --allow-root --no-browser --NotebookApp.token=''"]
