# 基础镜像 (明确指定 CUDA 11.8 + cuDNN + Ubuntu 22.04 开发环境)
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# 设置全局环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    NVIDIA_VISIBLE_DEVICES=all \
    CONDA_DIR=/opt/conda \
    PATH=$CONDA_DIR/bin:$PATH

# 第一阶段：安装基础系统依赖和Conda
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
    curl \
    wget \
    gnupg2 \
    lsb-release \
    gcc-9 g++-9 \
    && rm -rf /var/lib/apt/lists/*

ENV CONDA_DIR=/opt/conda \
    PATH=$CONDA_DIR/bin:$PATH

# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-py38_23.11.0-2-Linux-x86_64.sh -O miniconda.sh \
    && bash miniconda.sh -b -p $CONDA_DIR \
    && rm miniconda.sh \
    && conda clean -ya

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

# 第三阶段：创建Conda环境
RUN conda create -n nerfenv python=3.8 -y \
    && echo "conda activate nerfenv" >> ~/.bashrc

# 安装 PyTorch (CUDA 11.8)
RUN bash -c "source activate nerfenv \
    && pip install --no-cache-dir \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118"

# 安装 NeRFStudio 和依赖
RUN bash -c "source activate nerfenv \
    && pip install --no-cache-dir nerfstudio"

# 安装 PyPose 和 OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
RUN bash -c "source activate nerfenv \
    && pip install --no-cache-dir pypose"

# 安装 tiny-cuda-nn
RUN bash -c "source activate nerfenv \
    && pip install --no-cache-dir numpy==1.24.4"
WORKDIR /workspace
RUN git clone --recursive https://github.com/NVlabs/tiny-cuda-nn.git
WORKDIR /workspace/tiny-cuda-nn
RUN mkdir build
WORKDIR /workspace/tiny-cuda-nn/build
RUN bash -c "source activate nerfenv \
    && cmake .. -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CXX_COMPILER=g++-9 -DCMAKE_C_COMPILER=gcc-9"
RUN cmake --build . --config RelWithDebInfo -j
WORKDIR /workspace/tiny-cuda-nn/bindings/torch
RUN bash -c "source activate nerfenv \
    && python setup.py install"

# 安装 NerfBridge
WORKDIR /workspace
RUN git clone https://github.com/nerfbridge/nerfbridge.git
WORKDIR /workspace/nerfbridge
RUN bash -c "source activate nerfenv \
    && pip install -e ."

# 配置 ROS 环境
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}" \
    PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

# 设置工作目录
WORKDIR /workspace

# 启动命令
CMD ["bash", "-c", "source activate nerfenv && /bin/bash"]
