# 使用更精确的CUDA镜像标签
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# 环境变量配置优化
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    NVIDIA_VISIBLE_DEVICES=all \
    # 修复CUDA路径问题
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:/opt/ros/humble/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda/lib64/stubs:$LD_LIBRARY_PATH

# 第一阶段：精简系统依赖安装（移除冗余驱动安装）
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
    gcc-10 g++-10 \  
    cuda-toolkit-11-8 \
    && rm -rf /var/lib/apt/lists/*

# PyTorch安装优化（使用官方推荐安装方式）
RUN python3 -m venv /workspace/myenv && \
    /workspace/myenv/bin/pip install --upgrade pip setuptools wheel && \
    /workspace/myenv/bin/pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 \
    --index-url https://download.pytorch.org/whl/cu118

# 验证CUDA可用性（新增验证层）
RUN /workspace/myenv/bin/python -c "\
    import torch; \
    assert torch.cuda.is_available(), 'CUDA初始化失败！可能原因：\n'\
    '1. 容器运行时未加--gpus参数\n'\
    '2. 宿主机NVIDIA驱动不兼容\n'\
    '3. CUDA工具链安装错误\n'\
    '4. 内核模块加载问题'; \
    print('PyTorch CUDA验证通过:', torch.version.cuda)"

# tiny-cuda-nn编译优化
WORKDIR /workspace
RUN git clone --depth 1 --branch v1.7 https://github.com/NVlabs/tiny-cuda-nn.git && \
    cd tiny-cuda-nn && \
    mkdir build && cd build && \
    cmake .. \
    -DCMAKE_CUDA_ARCHITECTURES="61;70;75;80" \
    -DCMAKE_CXX_COMPILER=g++-10 \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc && \
    cmake --build . --config RelWithDebInfo -j $(nproc)

# 修复PyPose安装依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    ocl-icd-opencl-dev \ 
    libgl1-mesa-glx && \ 
    rm -rf /var/lib/apt/lists/*

# 分阶段安装依赖以优化缓存
RUN /workspace/myenv/bin/pip install --no-cache-dir \
    jupyterlab \
    nerfstudio \
    pypose \
    nvidia-pyindex  

# ROS安装优化
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    python3-colcon-common-extensions && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# 最终验证层
RUN /workspace/myenv/bin/python -c "\
    import torch, tinycudann; \
    print('CUDA设备数量:', torch.cuda.device_count()); \
    print('CUDA设备名称:', torch.cuda.get_device_name(0)); \
    print('tiny-cuda-nn版本:', tinycudann.__version__)"

# 启动配置优化
COPY nerfstudio_test.sh /usr/local/bin/nerfstudio_test.sh
RUN chmod +x /usr/local/bin/nerfstudio_test.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/myenv/bin/activate" >> /root/.bashrc

CMD ["bash", "-c", "source /workspace/myenv/bin/activate && /usr/local/bin/nerfstudio_test.sh && jupyter-lab --ip=0.0.0.0 --allow-root --no-browser --NotebookApp.token=''"]
