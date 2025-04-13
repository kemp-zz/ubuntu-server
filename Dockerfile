# 基础镜像 (明确指定 CUDA 11.8 + Ubuntu 22.04 开发环境)
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置全局环境变量（新增调试模式开关）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    DEBUG_MODE=true 

# 第一阶段：安装基础系统依赖（添加分层日志）
RUN echo "[阶段1] 安装系统依赖" && \
    apt-get update && apt-get install -y --no-install-recommends \
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
    && echo "[阶段1] 完成" && \
    rm -rf /var/lib/apt/lists/*

# 第二阶段：安装 ROS 2 Humble（添加错误捕获）
RUN { \
    echo "[阶段2] 安装ROS"; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list; \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    python3-colcon-common-extensions; \
    } || { echo "[错误] ROS安装失败"; exit 1; } 
RUN rosdep init && rosdep update

# -------------------------
# PyTorch 核心依赖锁定（强制覆盖系统包）
RUN echo "[阶段3.1] 安装PyTorch (强制覆盖系统依赖)" && \
    pip3 install --no-cache-dir --ignore-installed \
    numpy==1.23.5 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    echo "[验证] PyTorch版本: $(python3 -c 'import torch; print(torch.__version__)')"

# -------------------------
# JupyterLab 独立安装（隔离工具链）
RUN { \
    echo "[阶段3.2] 安装JupyterLab"; \
    pip3 install --no-cache-dir jupyterlab==4.0.0; \
    } && echo "[验证] JupyterLab版本: $(jupyter lab --version)"

# -------------------------
# NeRFStudio 显式依赖（通过 pip 管理）
RUN pip3 install --no-cache-dir \
    jaxtyping==0.2.24 \
    rich==13.7.1 \
    tyro==0.7.2 \
    opencv-python-headless==4.8.0.76 \  
    plotly==5.18.0 \
    pillow==10.0.0

# -------------------------
# NeRFStudio 本体安装（禁用自动依赖）
RUN { \
    echo "[阶段3.3] 安装NeRFStudio (纯 pip 依赖链)"; \
    pip3 install --no-cache-dir --no-deps nerfstudio; \
    } | tee /var/log/nerfstudio-install.log

# -------------------------

# PyPose 安装（移除 apt 依赖）
RUN { \
    echo "[阶段3.4] 安装PyPose (纯 pip 环境)"; \
    pip3 install --no-cache-dir pypose; \
    } && [ "$DEBUG_MODE" = "true" ] && \
    python3 -c "import cv2, pypose; print(f'OpenCV版本: {cv2.__version__}, PyPose版本: {pypose.__version__}')"
# 显式声明CUDA路径（网页7）
ENV CUDA_HOME=/usr/local/cuda-11.8 \
    LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH

# 调整验证命令（网页3）
RUN echo "[阶段3.6] 验证CUDA环境" && \
    echo "CUDA_HOME: $CUDA_HOME" && \
    nvcc --version | grep "release 11.8" && \
    ls /usr/lib/x86_64-linux-gnu/libcudnn*  # 修正cuDNN路径   

# -------------------------
# 显式指定计算能力（网页4）
RUN echo "[阶段3.5] 编译tiny-cuda-nn (SM61)" && \
    pip3 install --no-cache-dir \
    git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch \
    --install-option="--archs=61" && \
    python3 -c "import torch, tinycudann as tcnn; \
    print(f'tiny-cuda-nn版本: {tcnn.__version__}, CUDA版本: {torch.version.cuda}')"
    
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}" \
    PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

WORKDIR /workspace
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
