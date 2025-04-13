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

# [!NEW] 第三阶段前验证环境
RUN echo "[调试] CUDA架构: $TCNN_CUDA_ARCHITECTURES" && \
    nvidia-smi -L | tee /tmp/gpu-info.log

# 第三阶段：分步安装 Python 依赖（关键组件依赖隔离）
# -------------------------
# [!NEW] 锁定PyTorch依赖树
RUN echo "[阶段3.1] 安装PyTorch (强制锁定numpy版本)" && \
    pip3 install --no-cache-dir \
    numpy==1.23.5 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    echo "[验证] numpy版本: $(pip3 list | grep numpy)"  
# -------------------------
# [!NEW] JupyterLab独立安装（隔离UI工具链）
RUN { \
    echo "[阶段3.2] 安装JupyterLab"; \
    pip3 install --no-cache-dir jupyterlab; \
    } && echo "[验证] JupyterLab核心组件: $(pip3 list | grep jupyterlab)" 

# -------------------------
# [!NEW] NeRFStudio安装（依赖PyTorch CUDA兼容性检查）
RUN { \
    echo "[阶段3.3] 安装NeRFStudio"; \
    pip3 install --no-cache-dir nerfstudio; \
    } | tee /var/log/nerfstudio-install.log && \
    python3 -c "from nerfstudio.utils import colormaps; print('NeRFStudio CUDA支持:', torch.cuda.is_available())" 

# -------------------------
# [!NEW] PyPose安装（OpenCV依赖控制）
RUN { \
    echo "[阶段3.4] 安装PyPose (检查OpenCV兼容性)" && \
    pip3 install --no-cache-dir pypose; \
    } && [ "$DEBUG_MODE" = "true" ] && python3 -c "import cv2; print('OpenCV版本:', cv2.__version__)" 

# -------------------------
# [!NEW] tiny-cuda-nn显式编译（确保CUDA架构参数传递）
RUN echo "[阶段3.5] 安装tiny-cuda-nn (架构: $TCNN_CUDA_ARCHITECTURES)" && \
    TCNN_CUDA_ARCHITECTURES=$TCNN_CUDA_ARCHITECTURES \
    pip3 install --no-cache-dir \
    git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch && \
    python3 -c "import tinycudann as tcnn; print(f'tiny-cuda-nn版本: {tcnn.__version__}')"  

# 配置 ROS 环境
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}" \
    PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

WORKDIR /workspace
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
