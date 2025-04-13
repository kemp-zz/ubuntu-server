# 基础镜像 (明确指定 CUDA 11.8 + Ubuntu 22.04 开发环境)
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置全局环境变量（新增调试模式开关）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    DEBUG_MODE=false  # [!NEW] 调试模式控制变量

# [!NEW] 添加调试工具链
RUN apt-get update && apt-get install -y --no-install-recommends \
    strace lsof procps nano && \
    rm -rf /var/lib/apt/lists/*

# 第一阶段：安装基础系统依赖（添加日志输出）
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

# 第二阶段：安装 ROS 2 Humble（添加错误检查）
RUN { \
    echo "[阶段2] 安装ROS"; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list; \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    python3-colcon-common-extensions; \
    } || { echo "[错误] ROS安装失败"; exit 1; }  # [!NEW] 错误捕获
RUN rosdep init && rosdep update

# [!NEW] 第三阶段前验证环境
RUN echo "[调试] CUDA架构: $TCNN_CUDA_ARCHITECTURES" && \
    nvidia-smi -L | tee /tmp/gpu-info.log

# 第三阶段：分步安装 Python 依赖（添加分步日志）
RUN echo "[阶段3] 升级pip工具链" && \
    pip3 install --no-cache-dir --upgrade pip setuptools wheel

# [!NEW] 安装PyTorch后验证numpy版本
RUN { \
    echo "[阶段3.1] 安装PyTorch"; \
    pip3 install --no-cache-dir \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118; \
    } && echo "[验证] numpy版本: $(pip3 list | grep numpy)"  # 防止隐式升级

RUN { \
    echo "[阶段3.2] 安装JupyterLab"; \
    pip3 install --no-cache-dir jupyterlab; \
    } || { echo "[错误] Jupyter安装失败"; [ "$DEBUG_MODE" = "true" ] && pip3 debug -v; exit 1; }  # [!NEW] 调试模式响应

RUN { \
    echo "[阶段3.3] 安装NeRFStudio"; \
    pip3 install --no-cache-dir nerfstudio; \
    } | tee /var/log/nerfstudio-install.log  # [!NEW] 日志重定向

# [!NEW] 安装PyPose前检查OpenCV兼容性
RUN if [ "$DEBUG_MODE" = "true" ]; then \
    echo "[调试] 检查OpenCV依赖"; \
    apt-get update && apt-get install -y pkg-config; \
    fi

RUN { \
    echo "[阶段3.4] 安装PyPose"; \
    pip3 install --no-cache-dir pypose; \
    } && [ "$DEBUG_MODE" = "true" ] && python3 -c "import cv2; print('OpenCV版本:', cv2.__version__)"  # [!NEW] 调试模式验证

# [!NEW] 显式传递CUDA架构参数
RUN echo "[阶段3.5] 安装tiny-cuda-nn (架构: $TCNN_CUDA_ARCHITECTURES)" && \
    TCNN_CUDA_ARCHITECTURES=$TCNN_CUDA_ARCHITECTURES \
    pip3 install --no-cache-dir \
    git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# [!NEW] 构建后诊断脚本
COPY docker-debug.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-debug.sh && \
    echo "export PATH=$PATH:/usr/local/bin" >> /etc/bash.bashrc

# 配置 ROS 环境
ENV ROS_PYTHON_VERSION=3 \
    PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}" \
    PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"

WORKDIR /workspace
CMD ["jupyter-lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
