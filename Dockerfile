# syntax=docker/dockerfile:1

# 基础镜像优化：使用 runtime + devel 混合镜像
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 as builder

# 第一阶段：系统级依赖安装（root 权限）
# ==============================================
ENV DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/opt/ros2_ws \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=61

# 创建带写入权限的目录结构（提前声明便于权限管理）
RUN mkdir -p /opt/venv /app ${ROS2_WS}/src && \
    # 创建非特权用户
    groupadd -g 1000 appuser && \
    useradd -u 1000 -g appuser -m appuser && \
    chown -R appuser:appuser /opt/venv /app ${ROS2_WS}

# 安装系统级依赖（保持原子性）
RUN --mount=type=cache,target=/var/cache/apt \
    # 安装基础工具链
    apt-get update && apt-get install -y --no-install-recommends \
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
    libboost-all-dev \
    libeigen3-dev \
    libcudnn8-dev \           
    && \
    # 配置 ROS 2 仓库（必须在同一个 RUN 中完成密钥操作）
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
    && \
    # 安装 ROS 2 核心组件（合并 apt 操作）
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && \
    # 初始化 rosdep（忽略可能的初始化错误）
    rosdep init || true \
    && rosdep update \
    && \
    # 清理（保留缓存挂载）
    rm -rf /var/lib/apt/lists/*

RUN python3.10 -m venv /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir -U pip setuptools wheel

# 第二阶段：应用级依赖安装（非特权用户）
# ==============================================
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV ROS2_WS=/opt/ros2_ws \
    PATH="/opt/venv/bin:$PATH" \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=61

# 从 builder 阶段复制必要文件
COPY --from=builder /opt/venv /opt/venv
COPY --from=builder /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list
COPY --from=builder /etc/ros/rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/20-default.list

# 复制应用文件（保持用户权限）
COPY --chown=appuser:appuser . /app
COPY --chown=appuser:appuser requirements.txt /tmp/
COPY --chown=appuser:appuser entrypoint.sh /entrypoint.sh

# 切换到非特权用户
USER appuser

# 安装 Python 依赖（带编译缓存）
RUN --mount=type=cache,target=/home/appuser/.cache/pip \
    pip install -r /tmp/requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    # 安装定制化 tiny-cuda-nn（关键修改点）
    CMAKE_ARGS="-DTCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES}" \
    pip install --no-cache-dir \
    --global-option="--build-option=--cuda-arch=${CUDA_ARCH}" \
    "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# 配置 shell 环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ~/.bashrc

# 验证安装
RUN python -c "import tinycudann as tcnn; print(f'TCNN arch: {tcnn.get_cuda_arch_flags()}')" && \
    python -c "import torch; print(f'PyTorch CUDA: {torch.cuda.get_arch_list()}')"

# 配置运行时
VOLUME ["${ROS2_WS}/src", "/app/data", "/home/appuser/.jupyter"]
WORKDIR /app
EXPOSE 8888
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
