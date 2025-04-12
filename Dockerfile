# syntax=docker/dockerfile:1

# 基础镜像优化：使用统一基础镜像避免环境差异
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 as builder

# 第一阶段：系统级依赖安装（root 权限）
# ==============================================
ENV DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/opt/ros2_ws \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=sm_61  # 修正架构参数格式

# 用户及权限管理
RUN <<EOF
    # 创建带主目录的用户
    groupadd -g 1000 appuser && \
    useradd -m -u 1000 -g appuser -s /bin/bash appuser && \
    # 创建统一缓存目录
    mkdir -p /cache/pip && \
    chown -R appuser:appuser /cache
EOF

# 系统依赖安装（带APT缓存）
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    <<EOF
    apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    git \
    python3.10 \
    python3.10-venv \
    libopenmpi-dev \
    libboost-all-dev && \
    # ROS 2 安装
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-colcon-common-extensions && \
    # 清理
    rm -rf /var/lib/apt/lists/*
EOF

# 创建 Python 虚拟环境
RUN python3.10 -m venv /opt/venv && \
    /opt/venv/bin/pip install -U pip setuptools wheel

# 第二阶段：应用构建（非特权用户上下文）
# ==============================================
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04  # 保持开发镜像以支持编译

ENV ROS2_WS=/opt/ros2_ws \
    PATH="/opt/venv/bin:$PATH" \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=sm_61

# 复制用户信息及必要文件
COPY --from=builder /etc/passwd /etc/passwd
COPY --from=builder /etc/group /etc/group
COPY --from=builder /opt/venv /opt/venv
COPY --from=builder /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list

# 应用文件复制
COPY --chown=appuser:appuser . /app
COPY --chown=appuser:appuser requirements.txt /tmp/
COPY --chown=appuser:appuser entrypoint.sh /entrypoint.sh

# 用户上下文及缓存配置
USER appuser
WORKDIR /app

# 安装 Python 依赖（带分层缓存）
RUN --mount=type=cache,target=/cache/pip \
    <<EOF
    # 主依赖安装
    pip install -r /tmp/requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    # 定制化编译 tiny-cuda-nn
    CMAKE_ARGS="-DTCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES}" \
    pip install --no-cache-dir \
    --global-option="--build-option=--gpu-architecture=${CUDA_ARCH}" \
    "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"
EOF

# 环境配置及验证
RUN <<EOF
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ~/.bashrc && \
    # 安装验证
    python -c "import tinycudann; print(f'TCNN version: {tinycudann.__version__}')" && \
    python -c "import torch; print(f'PyTorch CUDA: {torch.cuda.get_arch_list()}')"
EOF

# 运行时配置
VOLUME ["${ROS2_WS}/src", "/app/data"]
EXPOSE 8888
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
