
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 as builder

# 环境变量配置
ENV DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/opt/ros2_ws \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=sm_61 \
    VENV_PATH=/opt/venv

# 用户及权限管理
RUN <<EOF
    groupadd -g 1000 appuser && \
    useradd -m -u 1000 -g appuser -s /bin/bash appuser && \
    mkdir -p /cache/pip && \
    chown -R appuser:appuser /cache
EOF

# 系统依赖安装（带APT缓存优化）
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    <<EOF
    apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    python3.10 \
    python3.10-venv && \
    # 创建虚拟环境
    python3.10 -m venv $VENV_PATH && \
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

# 第二阶段：应用构建（非特权用户上下文）
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 环境变量继承
ENV ROS2_WS=/opt/ros2_ws \
    VENV_PATH=/opt/venv \
    PATH="$VENV_PATH/bin:$PATH" \
    TCNN_CUDA_ARCHITECTURES=61 \
    CUDA_ARCH=sm_61

# 复制基础配置
COPY --from=builder /etc/passwd /etc/passwd
COPY --from=builder /etc/group /etc/group
COPY --from=builder $VENV_PATH $VENV_PATH
COPY --from=builder /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list

# 应用文件复制（保留权限）
COPY --chown=appuser:appuser . /app
COPY --chown=appuser:appuser requirements.txt /tmp/
COPY --chown=appuser:appuser entrypoint.sh /entrypoint.sh

# 切换非特权用户
USER appuser
WORKDIR /app

# Python依赖安装（带分层缓存优化）
RUN --mount=type=cache,target=/cache/pip \
    <<EOF
    # 显式使用虚拟环境中的pip
    $VENV_PATH/bin/pip install -r /tmp/requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cu118 && \
    # 定制化编译 tiny-cuda-nn
    CMAKE_ARGS="-DTCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES}" \
    $VENV_PATH/bin/pip install --no-cache-dir \
    --global-option="--build-option=--gpu-architecture=${CUDA_ARCH}" \
    "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"
EOF

# 环境验证（显式使用虚拟环境Python）
RUN <<EOF
    $VENV_PATH/bin/python -c "import tinycudann; print(f'TCNN version: {tinycudann.__version__}')" && \
    $VENV_PATH/bin/python -c "import torch; print(f'PyTorch CUDA: {torch.cuda.get_arch_list()}')" && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/local_setup.bash" >> ~/.bashrc
EOF

# 运行时配置
VOLUME ["${ROS2_WS}/src", "/app/data"]
EXPOSE 8888
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
