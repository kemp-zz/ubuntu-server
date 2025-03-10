# 阶段一：基础环境（含内核头文件）
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    git-core \
    build-essential \
    ca-certificates \
    build-essential \
    kmod \
    wget \
    linux-headers-$(uname -r) \
    libglvnd-dev \
    ocl-icd-opencl-dev \
    && rm -rf /var/lib/apt/lists/*

FROM base AS repo-setup
ARG CUDA_VERSION=12.8
ARG CUDA_PATCH=1

# 添加 NVIDIA CUDA 仓库
RUN wget https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
    add-apt-repository "deb https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/ /" && \
    apt-get update

# 阶段三：驱动及 CUDA 安装
FROM repo-setup AS cuda-install

# 安装 CUDA 12.8.1 完整套件
RUN apt-get install -y \
    cuda-toolkit-12-8=12.8.1-1 \
    cuda-libraries-dev-12-8=12.8.1-1 \
    cuda-drivers-550 && \
    apt-get clean

# 阶段四：运行时环境
FROM ubuntu:22.04 AS runtime

# 复制 CUDA 运行时文件
COPY --from=cuda-install /usr/local/cuda-12.8 /usr/local/cuda-12.8

# 配置环境变量
ENV PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}} \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CUDA_HOME=/usr/local/cuda-12.8

# 验证安装
RUN nvcc --version | grep "release 12.8" && \
    nvidia-smi | grep "Driver Version: 550"p/*
