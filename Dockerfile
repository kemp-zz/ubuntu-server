# 阶段一：基础环境 + CUDA 12.8 Toolkit
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    kmod \
    build-essential \
    linux-headers-generic \
    git \
    nano \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 阶段二：安装NVIDIA驱动460.32.03（兼容CUDA 12.8的最低要求）
FROM base AS nvidia-driver

ARG DRIVER_VERSION=460.32.03
RUN wget https://us.download.nvidia.com/XFree86/Linux-x86_64/${DRIVER_VERSION}/NVIDIA-Linux-x86_64-${DRIVER_VERSION}.run && \
    chmod +x NVIDIA-Linux-*.run && \
    ./NVIDIA-Linux-*.run --silent --no-questions --ui=none --no-nouveau-check --install-libglvnd && \
    rm NVIDIA-Linux-*.run

# 阶段三：安装CUDA 12.8 Toolkit（Blackwell GPU原生支持）
FROM nvidia-driver AS cuda
ARG CUDA_VERSION=12.8
RUN wget https://developer.download.nvidia.com/compute/cuda/${CUDA_VERSION}/local_installers/cuda_${CUDA_VERSION}_linux.run && \
    sh cuda_*_linux.run --silent --toolkit --override && \
    rm cuda_*_linux.run

# 阶段四：最终镜像构建
FROM base
COPY --from=cuda /usr/local/cuda-12.8 /usr/local/cuda-12.8
COPY --from=nvidia-driver /usr/lib/x86_64-linux-gnu/libGL* /usr/lib/x86_64-linux-gnu/
COPY --from=nvidia-driver /usr/lib/x86_64-linux-gnu/libnvidia* /usr/lib/x86_64-linux-gnu/

ENV PATH=/usr/local/cuda-12.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:/usr/local/cuda-12.8/lib:$LD_LIBRARY_PATH \
    CUDA_HOME=/usr/local/cuda-12.8

RUN nvidia-smi | grep "Driver Version: 460" && \
    nvcc --version | grep "release 12.8" && \
    nvcc -gencode arch=compute_120,code=compute_120 -V
