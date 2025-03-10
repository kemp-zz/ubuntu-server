# 阶段一：基础环境（含内核头文件）
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    git-core \
    build-essential \
    kmod \
    linux-headers-$(uname -r) \
    libglvnd-dev \
    ocl-icd-opencl-dev \
    && rm -rf /var/lib/apt/lists/*

# 禁用Nouveau驱动（关键步骤）
RUN echo "blacklist nouveau" >> /etc/modprobe.d/blacklist.conf && \
    echo "options nouveau modeset=0" >> /etc/modprobe.d/nvidia-blacklists-nouveau.conf

# 阶段二：NVIDIA驱动安装（兼容CUDA 12.8）
FROM base AS driver-install

ARG DRIVER_VERSION=550.54.15  # 官方推荐与CUDA 12.8配套的驱动版本
RUN wget https://cn.download.nvidia.com/XFree86/Linux-x86_64/${DRIVER_VERSION}/NVIDIA-Linux-x86_64-${DRIVER_VERSION}.run -O /tmp/NVIDIA.run && \
    chmod +x /tmp/NVIDIA.run && \
    /tmp/NVIDIA.run --silent \
        --no-kernel-module \
        --no-drm \
        --no-x-check \
        --install-libglvnd && \
    rm -f /tmp/NVIDIA.run

# 阶段三：CUDA Toolkit安装（含Blackwell架构支持）
FROM driver-install AS cuda-install

ENV CUDA_VERSION=12.8.0
ENV CUDA_PKG=cuda-${CUDA_VERSION}-local
RUN wget https://developer.download.nvidia.com/compute/cuda/${CUDA_VERSION}/local_installers/${CUDA_PKG}_linux.run && \
    sh ${CUDA_PKG}_linux.run --silent --toolkit --override && \
    echo "export PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}}" >> /etc/profile.d/cuda.sh && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> /etc/profile.d/cuda.sh && \
    rm -f ${CUDA_PKG}_linux.run

# 阶段四：最终运行时镜像
FROM nvidia/cuda:12.8.0-runtime-ubuntu22.04

# 复制驱动和CUDA组件
COPY --from=driver-install /usr/lib/x86_64-linux-gnu/libGL* /usr/lib/x86_64-linux-gnu/
COPY --from=driver-install /usr/lib/x86_64-linux-gnu/libnvidia* /usr/lib/x86_64-linux-gnu/
COPY --from=cuda-install /usr/local/cuda-12.8 /usr/local/cuda-12.8

# 环境变量配置
ENV PATH=/usr/local/cuda-12.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:/usr/local/cuda-12.8/lib:$LD_LIBRARY_PATH \
    CUDA_HOME=/usr/local/cuda-12.8 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility

# 验证安装
RUN nvidia-smi | grep "Driver Version: 550" && \
    nvcc --version | grep "release 12.8" && \
    nvidia-smi -L | grep "Blackwell"  # 确认Blackwell架构支持

# 多架构编译支持（可选）
RUN echo "CUDAARCHS=compute_120" >> /etc/environment

# 清理中间文件（减少镜像体积）
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
