# 使用多阶段构建（网页3、网页6推荐）
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04 AS builder

# 阶段一：基础配置
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# 阶段二：CUDA仓库配置
FROM builder AS repo-config
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | \
    gpg --dearmor -o /usr/share/keyrings/cuda-archive-keyring.gpg && \
    chmod 600 /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg arch=amd64] \
    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" \
    > /etc/apt/sources.list.d/cuda.list

# 阶段三：安装组件
FROM repo-config AS installer
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    cuda-toolkit-12-8=12.8.1-1 \
    cuda-libraries-dev-12-8=12.8.1-1 \
    cuda-drivers-550 \
    && apt-get clean

# 最终阶段
FROM nvidia/cuda:12.8.0-runtime-ubuntu22.04
COPY --from=installer /usr/local/cuda-12.8 /usr/local/cuda-12.8
ENV PATH="/usr/local/cuda-12.8/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}" \
    NVIDIA_DRIVER_CAPABILITIES="compute,utility"
