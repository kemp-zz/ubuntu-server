# 使用官方CUDA 12.8开发镜像作为基础（已包含CUDA工具链）
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

# 设置非交互模式
ENV DEBIAN_FRONTEND=noninteractive

# 配置APT镜像源并安装基础工具
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 配置NVIDIA CUDA仓库（网页6推荐方式）
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | \
    gpg --dearmor -o /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg arch=amd64] \
    https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/ /" \
    > /etc/apt/sources.list.d/cuda.list && \
    echo "Package: *\nPin: origin developer.download.nvidia.com\nPin-Priority: 1002" \
    > /etc/apt/preferences.d/cuda-repository-pin-600

# 安装CUDA工具链（网页3推荐分层安装）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    cuda-toolkit-12-8=12.8.1-1 \
    cuda-libraries-dev-12-8=12.8.1-1 \
    cuda-drivers-550 \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 配置多层级环境变量（网页7最佳实践）
ENV PATH="/usr/local/cuda/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}" \
    CUDA_HOME="/usr/local/cuda" \
    NVIDIA_DRIVER_CAPABILITIES="compute,utility" \
    NVIDIA_VISIBLE_DEVICES="all"

# 验证安装（网页3和网页6的双重验证）
RUN nvcc --version | grep "release 12.8" && \
    nvidia-smi | grep "CUDA Version: 12.8"
