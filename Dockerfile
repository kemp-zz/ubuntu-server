# 使用最新的 nvidia/cuda 基础镜像
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

# 设置非交互模式
ENV DEBIAN_FRONTEND=noninteractive

# 更新APT源并安装必要的软件包
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    usbutils \
    ca-certificates \
    gnupg && \
    rm -rf /var/lib/apt/lists/*

# 添加 NVIDIA CUDA 仓库并安装 CUDA 12.8 工具包
RUN wget https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
    gpg --dearmor -o /usr/share/keyrings/cuda-archive-keyring.gpg 3bf863cc.pub && \
    echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/cuda.list && \
    apt-get update && \
    apt-get install -y \
    cuda-toolkit-12-8 \
    cuda-libraries-dev-12-8 \
    cuda-drivers && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 配置环境变量
ENV PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}} \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CUDA_HOME=/usr/local/cuda-12.8

# 验证安装
RUN nvcc --version && \
    nvidia-smi
