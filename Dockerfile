# 基础镜像选择（使用官方LTS版本）
FROM ubuntu:18.04

# 环境变量预定义（减少硬编码）
ENV ASTRA_SDK_VERSION="2.1.3"
ENV ASTRA_SDK_ZIP="AstraSDK-v${ASTRA_SDK_VERSION}-Ubuntu-x86_64.zip"
ENV ASTRA_TARBALL="AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz"
ENV DOWNLOAD_URL="https://dl.orbbec3d.com/dist/astra/v${ASTRA_SDK_VERSION}/${ASTRA_SDK_ZIP}"

# 安装基础依赖（合并层 + 缓存清理）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        unzip \
        udev \
        libusb-1.0-0-dev \
        libgl1-mesa-glx \
        # 安装脚本依赖
        bash \
        coreutils \
    && rm -rf /var/lib/apt/lists/*

# SDK下载与解压（原子化操作）
WORKDIR /tmp
RUN wget -q ${DOWNLOAD_URL} && \
    unzip -q ${ASTRA_SDK_ZIP} && \
    tar -xzf ${ASTRA_TARBALL} --strip-components=1 && \
    rm ${ASTRA_SDK_ZIP} 

# 安装脚本执行（包含权限修复）
RUN cd install && \
    # 修复可能的权限问题
    sed -i 's/sudo //g' install.sh && \
    chmod +x install.sh && \
    ./install.sh --no-prompt && \
    # 清理安装残留
    rm -rf /tmp/*

# Udev规则更新（确保设备访问权限）
RUN udevadm control --reload-rules && \
    udevadm trigger

# 运行时配置（非root用户运行）
RUN useradd -ms /bin/bash astra-user
USER astra-user
WORKDIR /home/astra-user

# 验证路径配置
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
