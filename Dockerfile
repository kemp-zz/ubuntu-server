FROM ubuntu:18.04

# 环境变量预定义
ENV ASTRA_SDK_VERSION="2.1.3"
ENV ASTRA_SDK_ZIP="AstraSDK-v${ASTRA_SDK_VERSION}-Ubuntu-x86_64.zip"
ENV ASTRA_TARBALL="AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz"
ENV DOWNLOAD_URL="https://dl.orbbec3d.com/dist/astra/v${ASTRA_SDK_VERSION}/${ASTRA_SDK_ZIP}"

# 安装基础依赖（仅保留必要组件）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        unzip \
        udev \
        libusb-1.0-0-dev \
        libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# SDK下载与解压流程
WORKDIR /tmp
RUN wget -q ${DOWNLOAD_URL} && \
    unzip -q ${ASTRA_SDK_ZIP} && \
    tar -xzf ${ASTRA_TARBALL} --strip-components=1 && \
    rm ${ASTRA_SDK_ZIP}

# 手动安装udev规则（2025年推荐方法）
RUN mkdir -p /etc/udev/rules.d/ && \
    find . -name '*.rules' -exec cp {} /etc/udev/rules.d/ \; && \
    udevadm control --reload || true && \
    udevadm trigger --action=add || true

# 安装脚本调整（绕过systemd依赖）
RUN cd install && \
    sed -i \
        -e 's/sudo //g' \
        -e '/systemctl/d' \
        install.sh && \
    chmod +x install.sh && \
    ./install.sh --no-prompt && \
    rm -rf /tmp/*

# 环境变量配置
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

# 运行时配置（必须启用特权模式）
RUN useradd -ms /bin/bash astra-user
USER astra-user
WORKDIR /home/astra-user
