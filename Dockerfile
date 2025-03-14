# 基础镜像（保持原选择）
FROM ubuntu:18.04

# 环境变量预定义
ENV ASTRA_SDK_VERSION="2.1.3"
ENV ASTRA_SDK_ZIP="AstraSDK-v${ASTRA_SDK_VERSION}-Ubuntu-x86_64.zip"
ENV ASTRA_TARBALL="AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz"
ENV DOWNLOAD_URL="https://dl.orbbec3d.com/dist/astra/v${ASTRA_SDK_VERSION}/${ASTRA_SDK_ZIP}"

# 安装基础依赖（新增udev systemd依赖）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        unzip \
        udev \
        systemd-sysv \  # 关键修复项
        libusb-1.0-0-dev \
        libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# SDK下载与解压流程保持不变
WORKDIR /tmp
RUN wget -q ${DOWNLOAD_URL} && \
    unzip -q ${ASTRA_SDK_ZIP} && \
    tar -xzf ${ASTRA_TARBALL} --strip-components=1 && \
    rm ${ASTRA_SDK_ZIP}

# 安装脚本调整（新增udev规则调试）
RUN cd install && \
    sed -i 's/sudo //g' install.sh && \
    chmod +x install.sh && \
    echo "DEBUG: Udev rules content:" && \  # 调试信息
    cat ../lib/udev/rules.d/*.rules && \    # 验证规则文件
    ./install.sh --no-prompt && \
    rm -rf /tmp/*

# 更新环境变量（修复LD_LIBRARY_PATH警告）
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

# 使用替代方式加载udev规则（2025年推荐方案）
RUN mkdir -p /etc/udev/rules.d/ && \
    cp -f /usr/local/lib/udev/rules.d/*.rules /etc/udev/rules.d/ && \
    systemctl enable systemd-udevd && \  # 启用udev守护进程
    systemctl start systemd-udevd

# 运行时用户配置保持不变
RUN useradd -ms /bin/bash astra-user
USER astra-user
WORKDIR /home/astra-user
