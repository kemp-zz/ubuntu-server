# -------------------------------
# 基础镜像（显式指定小版本）
# -------------------------------
FROM ubuntu:18.04

# -------------------------------
# 安装工具链（合并层 + 清理缓存）
# -------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        unzip \
        udev \
        libusb-1.0-0-dev \
        libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# SDK 处理流程优化（减少中间层）
# -------------------------------
RUN wget -O AstraSDK.zip "https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Ubuntu-x86_64.zip" && \
    unzip AstraSDK.zip "AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz" && \
    tar -xzf AstraSDK*.tar.gz -C /opt && \
    mv /opt/AstraSDK* /opt/AstraSDK && \
    rm -rf AstraSDK.zip AstraSDK*.tar.gz
# 确保 install.sh 存放在构建上下文中的 install_scripts/ 子目录
COPY --chmod=755 install_scripts/install.sh /opt/AstraSDK/install/
# -------------------------------
# 安装流程强化（增加预验证）
# -------------------------------
WORKDIR /opt/AstraSDK/install
COPY --chmod=755 install.sh .
# 验证关键文件存在性
RUN ls -l /opt/AstraSDK/orbbec-usb.rules && \
    echo "INSTALLER PATH: $(realpath install.sh)" && \
    ./install.sh && \
    udevadm control --reload-rules && \
    udevadm trigger

# -------------------------------
# 运行时优化（非 Root 用户 + 环境变量）
# -------------------------------
RUN useradd -ms /bin/bash astrauser && \
    chown -R astrauser:astrauser /opt/AstraSDK
USER astrauser
ENV LD_LIBRARY_PATH=/usr/local/lib/astra:/opt/AstraSDK/lib
WORKDIR /home/astrauser
CMD ["/bin/bash"]
