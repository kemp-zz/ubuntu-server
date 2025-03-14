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
# SDK 处理流程优化（单层构建减少中间文件）
# -------------------------------
RUN wget -O AstraSDK.zip "https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Ubuntu-x86_64.zip" && \
    unzip AstraSDK.zip "AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz" && \
    tar -xzf AstraSDK*.tar.gz -C /opt && \
    mv /opt/AstraSDK-* /opt/AstraSDK && \
    rm -rf AstraSDK.zip AstraSDK*.tar.gz

# -------------------------------
# 安装流程强化（路径预创建 + 权限控制）
# -------------------------------
RUN mkdir -p /opt/AstraSDK/install && \
    chmod 755 /opt/AstraSDK/install

COPY --chmod=755 install_scripts/install.sh /opt/AstraSDK/install/

# -------------------------------
# 安装脚本执行验证
# -------------------------------
WORKDIR /opt/AstraSDK/install
RUN test -f install.sh && \
    { echo "Installer validation passed"; } && \
    ./install.sh && \
    udevadm control --reload-rules && \
    udevadm trigger

# -------------------------------
# 运行时优化（非 Root 用户 + 环境变量）
# -------------------------------
RUN useradd -ms /bin/bash astrauser && \
    chown -R astrauser:astrauser /opt/AstraSDK && \
    find /opt/AstraSDK -type d -exec chmod 755 {} \; && \
    find /opt/AstraSDK -type f -exec chmod 644 {} \;

USER astrauser
ENV LD_LIBRARY_PATH=/usr/local/lib/astra:/opt/AstraSDK/lib
WORKDIR /home/astrauser
CMD ["/bin/bash"]
