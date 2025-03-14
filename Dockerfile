# 使用官方 Ubuntu 18.04 镜像
FROM ubuntu:18.04

# 安装基础工具链
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
# 下载并解压 SDK（关键步骤）
# -------------------------------
# 1. 下载 ZIP 包
RUN wget -O AstraSDK.zip "https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Ubuntu-x86_64.zip"

# 2. 第一次解压（得到 .tar.gz 文件）
RUN unzip AstraSDK.zip && \
    rm AstraSDK.zip

# 3. 第二次解压（最终文件）
RUN tar -xzf AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz && \
    rm AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz

# -------------------------------
# 配置 udev 规则
# -------------------------------
# 创建目标目录（避免路径错误）
RUN mkdir -p /etc/udev/rules.d

# 写入规则（覆盖所有设备）
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="video"\n\
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0505", MODE="0666"\n\
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="060d", MODE="0666"\n\
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0407", MODE="0666"' \
> /etc/udev/rules.d/99-orbbec.rules

# -------------------------------
# 安装 SDK
# -------------------------------
# 确认解压路径（重要！）
WORKDIR /AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64/install

# 检查文件是否存在（调试用）
RUN ls -l

# 安装 SDK
RUN chmod +x install.sh && \
    ./install.sh && \
    udevadm control --reload-rules && \
    udevadm trigger

# -------------------------------
# 清理中间文件
# -------------------------------
WORKDIR /
RUN rm -rf /AstraSDK*

# 设置运行时环境变量
ENV LD_LIBRARY_PATH=/usr/local/lib/astra
CMD ["/bin/bash"]
