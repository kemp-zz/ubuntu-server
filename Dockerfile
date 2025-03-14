# -------------------------------
# 基础镜像
# -------------------------------
FROM ubuntu:18.04

# -------------------------------
# 安装基础工具链
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
# 下载并解压 SDK（仅处理 Ubuntu18.04）
# -------------------------------
# 1. 下载 ZIP 包（外层压缩）
RUN wget -O AstraSDK.zip "https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Ubuntu-x86_64.zip"

# 2. 第一次解压：解压 ZIP 得到 Ubuntu18.04 的 .tar.gz 文件
RUN unzip AstraSDK.zip "AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz" && \
    rm AstraSDK.zip

# 3. 第二次解压：仅解压 Ubuntu18.04 的 SDK 到固定路径
RUN tar -xzf AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz -C /opt && \
    mv /opt/AstraSDK* /opt/AstraSDK && \
    rm AstraSDK*.tar.gz

# -------------------------------
# 配置 udev 规则
# -------------------------------
RUN echo 'SUBSYSTEM="usb", ATTR{idVendor}="2bc5", MODE="0666", GROUP="video"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="0505", MODE="0666"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="060d", MODE="0666"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="0407", MODE="0666"' \
> /etc/udev/rules.d/99-orbbec.rules

# -------------------------------
# 安装 SDK
# -------------------------------
WORKDIR /opt/AstraSDK/install

# 验证文件存在性（关键调试步骤）
RUN ls -l && \
    echo "INSTALLER PATH: $(pwd)/install.sh"

# 执行安装脚本
RUN chmod +x install.sh && \
    ./install.sh && \
    udevadm control --reload-rules && \
    udevadm trigger

# -------------------------------
# 清理中间文件
# -------------------------------
WORKDIR /
RUN rm -rf /opt/AstraSDK*.tar.gz

# -------------------------------
# 设置运行时环境变量
# -------------------------------
ENV LD_LIBRARY_PATH=/usr/local/lib/astra
CMD ["/bin/bash"]
