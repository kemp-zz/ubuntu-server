FROM ubuntu:18.04 

# 安装基础工具链（国际源）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \         
    wget \               
    unzip \               
    libusb-1.0-0-dev \      
    libgl1-mesa-glx \       
    && rm -rf /var/lib/apt/lists/*

# 更新证书信任链
RUN update-ca-certificates --fresh

# 下载并解压 SDK（原厂地址）
RUN wget https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Ubuntu-x86_64.zip && \
    unzip AstraSDK-v2.1.3-Ubuntu-x86_64.zip && \
    tar -xzf AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz

RUN printf 'SUBSYSTEM="usb", ATTR{idVendor}="2bc5", MODE="0666", GROUP="video"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="0505", MODE="0666"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="060d", MODE="0666"\n\
SUBSYSTEM="usb", ATTR{idVendor}="2bc5", ATTR{idProduct}="0407", MODE="0666"\n' \
> /etc/udev/rules.d/99-orbbec.rules

# 安装SDK（绝对路径）
WORKDIR /AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64/install
RUN chmod +x install.sh && \
    ./install.sh && \
    udevadm control --reload-rules && \
    udevadm trigger

# 配置用户组
RUN groupadd -r video || true && \
    usermod -aG video root

# 清理中间文件
WORKDIR /
RUN rm -rf AstraSDK* *.zip

# 设置运行时环境
ENV LD_LIBRARY_PATH=/usr/local/lib/astra
CMD ["/bin/bash"]
