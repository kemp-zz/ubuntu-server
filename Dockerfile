# 使用官方的 Ubuntu 22.04 基础镜像
FROM ubuntu:22.04

# 设置环境变量以防止交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 更新软件包列表并安装基本的服务器软件包
RUN apt-get update && apt-get install -y \
    sudo \
    openssh-server \
    vim \
    net-tools \
    curl \
    wget \
    unzip \
    gnupg \
    lsb-release \
    && apt-get clean

# 创建一个新的用户并设置密码
RUN useradd -ms /bin/bash serveruser && echo 'serveruser:password' | chpasswd

# 允许新的用户使用 sudo
RUN usermod -aG sudo serveruser

# 设置 SSH 服务
RUN mkdir /var/run/sshd

# 允许 root 登录
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# 安装NVIDIA驱动和CUDA 12.8
RUN curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add - && \
    curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu22.04/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list && \
    apt-get update && apt-get install -y nvidia-driver-515 nvidia-cuda-toolkit

# 安装 ROS 2 Humble 并进行基本配置
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y ros-humble-desktop && \
    apt-get install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool && \
    apt-get clean

# 设置ROS 2环境变量
RUN echo "source /opt/ros/humble/setup.bash" >> /home/serveruser/.bashrc

# 暴露 SSH 端口
EXPOSE 22

# 启动 SSH 服务
CMD ["/usr/sbin/sshd", "-D"]
