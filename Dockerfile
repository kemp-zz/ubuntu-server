# 使用官方的 Ubuntu 22.04 基础镜像
FROM ubuntu:22.04

# 设置环境变量以防止交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 第一阶段：基础环境配置
RUN apt-get update && \
    apt-get install -y \
    sudo \
    openssh-server \
    vim \
    net-tools \
    curl \
    wget \
    unzip \
    gnupg \
    lsb-release \
    git \
    python3-pip \
    python3-apt && \
    apt-get clean

# 创建用户（必须在任何USER指令前完成）
RUN useradd -m -s /bin/bash serveruser && \
    echo 'serveruser:password' | chpasswd && \
    usermod -aG sudo serveruser && \
    echo "source /opt/ros/humble/setup.bash" >> /home/serveruser/.bashrc

# 安装ROS 2 Humble
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ament-cmake-auto \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep && \
    apt-get clean

# 修复rosdep初始化
RUN rosdep init || true && \
    rosdep update && \
    apt-get install -y \
    ros-humble-std-msgs \
    ros-humble-cv-bridge

# NVIDIA容器工具链安装
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && \
    apt-get update && \
    apt-get install -y nvidia-container-toolkit

# SSH配置
RUN mkdir /var/run/sshd && \
    sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# 第二阶段：以普通用户构建
USER serveruser
WORKDIR /home/serveruser

# 克隆并构建isaac_ros_common
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
    cd isaac_ros_common && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 构建nvblox插件
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    cd isaac_ros_nvblox && \
    bash -c "source ../isaac_ros_common/install/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 构建visual_slam插件
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git && \
    cd isaac_ros_visual_slam && \
    bash -c "source ../isaac_ros_common/install/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 最终配置
USER root
RUN echo "source /home/serveruser/isaac_ros_common/install/setup.bash" >> /home/serveruser/.bashrc && \
    echo "source /home/serveruser/isaac_ros_nvblox/install/setup.bash" >> /home/serveruser/.bashrc && \
    echo "source /home/serveruser/isaac_ros_visual_slam/install/setup.bash" >> /home/serveruser/.bashrc

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]
