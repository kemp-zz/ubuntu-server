# 使用官方的 Ubuntu 22.04 基础镜像
FROM ubuntu:22.04

# 设置环境变量以防止交互提示
ENV DEBIAN_FRONTEND=noninteractive

# ----------------- 基础依赖安装 -----------------
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
    build-essential \
    cmake \
    libssl-dev \
    libgl1-mesa-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python3-venv \
    python3-dev \
    python3-catkin-pkg && \
    apt-get clean

# ----------------- ROS 2 Humble 安装 -----------------
# 先配置ROS软件源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ament-cmake-auto \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep && \
    apt-get clean

# ----------------- 用户权限配置 -----------------
RUN useradd -ms /bin/bash serveruser && \
    echo 'serveruser:password' | chpasswd && \
    usermod -aG sudo serveruser && \
    mkdir -p /home/serveruser/.ros && \
    chown -R serveruser:serveruser /home/serveruser

# ----------------- NVIDIA 驱动安装 -----------------
RUN curl -fsSL https://nvidia.github.io/nvidia-docker/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-docker.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-docker.gpg] https://nvidia.github.io/nvidia-docker/ubuntu22.04/nvidia-docker.list" > /etc/apt/sources.list.d/nvidia-docker.list

RUN apt-get update && \
    apt-get install -y \
    nvidia-driver-535 \
    nvidia-cuda-toolkit-12-8 && \
    nvidia-smi --query-gpu=driver_version --format=csv

# ----------------- ROS环境初始化 -----------------
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> /home/serveruser/.bashrc

# ----------------- SSH服务配置 -----------------
RUN mkdir /var/run/sshd && \
    sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# ----------------- 用户空间构建 -----------------
USER serveruser
WORKDIR /home/serveruser

# 克隆并构建 isaac_ros_common
RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
    cd isaac_ros_common && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 安装 nvblox 插件
RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    cd isaac_ros_nvblox && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 安装 visual_slam 插件
RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git && \
    cd isaac_ros_visual_slam && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

# 环境变量配置
RUN echo "source /home/serveruser/install/setup.bash" >> /home/serveruser/.bashrc

# 暴露端口
EXPOSE 22

# 启动命令
CMD ["/usr/sbin/sshd", "-D"]
