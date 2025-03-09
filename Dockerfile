# 使用官方的 Ubuntu 22.04 基础镜像
FROM ubuntu:22.04

# 设置环境变量以防止交互提示
ENV DEBIAN_FRONTEND=noninteractive
# 在原apt安装段添加以下依赖
RUN apt-get update && \
    apt-get install -y \
    # 新增编译工具
    build-essential cmake libssl-dev libgl1-mesa-dev libprotobuf-dev protobuf-compiler \
    # 补充Python工具链
    python3-venv python3-dev python3-catkin-pkg \
    # 补充ROS构建工具
    ros-humble-ros-base ros-humble-rviz2 && \
    apt-get clean
# 添加ROS 2软件源并安装ROS 2 Humble和colcon工具
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ament-cmake-auto \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep && \
    apt-get clean

# 初始化rosdep
RUN rosdep init && rosdep update

# 创建一个新的用户并设置密码
RUN useradd -ms /bin/bash serveruser && echo 'serveruser:password' | chpasswd

# 允许新的用户使用 sudo
RUN usermod -aG sudo serveruser

# 在创建用户后添加
RUN mkdir -p /home/serveruser/.ros && \
    chown -R serveruser:serveruser /home/serveruser && \
    echo "serveruser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/serveruser

# 设置 SSH 服务
RUN mkdir /var/run/sshd && \
    sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# 替换原NVIDIA安装段
RUN curl -fsSL https://nvidia.github.io/nvidia-docker/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-docker.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-docker.gpg] https://nvidia.github.io/nvidia-docker/ubuntu22.04/nvidia-docker.list" > /etc/apt/sources.list.d/nvidia-docker.list && \
    apt-get update && apt-get install -y \
    nvidia-driver-535 nvidia-cuda-toolkit-12-8 \
    && apt-get clean

# 设置ROS 2环境变量
RUN echo "source /opt/ros/humble/setup.bash" >> /home/serveruser/.bashrc

# 替换原克隆构建段
USER serveruser
WORKDIR /home/serveruser

# 分步骤构建每个仓库
RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
    cd isaac_ros_common && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    cd isaac_ros_nvblox && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"

RUN git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git && \
    cd isaac_ros_visual_slam && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths . --ignore-src -r -y && \
             colcon build --symlink-install"
# 设置 NVIDIA Isaac ROS 环境变量
RUN echo "source /home/serveruser/install/setup.bash" >> /home/serveruser/.bashrc

# 暴露 SSH 端口
EXPOSE 22

# 启动 SSH 服务
CMD ["/usr/sbin/sshd", "-D"]
