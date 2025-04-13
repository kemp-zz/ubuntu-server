FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置非交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 安装 ROS 依赖和系统依赖
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    wget \
    curl \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# 设置 ROS 环境变量
ENV ROS_DISTRO=humble
ENV ROS_VERSION=humble
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 创建并初始化 ROS 工作空间
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# 下载 radiance_field_ros 源码
RUN git clone https://github.com/leggedrobotics/radiance_field_ros.git src/radiance_field_ros

# 使用 rosdep 安装依赖
RUN rosdep update
RUN rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# 创建 conda 环境
RUN pip3 install conda
RUN conda create --name nerfstudio -y python=3.8
# 激活 conda 环境
SHELL ["conda", "run", "-n", "nerfstudio", "/bin/bash", "-c"]

# 安装 PyTorch 和 CUDA 工具包
RUN conda install pytorch==2.1.2 torchvision==0.16.2 pytorch-cuda=11.8 -c pytorch -c nvidia
RUN conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit

# 安装 tiny-cuda-nn
RUN pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 安装 radiance_field_ros
WORKDIR /root/ros2_ws/src/radiance_field_ros
RUN pip install -e .

# 构建 ROS 包
WORKDIR /root/ros2_ws
RUN colcon build

# 设置 ROS 环境变量，包括 conda 激活
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "conda activate nerfstudio" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# 设置工作目录
WORKDIR /root
