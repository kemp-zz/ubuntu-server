FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# 设置非交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 设置语言环境为 UTF-8
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# 添加 ROS Noetic 软件源
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-noetic.list

# 安装 ROS Noetic 和相关工具
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    wget \
    curl \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && rosdep update

# 设置 ROS 环境变量
ENV ROS_DISTRO=noetic
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 创建并初始化 ROS 工作空间
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# 下载 radiance_field_ros 源码
RUN git clone https://github.com/leggedrobotics/radiance_field_ros.git src/radiance_field_ros

# 下载 jsk_visualization 源码
RUN git clone https://github.com/jsk-ros-pkg/jsk_visualization.git src/jsk_visualization

# 使用 rosdep 安装依赖，忽略无法解析的键
RUN rosdep install -y --from-paths src --ignore-src --skip-keys="jsk_rviz_plugins" --rosdistro $ROS_DISTRO

# 构建 jsk-rviz-plugins
WORKDIR /root/catkin_ws/src/jsk_visualization/jsk_rviz_plugins
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 构建 radiance_field_ros
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 创建 conda 环境
RUN pip3 install conda
RUN conda create --name nerfstudio -y python=3.8

# 激活 conda 环境并安装 PyTorch 和 CUDA 工具包
SHELL ["conda", "run", "-n", "nerfstudio", "/bin/bash", "-c"]
RUN conda install pytorch==2.1.2 torchvision==0.16.2 pytorch-cuda=11.8 -c pytorch -c nvidia
RUN conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit

# 安装 tiny-cuda-nn
RUN pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 安装 radiance_field_ros
WORKDIR /root/catkin_ws/src/radiance_field_ros
RUN pip install -e .

# 设置工作目录和环境变量
WORKDIR /root
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "conda activate nerfstudio" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
