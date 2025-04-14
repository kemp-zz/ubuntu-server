FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# 设置非交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 设置语言环境为 UTF-8
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 添加 ROS Noetic 软件源
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list

# 安装 ROS Noetic 和相关工具
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-catkin-tools \
    git wget curl ninja-build qtbase5-dev \
    ros-noetic-std-msgs ros-noetic-message-generation \
    ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
    ros-noetic-actionlib-msgs ros-noetic-rviz \
    ros-noetic-jsk-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && rosdep fix-permissions && rosdep update

# 设置 ROS 环境变量
ENV ROS_DISTRO=noetic
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 创建 Catkin 工作空间
RUN mkdir -p /catkin_ws/src

# 克隆项目仓库 (保留)
WORKDIR /catkin_ws/src
RUN git clone https://github.com/leggedrobotics/radiance_field_ros

# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 \
    && rm Miniconda3-latest-Linux-x86_64.sh

# 配置 Conda 环境变量
ENV PATH="/opt/miniconda3/bin:$PATH" \
    TCNN_CUDA_ARCHITECTURES="61"

# 创建环境并安装依赖
RUN conda create -n nerfstudio python=3.8 -y \
    && conda install -n nerfstudio -y \
    pytorch==2.1.2 torchvision==0.16.2 pytorch-cuda=11.8 \
    -c pytorch -c nvidia \
    && conda install -n nerfstudio -c "nvidia/label/cuda-11.8.0" cuda-toolkit -y

# 升级 pip 并安装依赖
RUN conda run -n nerfstudio pip install --upgrade pip \
    && conda run -n nerfstudio pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# 安装项目依赖 (保留)
WORKDIR /catkin_ws/src/radiance_field_ros
RUN conda run -n nerfstudio pip install -e . \
    && conda run -n nerfstudio pip install catkin_pkg

# 构建 Catkin 工作空间
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# 设置默认命令
CMD ["/bin/bash", "-c", "source /opt/miniconda3/etc/profile.d/conda.sh && conda activate nerfstudio && /bin/bash"]
