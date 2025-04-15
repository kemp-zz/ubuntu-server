FROM nvcr.io/nvidia/pytorch:22.11-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV TCNN_CUDA_ARCHITECTURES="61"

# 设置语言环境并安装 ROS Noetic 和依赖
RUN apt-get update && apt-get install -y locales curl software-properties-common \
    && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list \
    && apt-get update && apt-get install -y ros-noetic-desktop-full python3-rosdep python3-vcstool python3-catkin-tools git ninja-build qtbase5-dev \
       ros-noetic-std-msgs ros-noetic-message-generation ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-actionlib-msgs ros-noetic-rviz ros-noetic-jsk-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && rosdep fix-permissions && rosdep update

# 设置 ROS 环境变量
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# 创建工作空间并克隆项目
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/leggedrobotics/radiance_field_ros

# 安装 Python 和编译依赖
RUN apt-get update && apt-get install -y python3-pip python3-dev build-essential cmake git ninja-build \
    && python3 -m pip install --upgrade pip setuptools wheel ninja

# 克隆 tiny-cuda-nn 并编译安装

RUN pip3 install --upgrade  setuptools==65.5.1 && git clone --recursive https://github.com/NVlabs/tiny-cuda-nn.git /tmp/tiny-cuda-nn \
    && cd /tmp/tiny-cuda-nn/bindings/torch \
    && export CUDA_HOME=/usr/local/cuda \
    && export PATH=$CUDA_HOME/bin:$PATH \
    && export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH \
    && export TCNN_CUDA_ARCHITECTURES="61" \
    && python3 setup.py install \
    && rm -rf /tmp/tiny-cuda-nn

# 构建 Catkin 工作空间
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build"

# 默认启动命令，激活 ROS 环境
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && /bin/bash"]
