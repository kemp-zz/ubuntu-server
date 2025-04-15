FROM pytorch/pytorch:2.6.0-cuda11.8-cudnn9-devel

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV TCNN_CUDA_ARCHITECTURES="61"

# 设置语言环境，安装 ROS Noetic 和依赖
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

# 创建并激活 Conda 环境，安装项目依赖
RUN conda create --name nerfstudio python=3.8 -y \
    && echo "conda activate nerfstudio" >> /root/.bashrc

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source activate nerfstudio && pip install --upgrade pip && pip install catkin_pkg && cd /catkin_ws/src/radiance_field_ros && pip install -e ."

# 安装 tiny-cuda-nn（如果项目需要）
RUN /bin/bash -c "source activate nerfstudio && pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# 构建 Catkin 工作空间
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build"

# 默认启动命令，激活环境和 ROS
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source activate nerfstudio && /bin/bash"]
