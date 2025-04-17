FROM nvidia/cuda:11.8.0-devel-ubuntu20.04


# 环境变量设置
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    TCNN_CUDA_ARCHITECTURES="61" \
    CONDA_DIR=/opt/conda \
    PATH="/opt/conda/bin:$PATH"
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH \
    CUDA_HOME=/usr/local/cuda

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvcc-11-8 \
    cuda-cudart-dev-11-8 \
    libcublas-dev-11-8
# 阶段一：基础环境配置
# --------------------------------
# 安装 ROS Noetic 核心组件（合并 apt 操作减少层数）
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl software-properties-common \
    && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-rosdep \
        python3-vcstool \
        python3-catkin-tools \
        git ninja-build qtbase5-dev \
        ros-noetic-std-msgs \
        ros-noetic-message-generation \
        ros-noetic-geometry-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-actionlib-msgs \
        ros-noetic-rviz \
        ros-noetic-jsk-rviz-plugins \
        nano \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init \
    && rosdep fix-permissions \
    && chmod -R a+rwx /etc/ros/rosdep/sources.list.d \
    && sudo -u root rosdep update

# 阶段二：Miniconda 环境配置
# --------------------------------
# 安装 Miniconda（使用官方源）[6](@ref)
RUN apt-get update && apt-get install -y --no-install-recommends wget nano \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-py38_23.3.1-0-Linux-x86_64.sh -O miniconda.sh \
    && bash miniconda.sh -b -p $CONDA_DIR \
    && rm miniconda.sh \
    && conda clean -y --all

# 创建 Conda 环境并安装 PyTorch（版本精确匹配 CUDA 11.8）
RUN conda create --name nerfstudio python=3.8 -y \
    && conda install -n nerfstudio \
        pytorch==2.1.2 \
        torchvision==0.16.2 \
        pytorch-cuda=11.8 \
        -c pytorch -c nvidia -y \
    && conda install -n nerfstudio -c "nvidia/label/cuda-11.8.0" cuda-toolkit -y

# 阶段三：工作空间与依赖编译
# --------------------------------
# 创建工作空间并克隆 ROS 项目
RUN mkdir -p /catkin_ws/src \
    && cd /catkin_ws/src \
    && git clone https://github.com/leggedrobotics/radiance_field_ros \
    && git clone https://github.com/orbbec/ros_astra_camera.git

# 安装通用编译依赖（合并 apt 操作）
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential cmake \
    libgflags-dev libusb-1.0-0-dev libeigen3-dev libdw-dev \
    && rm -rf /var/lib/apt/lists/*

# 编译安装 libuvc（合并构建步骤）
RUN git clone https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cd /tmp/libuvc \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/libuvc

# 阶段四：Python 依赖安装
# --------------------------------
# 激活 Conda 环境安装 NerfStudio 依赖
RUN conda run -n nerfstudio pip install --upgrade pip setuptools==65.5.1 \
    && conda run -n nerfstudio pip install ninja \
    && conda run -n nerfstudio pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch \
    && conda run -n nerfstudio pip install -e /catkin_ws/src/radiance_field_ros

# 阶段五：Catkin 工作空间构建
# --------------------------------
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build"

# 复制 udev 规则（删除重载命令）
RUN cp /catkin_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/

# 最终启动命令（同时激活 ROS 和 Conda 环境）
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && conda activate nerfstudio && /bin/bash"]

