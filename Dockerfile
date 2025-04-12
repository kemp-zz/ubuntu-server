# 使用NVIDIA CUDA开发镜像（包含完整的CUDA工具链）
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES=61 \
    SHELL=/bin/bash \
    PYTHONVER=3.10 \
    PATH="/opt/venv/bin:$PATH"

# 安装系统依赖（优化后的合并安装）
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    wget \
    curl \
    gnupg2 \
    udev \
    usbutils \
    lsb-release \
    build-essential \
    cmake \
    git \
    ninja-build \
    zlib1g-dev \
    libtcmalloc-minimal4 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-6 \
    libxext6 \
    libxrender1 \
    ffmpeg \
    freeglut3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# 设置ROS2仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
SHELL ["/bin/bash", "-c"]
# 安装ROS2核心组件（新增相机相关组件）
# 安装ROS2核心组件（新增相机相关组件及消息生成依赖）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rosbridge-server \
    ros-humble-image-geometry \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep && \
    rosdep init && rosdep update

# 安装Python工具链
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y \
    python$PYTHONVER \
    python$PYTHONVER-dev \
    python3-pip \
    python3-venv && \
    python$PYTHONVER -m venv /opt/venv

# 安装Python依赖（优化分步安装顺序）
RUN pip install --upgrade pip setuptools wheel
RUN pip install --extra-index-url https://download.pytorch.org/whl/cu118 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118
RUN pip install \
    jupyterlab \
    nerfstudio \
    pypose \
    numpy==1.24.4 \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools

RUN pip install "tinycudann @ git+https://github.com/NVlabs/tiny-cuda-nn@master#subdirectory=bindings/torch"

# 编译安装libuvc
RUN git clone https://github.com/libuvc/libuvc.git /root/libuvc && \
    cd /root/libuvc && \
    mkdir build && cd build && \
    cmake .. && make -j4 && make install && ldconfig

# 创建工作空间并克隆驱动代码
RUN mkdir -p /root/ros2_ws/src && \
    git clone https://github.com/orbbec/ros2_astra_camera.git /root/ros2_ws/src/ros2_astra_camera

# 安装udev规则（需提前创建scripts目录）
RUN cd /root/ros2_ws/src/ros2_astra_camera/astra_camera/scripts && \
    chmod +x install.sh && \
    ./install.sh 

# 构建ROS2工作空间
RUN cd /root/ros2_ws && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# 配置JupyterLab
RUN mkdir -p /root/.jupyter/lab/workspaces && \
    { echo "c.ServerApp.ip = '0.0.0.0'"; \
      echo "c.ServerApp.allow_root = True"; } > /root/.jupyter/jupyter_server_config.py

# 设置ROS环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# 启动脚本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
