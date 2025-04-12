# 使用NVIDIA CUDA开发镜像（包含完整的CUDA工具链）
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TCNN_CUDA_ARCHITECTURES="61" \
    SHELL=/bin/bash \
    PYTHONVER=3.10 \
    WORKSPACE=/root/ros2_ws \
    VENV_PATH=/opt/venv \
    SYSTEM_PYTHON_PATH=/usr/bin/python3

# 安装基础系统依赖
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
    libeigen3-dev \
    python3-empy && \
    rm -rf /var/lib/apt/lists/*

# 设置ROS2仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS2核心组件（使用系统Python）
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
    ros-humble-rosidl-default-runtime \
    ros-humble-builtin-interfaces \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep && \
    rosdep init && \
    rosdep update

# 创建Python虚拟环境（与系统Python隔离）
RUN apt-get update && apt-get install -y \
    python${PYTHONVER}-venv \
    python3-pip && \
    python${PYTHONVER} -m venv ${VENV_PATH} && \
    ln -s /usr/bin/python3 /usr/bin/python

# 安装PyTorch到虚拟环境（使用虚拟环境的pip）
RUN ${VENV_PATH}/bin/pip install --upgrade pip setuptools wheel && \
    ${VENV_PATH}/bin/pip install --extra-index-url https://download.pytorch.org/whl/cu118 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118

# 安装其他Python依赖到虚拟环境
RUN ${VENV_PATH}/bin/pip install \
    jupyterlab \
    nerfstudio \
    pypose \
    numpy==1.24.4 \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools

# 编译安装libuvc
WORKDIR /root
RUN git clone --depth 1 https://github.com/libuvc/libuvc.git && \
    mkdir libuvc/build && cd libuvc/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# 准备ROS工作空间
RUN mkdir -p ${WORKSPACE}/src && \
    git clone https://github.com/orbbec/ros2_astra_camera.git ${WORKSPACE}/src/ros2_astra_camera

# 安装udev规则（使用系统Python）
RUN cd ${WORKSPACE}/src/ros2_astra_camera/astra_camera/scripts && \
    chmod +x install.sh && \
    ./install.sh

# 构建ROS工作空间（强制使用系统Python）
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} && \
    PYTHONPATH= PATH=${SYSTEM_PYTHON_PATH}:${PATH} colcon build \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 配置环境变量隔离
ENV PATH="${VENV_PATH}/bin:${PATH}"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=\"${VENV_PATH}/lib/python${PYTHONVER}/site-packages:\${PYTHONPATH}\"" >> /root/.bashrc

# 剩余配置保持不变...

# 设置入口脚本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 设置默认工作目录
WORKDIR ${WORKSPACE}

ENTRYPOINT ["/entrypoint.sh"]
