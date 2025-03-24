FROM nvidia/cuda:12.8.0-devel-ubuntu22.04 AS base
ENV TZ=Asia/Shanghai \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8
   
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 安装基础工具和ROS2依赖（新增colcon和udev）
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    git \
    wget \
    build-essential \
    python3-pip \
    libx11-dev \
    usbutils \
    udev \
    libgl1-mesa-dev \
    libgtk-3-dev \    
    && rm -rf /var/lib/apt/lists/*

FROM base AS ros2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS2核心组件
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    libgflags-dev \
    ros-humble-image-geometry \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Orbbec设备规则
RUN mkdir -p /tmp/orbbec-udev \
    && cd /tmp/orbbec-udev \
    && curl -sSLO https://raw.githubusercontent.com/orbbec/OrbbecSDK_ROS2/main/orbbec_camera/scripts/99-obsensor-libusb.rules \
    && mkdir -p /etc/udev/rules.d \
    && cp 99-obsensor-libusb.rules /etc/udev/rules.d/ \
    && rm -rf /tmp/orbbec-udev

# 创建工作空间（路径改为/ros2_ws）
RUN mkdir -p ~/ros2_ws/src

# 安装libuvc
WORKDIR ~/ros2_ws/src
RUN git clone https://github.com/libuvc/libuvc \
    && cd libuvc \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && ldconfig

# 克隆相机驱动
WORKDIR ~/ros2_ws/src
RUN git clone --depth 1 https://github.com/orbbec/ros2_astra_camera.git

# 构建配置
WORKDIR ~/ros2_ws
RUN ["/bin/bash", "-c", "export PYTHONPATH=\"/usr/lib/python3/dist-packages:$PYTHONPATH\" \
    && python3 -c \"import em; print(f'Active empy: {em.__file__}')\" \
    && source /opt/ros/humble/setup.bash \
    && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"]
    
FROM ros2 AS open3d

# 添加Kitware APT源并安装CMake 3.31.6
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - > /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" > /etc/apt/sources.list.d/kitware.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends cmake=3.31.6-0kitware1ubuntu22.04.1

# 安装额外依赖
RUN apt-get install -y --no-install-recommends \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtbb-dev \
    libomp-dev \
    libc++-dev \
    libc++abi-dev \
    libc++1 \
    libc++abi1 \
    cuda-compat-12-8 \
    nano
  

RUN git clone --depth 1 https://github.com/isl-org/Open3D.git \
    && cd Open3D \
    && mkdir build \
    && cd build \
    && cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_CUDA_MODULE=ON \
        -DBUILD_GUI=ON \
        -DCMAKE_CUDA_ARCHITECTURES="61" \
        -DGLFW_USE_WAYLAND=OFF \
        -DCUDA_SEPARABLE_COMPILATION=ON \
        -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
    && make -j$(nproc) \
    && make install \
    && cd / \
    && rm -rf Open3D



# 设置工作空间并启动shell
WORKDIR ~/ros2_ws
CMD ["bash"]
