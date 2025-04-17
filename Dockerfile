

FROM linuxserver/blender:4.4.0

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y --fix-missing \
        ros-humble-desktop-full \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rosdep init \
    && rosdep update


# 安装相机驱动依赖
RUN apt-get update && apt-get install -y \
    libgflags-dev \
    ros-humble-image-geometry \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev

# 安装 libuvc
RUN apt-get update && apt-get install -y \
    git cmake wget 
    

RUN git clone https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cd /tmp/libuvc \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && make install \
    && ldconfig
# 创建 ROS2 工作空间
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# 下载 ros2_astra_camera 源码
RUN git clone https://github.com/orbbec/ros2_astra_camera.git src/ros2_astra_camera
RUN git clone https://github.com/emanuelbuholzer/ros2_blender.git src/ros2_blender

# 安装 libusb 规则
RUN bash /ros2_ws/src/ros2_astra_camera/astra_camera/scripts/install.sh

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release



# 设置环境变量
ENV ROS_DOMAIN_ID=38

# CMD ["ros2", "launch", "astra_camera", "astra.launch.xml"]
