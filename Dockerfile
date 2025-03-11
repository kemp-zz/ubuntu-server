# Stage 0: 基础环境（整合网页3 CUDA支持 + 网页4时区配置）
FROM nvidia/cuda:12.2.2-base-ubuntu22.04 AS base

# 系统级配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai
RUN apt-get update && \
    apt-get install -y locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 1: ROS2 Humble安装（综合网页1和网页3）
FROM base AS ros-installer
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep

# 初始化rosdep（网页1步骤优化）
RUN rosdep init && rosdep update --rosdistro humble

# Stage 2: Isaac ROS构建（整合网页3 GPU支持）
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble

# 安装构建工具链
RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libssl-dev libopencv-dev \
    python3-pip

# 克隆Isaac ROS仓库
WORKDIR /isaac_ws/src
RUN git clone --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common && \
    git clone --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox && \
    git clone --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam

# 安装依赖（网页3 colcon优化）
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 3: 最终镜像（网页2安全加固）
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

# 环境变量配置
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH

# 用户权限（网页2建议）
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws
USER appuser
WORKDIR /home/appuser

# 启动配置
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /isaac_ws/install/setup.bash" >> ~/.bashrc
CMD ["bash"]
