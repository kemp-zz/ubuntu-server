FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata ca-certificates && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y ros-humble-desktop ros-humble-ros-base python3-rosdep python3-vcstool  && \ # Added vcstool
    rosdep init && \
    rosdep update --include-eol-distros


# Stage 3: Isaac ROS 构建
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

# 安装核心依赖
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake build-essential python3-rosdep python3-venv \
    libopencv-dev libeigen3-dev python3-colcon-common-extensions wget \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# Use the official ROS sources and add Isaac ROS sources
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    wget https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros-humble/main/rosdep/isaac-ros.repos -O /etc/ros/rosdep/sources.list.d/30-isaac-ros.list && \
    rosdep update  # Update rosdep after adding Isaac ROS sources


# Clone repositories using vcstool
WORKDIR /isaac_ws/src
COPY isaac_ros.repos .  # Assumes you have an isaac_ros.repos file in your build context
RUN vcs import < isaac_ros.repos # Use vcstool for consistent versioning

# Install specific dependencies for the packages you're building (adjust as needed)
RUN apt-get update && apt-get install -y python3-pytest ament-python  libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
     && rm -rf /var/lib/apt/lists/*

# Build - Key improvements here for reliability and clarity
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -yrv && \  # Added -r for retrying failed dependencies
    colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+


# Stage 4: 最终镜像
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser
USER appuser
WORKDIR $HOME

# Set environment variables for ROS (important!)
RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
ENV ROS_DISTRO=humble

CMD ["bash"] # Or your desired command
