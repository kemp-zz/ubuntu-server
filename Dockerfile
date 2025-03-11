# Stage 1: Base image with CUDA
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC

# Configure system essentials
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    locales \
    ca-certificates \
    tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS 2 Humble Installation
FROM base AS ros-installer

# Install ROS 2 repository setup
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Add ROS 2 GPG key with retries
RUN for i in {1..5}; do \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && break || sleep 15; \
    done

# Set up ROS 2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 packages with retries
RUN apt-get update && \
    for i in {1..3}; do \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-rosdep \
    python3-pytest \
    python3-ament-python \
    && break || sleep 30; \
    done && \
    rm -rf /var/lib/apt/lists/*

# Configure rosdep
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" \
    > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update --include-eol-distros

# Stage 3: Isaac ROS Builder
FROM base AS isaac-builder

# Copy ROS installation
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

# Install build dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    libopencv-dev \
    libeigen3-dev \
    python3-venv \
    python3-colcon-common-extensions \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Configure workspace
WORKDIR /isaac_ws/src

# Clone core repositories with version pinning
RUN git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
    git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Clone dependencies
RUN mkdir -p /isaac_ws/src/isaac_ros_deps && \
    cd /isaac_ws/src/isaac_ros_deps && \
    git clone --depth 1 --branch humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

# Stage 4: Final Image
FROM base

# Copy installed components
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

# Configure non-root user
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser \
    ROS_DISTRO=humble

USER appuser
WORKDIR $HOME

CMD ["bash"]
