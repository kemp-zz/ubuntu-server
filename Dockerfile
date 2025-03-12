# ====================== Stage 1: 基础环境 ======================
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=humble \
    ISAAC_WS=/isaac_ws \
    PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH}"

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        locales tzdata ca-certificates && \
    ln -snf "/usr/share/zoneinfo/$TZ" /etc/localtime && \
    echo "$TZ" > /etc/timezone && \
    locale-gen en_US.UTF-8 && \
    update-ca-certificates

# ====================== Stage 2: ROS2 核心 ======================
FROM base AS ros-core

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common curl gnupg2 && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3-colcon-common-extensions \
        ros-humble-rcl-logging-spdlog \
        ros-humble-ament-cmake-auto \
        ros-humble-rclcpp \
        libspdlog-dev \
        python3-rosdep && \
    ln -sf /usr/bin/python3 /usr/bin/python && \
    pip3 install --upgrade setuptools wheel && \
    rosdep init || true && \
    rosdep update --include-eol-distros

# ====================== Stage 3: 构建环境 ======================
FROM base AS builder

COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=ros-core /usr/lib/python3/dist-packages /usr/lib/python3/dist-packages
COPY --from=ros-core /usr/bin/colcon /usr/bin/colcon

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential libssl-dev ros-humble-ament-cmake \
        cmake git libopencv-dev ccache && \
    ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++

WORKDIR $ISAAC_WS/src

RUN git clone --branch main --recurse-submodules --depth 1 \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
    && { [ -f "isaac_ros_common/isaac_ros_common/package.xml" ] && \
         [ -d "isaac_ros_common/isaac_common" ]; } || exit 1 \
    && git clone --branch main --recurse-submodules --depth 1 \
        https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                -DBUILD_TESTING=OFF \
                -DCMAKE_CUDA_ARCHITECTURES='80-real;86-real;89-virtual' \
                -DCMAKE_PREFIX_PATH=/opt/ros/humble \
                -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog"

RUN --mount=type=cache,target=/root/.cache/ccache \
    . /opt/ros/humble/setup.sh && \
    cd $ISAAC_WS && \
    colcon build --symlink-install --parallel-workers $(nproc) --cmake-args $CMAKE_ARGS

# ====================== Stage 4: 运行时镜像 ======================
FROM base

COPY --from=ros-core /opt/ros/humble /opt/ros/humble
COPY --from=builder $ISAAC_WS/install $ISAAC_WS/install

RUN useradd -m -d /home/appuser -s /bin/bash -u 1001 appuser && \
    chown -R appuser:appuser $ISAAC_WS /opt/ros/humble && \
    chmod 750 $ISAAC_WS /opt/ros/humble

ENV PATH="/home/appuser/.local/bin:/opt/ros/humble/bin:$PATH" \
    LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH" \
    PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

USER appuser
WORKDIR /home/appuser
EXPOSE 5000/tcp 5005/udp
HEALTHCHECK --interval=30s --timeout=5s --retries=3 CMD ros2 node list || exit 1
CMD ["bash"]
