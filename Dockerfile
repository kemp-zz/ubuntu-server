# Stage 0: 基础环境（整合网页3 CUDA支持 + 网页4时区配置）
FROM fskemp/ubuntu:latest AS base


ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    locales tzdata ca-certificates \
    && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Stage 1: ROS2 Humble安装（整合网页1和网页3最佳实践）
FROM base AS ros-installer
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init \
    && rosdep update --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

# Stage 2: Isaac ROS构建（整合网页3 GPU支持和网页6分层优化）
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble

# 安装构建工具链
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libssl-dev libopencv-dev \
    python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# 克隆仓库带重试机制（解决网页1的128错误）
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        retries=0; max_retries=3; \
        until [ $retries -ge $max_retries ]; do \
            git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git && break; \
            retries=$((retries+1)); \
            sleep 10; \
        done; \
        [ $retries -ge $max_retries ] && exit 1; \
    done

# 构建优化（网页3分层构建+网页6缓存清理）
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Stage 3: 最终运行时镜像（网页2安全加固+网页10用户权限）
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

# 安全加固配置
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER appuser
WORKDIR /home/appuser

# 环境配置（网页7启动优化）
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /isaac_ws/install/setup.bash" >> ~/.bashrc \
    && echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1' >> ~/.bashrc

CMD ["bash"]
# 启动配置
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /isaac_ws/install/setup.bash" >> ~/.bashrc
CMD ["bash"]
