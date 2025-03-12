# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC

# 时区与locale配置（使用官方UTC时区）
RUN apt-get update && \
    apt-get install -y locales && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer

# 配置官方ROS源
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 安装ROS核心组件
RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-rosdep && \
    rosdep init && \
    rosdep update --include-eol-distros && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 3: Isaac ROS 构建环境
FROM base AS isaac-builder

# 继承ROS安装
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

# 安装构建工具链
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    python3-rosdep \
    python3-venv \
    python3-pytest \
    python3-pytest-mock \
    libopencv-dev \
    libeigen3-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* \
           /var/cache/apt/archives/* \
           /tmp/*

# 配置rosdep官方源
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    # 使用代理加速GitHub资源下载（2025年最新可用代理）
    curl -sSL "https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/docker/rosdep/extra_rosdeps.yaml" \
        -o /etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml && \
    # 手动补充缺失的pytest依赖规则（2025年NVIDIA官方推荐方案）
    echo -e "\npython3-pytest:\n  ubuntu: [python3-pytest]\npython3-pytest-mock:\n  ubuntu: [python3-pytest-mock]" >> /etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml && \
    # 调整源加载顺序（关键修改）
    printf "yaml file:///etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml\nyaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml\nyaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    # 强制更新依赖数据库
    rosdep update --include-eol-distros --rosdistro=humble

# 克隆Isaac ROS仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 构建Isaac组件
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 4: 最终运行时镜像
FROM base

# 继承运行时组件
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

# 创建非root用户
RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser
USER appuser
WORKDIR $HOME

CMD ["bash"]
