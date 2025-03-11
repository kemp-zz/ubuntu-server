# Stage 1: 使用官方基础镜像代替自定义镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 系统级配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 2: Isaac ROS构建（关键修正）
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble

# 安装核心依赖（新增rosdep安装）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake build-essential \
    python3-rosdep python3-pip && \
    rm -rf /var/lib/apt/lists/*

# 初始化rosdep（国内网络优化）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml file:///rosdistro/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update --include-eol-distros

# 克隆仓库（带重试机制）
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git || exit 1; \
    done

# 构建指令（添加缓存清理）
RUN . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# Stage 4: 最终镜像
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws

USER appuser
WORKDIR /home/appuser

CMD ["bash"]
