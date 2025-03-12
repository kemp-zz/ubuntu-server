# 阶段1：基础环境构建
FROM nvcr.io/nvidia/cuda:12.8.0-devel-ubuntu22.04 AS builder

# 必须分两阶段设置（先全局后局部）
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Hong_Kong

RUN apt-get update && \
    # 预配置时区选择答案
    ln -fs /usr/share/zoneinfo/Asia/Hong_Kong /etc/localtime && \
    echo "tzdata tzdata/Areas select Asia" | debconf-set-selections && \
    echo "tzdata tzdata/Zones/Asia select Hong_Kong" | debconf-set-selections && \
    # 安装并配置tzdata
    apt-get install -y --no-install-recommends tzdata && \
    dpkg-reconfigure -f noninteractive tzdata && \
    # 清理缓存
    rm -rf /var/lib/apt/lists/*
# 配置APT源（包含ROS和NVIDIA源）
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# 安装核心依赖（参考Isaac ROS构建实践）
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-desktop \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# 配置非特权用户（安全最佳实践）
RUN useradd -m -u 1000 -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 阶段2：运行时镜像
FROM nvcr.io/nvidia/cuda:12.2.2-runtime-ubuntu22.04

# 继承构建阶段配置
COPY --from=builder /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/
COPY --from=builder /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=builder /etc/sudoers /etc/sudoers
COPY --from=builder /home/developer /home/developer

# 安装运行时依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# 配置环境变量
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV ROS_DOMAIN_ID=0
ENV USER=developer

# 设置工作目录及权限
WORKDIR /workspace
RUN chown -R developer:developer /workspace

# 切换非root用户
USER developer

# 初始化ROS环境
RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc
CMD ["/bin/bash"]
