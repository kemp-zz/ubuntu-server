# 使用 NVIDIA CUDA 11.8 开发环境作为基础镜像
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

LABEL org.opencontainers.image.source="https://github.com/SimonSchwaiger/ros-ml-container"

# 构建参数
ARG GRAPHICS_PLATFORM=opensource
ARG PYTHONVER=3.10
ARG ROS_DISTRO=humble
ARG IGNITION_VERSION=fortress
ARG ROS2_WS=/opt/ros2_ws

# 环境配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_WS=${ROS2_WS} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# 安装基础图形库（NVIDIA 镜像已包含驱动）
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# 时区配置
RUN ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -y tzdata && rm -rf /var/lib/apt/lists/*

# 安装 ROS 构建依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    python3-flake8 \
    python3-pip \
    python3-pytest \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# 配置 ROS 软件源
RUN wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# 安装 ROS 开发工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    && rosdep init && rosdep update

# 安装指定 ROS 发行版
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# 配置 Python 环境
RUN apt-get update && apt-get install -y --no-install-recommends \
    python${PYTHONVER} \
    python${PYTHONVER}-dev \
    python${PYTHONVER}-venv \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHONVER} 1

# 创建 Python 虚拟环境
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip setuptools

# 安装 Python 依赖
ADD requirements.txt .
RUN /opt/venv/bin/pip install -r requirements.txt && \
    /opt/venv/bin/pip install launchpadlib rosinstall_generator empy catkin_tools

# 创建工作空间并初始化
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}

# 初始化工作区（可选：添加示例包）
RUN touch ${ROS2_WS}/src/COLCON_IGNORE  # 防止空目录导致构建错误

# 清理构建文件（根据实际需求调整）
# RUN rm -rf ${ROS2_WS}/src  # 如果不需要保留空目录

# 配置 Shell 环境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS2_WS}/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ~/.bashrc

# 设置入口点
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
