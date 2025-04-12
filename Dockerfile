# syntax=docker/dockerfile:1

# 基础镜像直接指定CUDA 11.8和Ubuntu 22.04
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS2_WS=/opt/ros2_ws
ENV PYTHON_VERSION=3.10
ENV SHELL=/bin/bash

# 配置基础系统
RUN <<EOF
apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git \
    wget \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*
EOF

# 设置ROS 2仓库
RUN <<EOF
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
EOF

# 安装ROS 2软件包
RUN <<EOF
apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*
EOF

# 配置Python环境
RUN <<EOF
add-apt-repository -y ppa:deadsnakes/ppa
apt-get update && apt-get install -y \
    python3.10 \
    python3.10-dev \
    python3.10-venv \
    && rm -rf /var/lib/apt/lists/*
EOF

# 创建Python虚拟环境
RUN <<EOF
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
python3 -m venv /opt/venv
/opt/venv/bin/pip install --upgrade pip setuptools wheel
EOF

# 安装Python依赖
COPY requirements.txt /tmp/
RUN <<EOF
/opt/venv/bin/pip install -r /tmp/requirements.txt
/opt/venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cu118 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118
EOF

# 构建tiny-cuda-nn（直接指定CUDA架构）
RUN <<EOF
git clone https://github.com/NVlabs/tiny-cuda-nn /tmp/tiny-cuda-nn
cd /tmp/tiny-cuda-nn
mkdir build && cd build
cmake -DTCNN_CUDA_ARCHITECTURES=61 ..
make -j$(nproc)
cp -r ../bindings/torch /opt/venv/lib/python3.10/site-packages/tcnn
EOF

# 设置ROS工作空间
RUN mkdir -p /opt/ros2_ws/src
WORKDIR /opt/ros2_ws

# 构建ROS工作空间
RUN <<EOF
source /opt/ros/humble/setup.sh
colcon build --symlink-install
EOF

# 配置入口脚本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 配置Jupyter Lab环境
RUN <<EOF
mkdir -p /root/.jupyter/lab/workspaces
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
echo "source /opt/ros2_ws/install/local_setup.bash" >> /root/.bashrc
echo "source /opt/venv/bin/activate" >> /root/.bashrc
EOF

# 运行时配置
EXPOSE 8888
VOLUME ["/opt/ros2_ws/src", "/app", "/root/.jupyter/lab"]
WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root"]
