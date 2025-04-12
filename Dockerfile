# syntax=docker/dockerfile:1

ARG CUDA_VERSION=11.8.0
ARG UBUNTU_VERSION=22.04
ARG ROS_DISTRO=humble
ARG TCNN_CUDA_ARCHITECTURES=61

# Base image with CUDA 11.8 development environment
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES}
ENV ROS2_WS=/opt/ros2_ws
ENV PYTHON_VERSION=3.10
ENV SHELL=/bin/bash

# Configure base system
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

# Setup ROS 2 repository
RUN <<EOF
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
EOF

# Install ROS 2 packages
RUN <<EOF
apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*
EOF

# Setup Python environment
RUN <<EOF
add-apt-repository -y ppa:deadsnakes/ppa
apt-get update && apt-get install -y \
    python${PYTHON_VERSION} \
    python${PYTHON_VERSION}-dev \
    python${PYTHON_VERSION}-venv \
    && rm -rf /var/lib/apt/lists/*
EOF

# Create Python virtual environment
RUN <<EOF
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1
python3 -m venv /opt/venv
/opt/venv/bin/pip install --upgrade pip setuptools wheel
EOF

# Install Python dependencies
COPY requirements.txt /tmp/
RUN <<EOF
/opt/venv/bin/pip install -r /tmp/requirements.txt
/opt/venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cu118 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118
EOF

# Build tiny-cuda-nn with specified architectures
RUN <<EOF
git clone https://github.com/NVlabs/tiny-cuda-nn /tmp/tiny-cuda-nn
cd /tmp/tiny-cuda-nn
mkdir build && cd build
cmake -DTCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES} ..
make -j$(nproc)
cp -r ../bindings/torch /opt/venv/lib/python3.10/site-packages/tcnn
EOF

# Setup ROS workspace
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}

# Build ROS workspace
RUN <<EOF
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install
EOF

# Configure entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Configure Jupyter Lab
RUN <<EOF
mkdir -p /root/.jupyter/lab/workspaces
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
echo "source ${ROS2_WS}/install/local_setup.bash" >> /root/.bashrc
echo "source /opt/venv/bin/activate" >> /root/.bashrc
EOF

# Runtime configuration
EXPOSE 8888
VOLUME ["/opt/ros2_ws/src", "/app", "/root/.jupyter/lab"]
WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root"]
