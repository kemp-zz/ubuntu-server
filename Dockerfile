# 基础镜像（CUDA 11.8 + Ubuntu 22.04）
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# 元数据
LABEL maintainer="Your Name <your.email@example.com>"
LABEL org.opencontainers.image.description="ROS 2 Humble + CUDA 11.8 + JupyterLab Development Container"

# 构建参数（支持多平台配置）
ARG GRAPHICS_PLATFORM=opensource
ARG PYTHONVER=3.10
ARG ROS_DISTRO=humble
ARG ROS2_WS=/opt/ros2_ws

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    SHELL=/bin/bash \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS2_WS=${ROS2_WS} \
    WORKSPACE=/nerf_ws \
    MODEL_CONFIG_PATH=${WORKSPACE}/nerf_config/model_config.yaml \
    TRAINER_CONFIG_PATH=${WORKSPACE}/nerf_config/trainer_config.yaml \
    OCCUPANCY_CONFIG_PATH=${WORKSPACE}/nerf_config/occupancy_config.yaml \
    PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
    TORCH_EXTENSIONS_DIR=${WORKSPACE}/torch_extensions \
    TORCH_CUDA_ARCH_LIST="6.1" \
    ENV TCNN_CUDA_ARCHITECTURES="61" \
    PYTHONPATH=${WORKSPACE}:${PYTHONPATH:-}

# 第一阶段：基础系统配置
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt repo and key
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Mesa for GUI
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install more dependencies including flake8
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    dirmngr \
    gnupg2 \
    software-properties-common \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Python setup and upgrade
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHONVER} 1 \
    && python3 -m pip install --upgrade pip wheel setuptools

# Basic ROS setup
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# More pip packages
RUN pip3 install -U \
    argcomplete pytest

# ROS Dep initialization
RUN rosdep init \
    && rosdep update

# Setting up colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# More ROS-related and general packages
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-base ros-dev-tools ros-${ROS_DISTRO}-rqt* ros-${ROS_DISTRO}-rosbridge-server

# Python Version Setup using deadsnakes (more control over versions)
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y python${PYTHONVER} python${PYTHONVER}-dev python${PYTHONVER}-tk

# More tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    libopenmpi-dev \
    zlib1g-dev \
    imagemagick \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install virtualenv

# Setup virtualenv using deadsnakes's Python
RUN virtualenv -p /usr/bin/python${PYTHONVER} ~/myenv

# Activate the virtualenv and upgrade pip
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip"

# Install ROS python dependencies within the virtualenv
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools \
    lark \
    lxml \
    pytest \
    numpy \
    netifaces pymongo Pillow \
    && pip3 install --upgrade setuptools"

# NeRF related stuff + JupyterLab
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install jupyterlab nerfstudio pypose --extra-index-url https://download.pytorch.org/whl/cu118 --trusted-host download.pytorch.org \
    torch==2.1.2+cu118 torchvision==0.16.2+cu118"

# Tiny CUDA NN
RUN git clone https://github.com/NVlabs/tiny-cuda-nn.git /tmp/tcnn \
    && cd /tmp/tcnn/bindings/torch \
    && /bin/bash -c "source ~/myenv/bin/activate \
    && python setup.py install" \
    && rm -rf /tmp/tcnn

# Workspace creation
RUN mkdir -p ${ROS2_WS}/src

# Clone astra camera driver
WORKDIR ${ROS2_WS}/src
RUN git clone https://github.com/orbbec/ros2_astra_camera.git

# Setup and compile the ROS workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update \
    && rosdep install --from-paths $ROS2_WS/src -i -y --rosdistro $ROS_DISTRO"

# colcon build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd $ROS2_WS \
    && colcon build --symlink-install"

# Cleanup
RUN rm -rf /var/lib/apt/lists/*

# Add sourcing to bashrc (make sure to source venv, then ROS)
RUN echo "source ~/myenv/bin/activate" >> ~/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $ROS2_WS/install/local_setup.bash" >> ~/.bashrc

# Set SHELL env variable (fixes autocompletion in web-based shell)
ENV SHELL=/bin/bash

# Add entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

# Command to run
CMD ["jupyter-lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root"]
