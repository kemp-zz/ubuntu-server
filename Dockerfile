# 基础镜像（CUDA 11.8 + Ubuntu 20.04）
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# --------------------------------
# 环境变量配置
# --------------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    CONDA_DIR=/opt/conda \
    PATH="$CONDA_DIR/bin:$PATH" \
    CUDA_HOME=/usr/local/cuda \
    TCNN_CUDA_ARCHITECTURES="61" \
    LD_LIBRARY_PATH="$CUDA_HOME/lib64:$LD_LIBRARY_PATH"

# --------------------------------
# 阶段一：基础环境配置
# --------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl software-properties-common git wget \
    build-essential cmake ninja-build python3.9 python3.9-dev python3.9-venv \
    libboost-all-dev libeigen3-dev libglew-dev libfreeimage-dev \
    && rm -rf /var/lib/apt/lists/*

# 设置 Python 3.9 为默认解释器
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 100

# 安装 ROS Noetic 核心组件
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor | tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-rosdep python3-catkin-pkg python3-rospkg \
        ros-noetic-rviz ros-noetic-jsk-rviz-plugins \
        libgflags-dev libusb-1.0-0-dev libdw-dev \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && rosdep update --rosdistro=$ROS_DISTRO

# --------------------------------
# 阶段二：Conda 环境配置（关键修复）
# --------------------------------
# 安装 Miniconda 并配置环境变量
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-py39_24.1.2-0-Linux-x86_64.sh -O miniconda.sh \
    && bash miniconda.sh -b -p $CONDA_DIR \
    && rm miniconda.sh \
    && echo "export PATH=\"$CONDA_DIR/bin:\$PATH\"" >> ~/.bashrc \
    && echo "source $CONDA_DIR/etc/profile.d/conda.sh" >> ~/.bashrc \
    && conda clean -y --all

# 创建并激活 Python 3.9 Conda 环境
RUN conda create --name ros2_env python=3.9 -y \
    && conda install -n ros2_env \
        pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y

# --------------------------------
# 阶段三：CUDA 工具链与依赖
# --------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvcc-11-8 cuda-cudart-dev-11-8 libcublas-dev-11-8 \
    && rm -rf /var/lib/apt/lists/*

# 安装 Tiny CUDA NN（优化 CUDA 架构参数）
RUN pip install ninja \
    && pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch \
    && TCNN_CUDA_ARCHITECTURES=$TCNN_CUDA_ARCHITECTURES pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

# --------------------------------
# 阶段四：ROS 工作空间构建
# --------------------------------
RUN mkdir -p /catkin_ws/src \
    && cd /catkin_ws/src \
    && git clone https://github.com/leggedrobotics/radiance_field_ros \
    && git clone https://github.com/orbbec/ros_astra_camera.git

# 优化 libuvc 编译（多线程+路径优化）
RUN git clone https://github.com/libuvc/libuvc /tmp/libuvc \
    && cd /tmp/libuvc/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) && make install \
    && ldconfig

# 安装系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-10 g++-10 libopenblas-dev \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100 \
    && rm -rf /var/lib/apt/lists/*

# 安装主项目依赖
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    conda run -n ros2_env pip install --no-build-isolation -e /catkin_ws/src/radiance_field_ros"

# --------------------------------
# 阶段五：编译与部署
# --------------------------------
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin build -j$(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 配置 udev 规则
RUN cp /catkin_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/

# 设置启动环境
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && conda activate ros2_env && /bin/bash"]
