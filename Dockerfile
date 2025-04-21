# 基础镜像选择（CUDA 11.8 + Ubuntu 20.04 开发环境）
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# --------------------------------
# 阶段一：基础环境配置
# --------------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    CONDA_DIR=/opt/conda \
    PATH="/opt/conda/bin:$PATH" \
    CUDA_HOME=/usr/local/cuda \
    TCNN_CUDA_ARCHITECTURES="61;75;80;86" \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# 安装系统工具与语言环境
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl software-properties-common nano git wget \ 
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS Noetic 核心组件
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-rosdep python3-catkin-pkg python3-rospkg \
        python3-rosinstall-generator python3-vcstool \
        ros-noetic-rviz ros-noetic-jsk-rviz-plugins \
        libgflags-dev libusb-1.0-0-dev libeigen3-dev libdw-dev \       
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep（修复权限问题）[9](@ref)
RUN rosdep init \
    && chmod -R a+rwx /etc/ros/rosdep/sources.list.d \
    && rosdep update --rosdistro=$ROS_DISTRO

# --------------------------------
# 阶段二：CUDA 开发工具链配置
# --------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvcc-11-8 cuda-cudart-dev-11-8 libcublas-dev-11-8 \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------
# 阶段三：Miniconda 环境配置
# --------------------------------
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-py38_23.3.1-0-Linux-x86_64.sh -O miniconda.sh \
    && bash miniconda.sh -b -p $CONDA_DIR \
    && rm miniconda.sh \
    && conda clean -y --all

# 创建 Conda 环境并安装 PyTorch
RUN conda create --name nerfstudio python=3.8 -y \
    && conda install -n nerfstudio \
        pytorch==2.1.2 torchvision==0.16.2 pytorch-cuda=11.8 \
        -c pytorch -c nvidia -y

# --------------------------------
# 阶段四：ROS 工作空间构建
# --------------------------------
# 克隆 ROS 项目仓库
RUN mkdir -p /catkin_ws/src \
    && cd /catkin_ws/src \
    && git clone https://github.com/leggedrobotics/radiance_field_ros \
    && git clone https://github.com/orbbec/ros_astra_camera.git

# 安装 libuvc（优化构建步骤）
RUN git clone https://github.com/libuvc/libuvc /tmp/libuvc \
    && cd /tmp/libuvc && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install \
    && ldconfig && rm -rf /tmp/libuvc


RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-9 g++-9 build-essential cmake libopenblas-dev \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 900 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 900 \
    && rm -rf /var/lib/apt/lists/*
    
ENV TCNN_CUDA_ARCHITECTURES="61;75;80;86"
RUN pip install ninja \
    && CUDA_HOME=/usr/local/cuda \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH \
    TORCH_CUDA_ARCH_LIST="$TCNN_CUDA_ARCHITECTURES" \
    pip install -v --no-cache-dir "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# 安装主项目（修复 catkin_pkg 依赖）
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && conda run -n nerfstudio pip install \
        --no-build-isolation \
        -e /catkin_ws/src/radiance_field_ros"

# --------------------------------
# 阶段六：Catkin 工作空间构建
# --------------------------------
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin build -j$(nproc)"

# 复制 udev 规则
RUN cp /catkin_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/

# 最终启动命令
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && conda activate nerfstudio && /bin/bash"]
