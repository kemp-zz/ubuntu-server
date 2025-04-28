# 基础镜像
FROM pytorch/pytorch:2.1.2-cuda11.8-cudnn8-devel

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    CONDA_DIR=/opt/conda \
    PATH="$CONDA_DIR/bin:$PATH" \
    CUDA_HOME=/usr/local/cuda \
    TCNN_CUDA_ARCHITECTURES="61;75;80;86" \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# 系统依赖安装（关键修复）
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build libusb-1.0-0-dev \
    libpthread-stubs0-dev libgl1-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

# Miniconda 安装
RUN rm -rf /opt/conda* 2>/dev/null || true && \
    wget --no-check-certificate \
         --retry-connrefused \
         --waitretry=1 \
         --read-timeout=20 \
         --timeout=15 \
         https://repo.anaconda.com/miniconda/Miniconda3-py38_23.3.1-0-Linux-x86_64.sh -O miniconca.sh && \
    bash miniconda.sh -b -p $CONDA_DIR && \
    rm miniconda.sh && \
    conda clean -y --all

# Conda 环境配置
RUN conda create --name nerfstudio python=3.8 -y && \
    conda install -n nerfstudio \
        pytorch==2.1.2 torchvision==0.16.2 pytorch-cuda=11.8 \
        cudatoolkit=11.8 -c pytorch -c nvidia -y

# ROS 工作空间
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone https://github.com/leggedrobotics/radiance_field_ros && \
    git clone https://github.com/orbbec/ros_astra_camera

# 编译 libuvc（修复版）
RUN git clone https://github.com/libuvc/libuvc /tmp/libuvc && \
    cd /tmp/libuvc && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DCMAKE_BUILD_TYPE=Release \
             -DENABLE_PTHREAD=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/libuvc

# 系统编译工具配置
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-9 g++-9 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 900 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 900 && \
    rm -rf /var/lib/apt/lists/*

# Tiny-CUDA-NN 安装
ENV TCNN_CUDA_ARCHITECTURES="61;75;80;86"
RUN pip install ninja && \
    pip install -v --no-cache-dir \
    "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# ROS 依赖安装
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    conda run -n nerfstudio pip install \
    --no-build-isolation \
    -e /catkin_ws/src/radiance_field_ros"

# Catkin 编译
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin build -j$(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release"

# udev 规则复制
RUN cp /catkin_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/

# 最终环境
SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
    conda activate nerfstudio && \
    exec bash"] 
