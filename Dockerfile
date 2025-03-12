# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# 新增Python警告抑制（来自colcon问题#454）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=UTC \
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

# 时区与locale配置
RUN apt-get update && \
    apt-get install -y locales && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer


# 配置基础工具链
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        software-properties-common \
        curl gnupg2 lsb-release && \
    rm -rf /var/lib/apt/lists/*

# 显式指定Ubuntu版本（避免动态变量问题）
ENV UBUNTU_CODENAME=jammy

# 配置ROS源（固定版本）
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" > /etc/apt/sources.list.d/ros2.list

# 配置CUDA源（NVIDIA官方推荐方式）
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | gpg --dearmor > /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64 /" > /etc/apt/sources.list.d/cuda.list
# 安装ROS核心组件（合并新包列表）
# 安装ROS核心组件（合并新包列表）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    # 保留系统级构建工具
    build-essential \
    cmake \
    git \
    devscripts \
    && rosdep init \
    && rosdep update --include-eol-distros \
    && rosdep install --from-paths /isaac_ws/src --ignore-src -y \
    && apt-get clean
   
# 升级setuptools并安装Python依赖
RUN python3 -m pip install --upgrade --force-reinstall --target=/usr/lib/python3/dist-packages setuptools==65.7.0 \
    && python3 -m pip install -U --no-cache-dir \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    matplotlib \
    pandas \
    rosbags \
    boto3


FROM base AS isaac-builder

# 继承ROS安装（新增关键依赖）
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/
COPY --from=ros-installer /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /usr/share/keyrings/cuda-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
RUN ls -l /opt/ros/humble && \
    if [ ! -f "/opt/ros/humble/setup.sh" ]; then echo "ROS目录复制失败"; exit 1; fi


# 安装核心构建工具链（强化开发依赖）
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
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
    libeigen3-dev \
    # 新增构建必需组件
    devscripts \
    dh-make \
    fakeroot \
    quilt \
    python3-bloom \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 配置rosdep源（增强完整性）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    apt-get update && \
    apt-get install -y ca-certificates software-properties-common && \
    update-ca-certificates && \
    rosdep update --rosdistro humble --include-eol-distros

# 单次克隆操作（合并重复步骤）
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        rm -rf ${repo} && \
        git clone --depth 1 --branch main --no-tags https://github.com/NVIDIA-ISAAC-ROS/${repo}.git; \
    done

# 构建优化参数扩展
ENV CMAKE_ARGS="-DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
                -DCMAKE_C_COMPILER_LAUNCHER=ccache \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CUDA_ARCHITECTURES=80 \
                -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# 启用分层编译缓存
RUN --mount=type=cache,target=/root/.cache/ccache \  
    . /opt/ros/humble/setup.sh && \
    cd /isaac_ws && \
    colcon build \
        --symlink-install \
        --parallel-workers $(($(nproc) * 3)) \
        --event-handlers console_cohesion+ \
        --cmake-args $CMAKE_ARGS
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
