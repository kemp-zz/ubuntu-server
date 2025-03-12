# Stage 1: 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y locales tzdata ca-certificates && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8

# Stage 2: ROS2 Humble 安装
FROM base AS ros-installer
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y ros-humble-desktop ros-humble-ros-base python3-rosdep && \
    rosdep init && \
    rosdep update --include-eol-distros

# Stage 3: Isaac ROS 构建
FROM base AS isaac-builder
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=ros-installer /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/
COPY --from=ros-installer /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/

# 新增：配置自定义rosdep源
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    # 下载NVIDIA官方额外依赖定义
    curl -sSL https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/docker/rosdep/extra_rosdeps.yaml \
        -o /etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml && \
    # 更新rosdep配置
    echo "yaml file:///etc/ros/rosdep/sources.list.d/99-isaac-rosdeps.yaml" \
        >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    # 添加标准ROS源
    echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" \
        >> /etc/ros/rosdep/sources.list.d/20-default.list && \


# 安装核心依赖（新增python3-pip）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake build-essential python3-pip python3-rosdep python3-venv \
    libopencv-dev libeigen3-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 更新rosdep（新增--include-eol-distros参数）
RUN rosdep update --include-eol-distros --rosdistro=humble
# 克隆仓库
WORKDIR /isaac_ws/src
RUN for repo in isaac_ros_common isaac_ros_nvblox isaac_ros_visual_slam; do \
        retries=0; max_retries=5; \
        until [ $retries -ge $max_retries ]; do \
            git clone --depth 1 --branch main https://github.com/NVIDIA-ISAAC-ROS/${repo}.git && break; \
            retries=$((retries+1)); \
            echo "Retrying $repo ($retries/$max_retries)..."; \
            sleep 10; \
        done; \
        [ $retries -lt $max_retries ] || { echo "Clone failed for $repo"; exit 1; }; \
    done

# 构建
RUN . /opt/ros/humble/setup.sh && \
    # 安装额外Python依赖
    pip3 install --no-cache-dir \
    "setuptools>=58.0" \
    "vcd>=0.1" \
    "nvidia-pyindex>=1.0.9" && \
    # 预安装Isaac ROS核心依赖
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y \
    --skip-keys "libopencv-dev libeigen3-dev"

# Stage 4: 最终镜像
FROM base
COPY --from=ros-installer /opt/ros/humble /opt/ros/humble
COPY --from=isaac-builder /isaac_ws/install /isaac_ws/install

RUN useradd -m appuser && \
    chown -R appuser:appuser /isaac_ws /opt/ros/humble

ENV HOME=/home/appuser
USER appuser
WORKDIR $HOME

CMD ["bash"]
