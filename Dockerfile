# 使用 NVIDIA CUDA 12.8 基础镜像
FROM nvidia/cuda:12.8.0-base-ubuntu22.04 AS base

# Store list of packages (must be first)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-start-packages.csv

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Env setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop
RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# Add ROS 2 apt repository
RUN --mount=type=cache,target=/var/cache/apt \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# ROS fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
        devscripts \
        dh-make \
        fakeroot \
        libxtensor-dev \
        python3-bloom \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        quilt

# Upgrade system setuptools
RUN python3 -m pip install --upgrade --force-reinstall --target=/usr/lib/python3/dist-packages setuptools==65.7.0

# ROS Python fundamentals
RUN python3 -m pip install -U \
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
        boto3 \
        setuptools==65.7.0

# Install ROS 2 Humble
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-angles \
    ros-humble-apriltag \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-cv-bridge \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-diagnostics \
    ros-humble-diagnostic-aggregator \
    ros-humble-diagnostic-updater \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-graceful-controller \
    ros-humble-navigation2 \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbridge-suite \
    ros-humble-rosx-introspection \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-robot-monitor \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-vision-msgs-rviz-plugins

# Setup rosdep
COPY rosdep/extra_rosdeps.yaml /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml
RUN --mount=type=cache,target=/var/cache/apt \
    rosdep init \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && rosdep update

####### -- Install updated packages over installed debians

# Install negotiated from source
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/osrf/negotiated && cd negotiated && git checkout master \
    && source ${ROS_ROOT}/setup.bash \
    && cd negotiated_interfaces && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd negotiated && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install image_proc from 55bf2a38 with backported resize node fix
# https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15
# Revert breaking QoS changes in https://github.com/ros-perception/image_pipeline/pull/814
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-perception/image_pipeline.git && cd image_pipeline && git checkout 55bf2a38c327b829c3da444f963a6c66bfe0598f \
    && git config user.email "builder@nvidia.com" && git config user.name "NVIDIA Builder" \
    && git remote add fork https://github.com/schornakj/image_pipeline.git && git fetch fork && git cherry-pick 969d6c763df99b42844742946f7a70c605a72a15 \
    && source ${ROS_ROOT}/setup.bash \
    && cd image_proc && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb \
    && echo "image_pipeline (image_proc) https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15 on 55bf2a38" >> ${ROS_ROOT}/VERSION \
    && cd ../ && rm -Rf src build log



# Install Moveit 2 ROS packages
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-control-msgs \
    ros-humble-controller-manager \
    ros-humble-geometric-shapes \
    ros-humble-gripper-controllers \
    ros-humble-interactive-markers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-joy \
    ros-humble-launch-param-builder \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-pluginlib \
    ros-humble-py-binding-tools \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz2 \
    ros-humble-srdfdom \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-topic-based-ros2-control \
    ros-humble-ur \
    ros-humble-ur-bringup \
    ros-humble-ur-calibration \
    ros-humble-ur-client-library \
    ros-humble-ur-controllers \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-xacro

# Install various moveit_resources packages from source.
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit_resources.git -b humble \
    && cd moveit_resources && source ${ROS_ROOT}/setup.bash \
    && cd fanuc_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd fanuc_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd panda_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd panda_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd pr2_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd moveit_resources && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb

# Install MoveIt task constructor from source.  The "demo" package depends on moveit_resources_panda_moveit_config,
# installed from source above.
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit_task_constructor.git -b humble \
    && cd moveit_task_constructor && source ${ROS_ROOT}/setup.bash \
    && cd msgs && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd rviz_marker_tools && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd core && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd capabilities && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd visualization && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd demo && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# MoveIt 2's hybrid planning package depends on moveit_resources_panda_moveit_config, installed from source above.
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-moveit-hybrid-planning

# Install moveit2_tutorials from source (depends on moveit_hybrid_planning).
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble \
    && cd moveit2_tutorials && source ${ROS_ROOT}/setup.bash \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install paho-mqtt for isaac_ros_mission_client
RUN python3 -m pip install -U \
        paho-mqtt==1.6.1

# Install cuda-python for isaac_ros_pynitros
RUN python3 -m pip install \
        cuda-python
RUN mkdir -p /tmp/ros-humble-cuda-python-placeholder && \
    wget -O /tmp/ros-humble-cuda-python-placeholder/control \
    https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_dev/main/rosdep/ros-humble-cuda-python-placeholder/control && \
    wget -O /tmp/ros-humble-cuda-python-placeholder/changelog \
    https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_dev/main/rosdep/ros-humble-cuda-python-placeholder/changelog

# 构建并安装占位包
RUN --mount=type=cache,target=/var/cache/apt \
    cd /tmp/ros-humble-cuda-python-placeholder && \
    mkdir -p debian && \
    ln -s control debian/control && \
    ln -s changelog debian/changelog && \
    dpkg-buildpackage -us -uc -b && \
    apt-get install -y ../ros-humble-cuda-python-placeholder_1.0.0_all.deb && \
    rm -rf ../ros-humble-cuda-python-placeholder*

# Patch gtest to make it work with CXX 17
RUN sudo sed -i '917i #ifdef GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '920i #endif' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '2392i #if defined(GTEST_INTERNAL_CPLUSPLUS_LANG) && \\\n    GTEST_INTERNAL_CPLUSPLUS_LANG < 201703L\n#define GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL 1\n#endif' \
    /usr/src/googletest/googletest/include/gtest/internal/gtest-port.h

# Install MCAP CLI
ARG TARGETPLATFORM
ARG MCAP_VERSION=v0.0.51
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-amd64 && \
        chmod +x mcap-linux-amd64 && \
        mv mcap-linux-amd64 /opt/ros/humble/bin/mcap; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-arm64 && \
        chmod +x mcap-linux-arm64 && \
        mv mcap-linux-arm64 /opt/ros/humble/bin/mcap; \
    else \
        echo "Unknown architecture, can't install MCAP CLI" && \
        exit -1; \
    fi

# Install custom vcstool with --delay flag to be robust against
# GitHub rate-limiting (nvbugs/4872446)
RUN mkdir -p /opt/ros/humble && cd /opt/ros/humble \
    && git clone https://github.com/andrewbest-tri/vcstool.git -b andrewbest/delay \
    && echo 'source /opt/ros/humble/vcstool/setup.sh' | tee --append /etc/bash.bashrc

# Make sure that the workspace is always sourced
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" | sudo tee --append /etc/bash.bashrc

# Colcon auto complete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" | sudo tee --append /etc/bash.bashrc

# Store list of packages (must be last)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-end-packages.csv
