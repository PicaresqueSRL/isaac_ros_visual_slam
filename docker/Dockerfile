# Build image for pq_cuvslam (ROS 2 Lyrical, Ubuntu 24.04, CUDA).
#
# One Dockerfile for desktop PC and Jetson: only the base image changes.
# Build from the repository root (the cuvslam submodule must be initialized):
#
#   git submodule update --init
#   docker build -f docker/Dockerfile -t pq_cuvslam:dev .
#
# See docker/README.md for the build arguments and how to run the container.

# Base image with the CUDA toolkit (devel) on Ubuntu 24.04.
#   PC (x86_64):  nvidia/cuda:13.1.1-devel-ubuntu24.04 (default)
#   Jetson:       JetPack 7.2 based image (to be defined and validated on the Orin Nano)
ARG BASE_IMAGE=nvidia/cuda:13.1.1-devel-ubuntu24.04

FROM ${BASE_IMAGE}

# Build options forwarded to the pq_cuvslam CMake configuration.
#   CUVSLAM_REF:     cuVSLAM git tag or commit to build (e.g. v17.0.1); empty: the submodule
#   USE_CUNLS:       cuNLS solver in cuVSLAM (AUTO: on from cuVSLAM 17, off before)
#   USE_CUDA_BUFFER: zero-copy images through Isaac ROS cuda_buffer (AUTO/ON/OFF)
#   BUILD_TYPE:      CMake build type
ARG CUVSLAM_REF=
ARG USE_CUNLS=AUTO
ARG USE_CUDA_BUFFER=AUTO
ARG BUILD_TYPE=Release
# Version of the ros2-apt-source package that configures the ROS 2 apt repository.
ARG ROS_APT_SOURCE_VERSION=1.2.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=lyrical

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# System tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg \
        python3 \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Lyrical for Ubuntu 24.04 comes from the Isaac ROS buildfarm (Lyrical is
# Tier 3 on Ubuntu 24.04 upstream). The standard ROS 2 repository provides colcon
# and rosdep.
RUN curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
        | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg]" \
            "https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list \
    && . /etc/os-release \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-lyrical-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Workspace: dependencies first (cached layer), then the sources.
WORKDIR /ws
COPY pq_cuvslam/package.xml src/isaac_ros_visual_slam/pq_cuvslam/package.xml
COPY isaac_ros_visual_slam_interfaces/package.xml \
     src/isaac_ros_visual_slam/isaac_ros_visual_slam_interfaces/package.xml
# The upstream rosdep database has no Lyrical entries for Ubuntu 24.04 (noble),
# and the Isaac ROS buildfarm does not provide a rosdep source. Keys are resolved
# as for Ubuntu 26.04 (resolute), the reference platform of Lyrical: the apt
# package names are the same, and the packages are installed from the noble
# repositories configured above.
ARG ROSDEP_OS=ubuntu:resolute
RUN apt-get update \
    && rosdep init \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install -y --rosdistro "${ROS_DISTRO}" --os "${ROSDEP_OS}" \
        --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

COPY . src/isaac_ros_visual_slam

# The cuVSLAM build needs the submodule sources.
RUN test -f src/isaac_ros_visual_slam/cuvslam/CMakeLists.txt \
    || { echo "cuvslam submodule missing: run 'git submodule update --init'" >&2; exit 1; }

# The cuvslam submodule is a plain CMake project: colcon would build it as a
# separate package. It is built by pq_cuvslam (ExternalProject), so ignore it.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
        --packages-ignore cuVSLAM \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
            -DCUVSLAM_REF=${CUVSLAM_REF} \
            -DUSE_CUNLS=${USE_CUNLS} \
            -DUSE_CUDA_BUFFER=${USE_CUDA_BUFFER}

RUN printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        "source /ws/install/setup.bash" \
        >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
