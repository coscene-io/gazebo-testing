# syntax = docker/dockerfile:1.2
ARG BASE_IMAGE=osrf/ros:humble-desktop-full

FROM ${BASE_IMAGE} as base

LABEL org.opencontainers.image.source https://github.com/coscene-io/gazebo-testing

# Re-declare ROS_DISTRO after FROM to make it available
ENV ROS_DISTRO=humble

# Add ROS2 repository and install base software
RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean\
 && apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl wget gnupg lsb-release \
 && SOURCE_URL=https://download.coscene.cn/coscene-apt-source \
 && GPG_PATH=/etc/apt/trusted.gpg.d/coscene.gpg \
 && curl -fsSL "$SOURCE_URL/coscene.gpg" | gpg --dearmor -o $GPG_PATH \
 && echo "deb [signed-by=$GPG_PATH] $SOURCE_URL $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/coscene.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cobridge \
 && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
 && echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2* \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

COPY ./scripts /scripts
ENTRYPOINT [ "/scripts/entrypoint.sh" ]

WORKDIR /action/ros2_ws