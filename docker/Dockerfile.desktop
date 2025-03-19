# syntax = docker/dockerfile:1.2
ARG BASE_IMAGE=kasmweb/ubuntu-jammy-desktop:1.16.1
# ARG BASE_IMAGE=ubuntu:22.04

FROM ${BASE_IMAGE}

# Re-declare ROS_DISTRO after FROM to make it available
ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive

# Switch to root for installation
USER root

# Add ROS2 repository and install base software
RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends software-properties-common curl gnupg lsb-release \
    && add-apt-repository universe \
    && ARCH=$(dpkg --print-architecture) \
    && UBUNTU_CODENAME=$(. /etc/os-release && echo $UBUNTU_CODENAME) \
    && KEYRING_PATH=/usr/share/keyrings/ros-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ${KEYRING_PATH} \
    && echo "deb [arch=${ARCH} signed-by=${KEYRING_PATH}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop-full ros-${ROS_DISTRO}-turtlebot3*

# Install Gazebo using the new keyring method
RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && ARCH=$(dpkg --print-architecture) \
    && UBUNTU_CODENAME=$(lsb_release -cs) \
    && KEYRING_PATH=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o ${KEYRING_PATH} \
    && echo "deb [arch=${ARCH} signed-by=${KEYRING_PATH}] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic

# Set ROS environment variables
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV GAZEBO_MODEL_PATH=/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
WORKDIR /action/ros2_ws

# Copy launch scripts and files
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
COPY ros2_ws/ /action/ros2_ws/

USER 1000
# Create workspace and copy scripts
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && mkdir -p $HOME/Desktop

# Set entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "integration-test", "launch/launch.py"]