# syntax = docker/dockerfile:1.2
ARG BASE_IMAGE=osrf/ros:humble-desktop-full

FROM ${BASE_IMAGE}

# Re-declare ROS_DISTRO after FROM to make it available
ENV ROS_DISTRO=humble

# Add ROS2 repository and install base software
RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
 && SOURCE_URL=https://download.coscene.cn/coscene-apt-source \
 && GPG_PATH=/etc/apt/trusted.gpg.d/coscene.gpg \
 && curl -fsSL "$SOURCE_URL/coscene.gpg" | gpg --dearmor -o $GPG_PATH \
 && echo "deb [signed-by=$GPG_PATH] $SOURCE_URL $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/coscene.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl gnupg lsb-release \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2* \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-cobridge \
    python3-ujson \
 && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
 && echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

COPY ./docker/ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]

WORKDIR /action/ros2_ws
COPY ros2_ws/src/ /action/ros2_ws/src/

RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install'

CMD ["ros2", "launch", "nav_test", "test_nav.launch.py"]