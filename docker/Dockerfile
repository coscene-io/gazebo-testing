# syntax = docker/dockerfile:1.2
ARG BASE_IMAGE=osrf/ros:humble-desktop-full

FROM ${BASE_IMAGE} as base

# Re-declare ROS_DISTRO after FROM to make it available
ENV ROS_DISTRO=humble

# Add ROS2 repository and install base software
RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
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
 && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
 && echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

WORKDIR /action/ros2_ws
COPY ./docker/ros_entrypoint.sh /
COPY ros2_ws/src/turtlebot3_simulations/ /action/ros2_ws/src/turtlebot3_simulations/

# Create workspace and copy scripts
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --packages-select turtlebot3_gazebo'

ENTRYPOINT [ "/ros_entrypoint.sh" ]

########################################################

FROM base as nav_test

COPY ros2_ws/src/nav_test/ /action/ros2_ws/src/nav_test/

RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --packages-select nav_test'

ENTRYPOINT [ "/ros_entrypoint.sh" ]

########################################################

FROM base as cobridge

RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
 && SOURCE_URL=https://download.coscene.cn/coscene-apt-source \
 && GPG_PATH=/etc/apt/trusted.gpg.d/coscene.gpg \
 && curl -fsSL "$SOURCE_URL/coscene.gpg" | gpg --dearmor -o $GPG_PATH \
 && echo "deb [signed-by=$GPG_PATH] $SOURCE_URL $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/coscene.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-cobridge

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "cobridge", "cobridge_launch.xml"]

########################################################

FROM base as coscene_recorder

COPY ros2_ws/src/coscene_recorder/ /action/ros2_ws/src/coscene_recorder/

RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --packages-select coscene_recorder'

ENTRYPOINT [ "/ros_entrypoint.sh" ]