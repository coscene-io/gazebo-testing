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
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
 && bash -c 'echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
 && echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc'

FROM base as workspace

# Set ROS environment variables
ENV TURTLEBOT3_MODEL=waffle_pi
ENV GAZEBO_MODEL_PATH=/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ENV ROS_DOMAIN_ID=0
WORKDIR /action/ros2_ws

# Copy launch scripts and files
COPY ./docker/ros_entrypoint.sh /
COPY ros2_ws/ /action/ros2_ws/

# Create workspace and copy scripts
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install'

# Set entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]