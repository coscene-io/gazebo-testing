FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

ARG ARCH=$(dpkg --print-architecture)
ARG KASM_VNC_VERSION=1.3.3

# 安装ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-example \
    ros-humble-turtlebot3-teleop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2bag \
    gazebo \
    && rm -rf /var/lib/apt/lists/*

# 安装KasmVNC - 自动检测架构
RUN apt-get update && apt-get install -y \
    xfce4 \
    dbus-x11 \
    wget \
    apt-utils \
    libxtst6 \
    libxrandr2 \
    && mkdir -p /tmp/kasmvnc \
    && wget -q -O /tmp/kasmvnc.deb https://github.com/kasmtech/KasmVNC/releases/download/v${KASM_VNC_VERSION}/kasmvncserver_jammy_${KASM_VNC_VERSION}_${ARCH}.deb \
    && dpkg -i /tmp/kasmvnc.deb || true \
    && apt-get -f install -y \
    && rm -f /tmp/kasmvnc.deb \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace for the launch files
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy launch and recording scripts
COPY launch_nav2.sh /root/launch_nav2.sh
COPY record_topics.sh /root/record_topics.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY start_kasm.sh /root/start_kasm.sh

# Set environment variable for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]