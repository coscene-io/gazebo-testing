#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source any additional workspace if it exists
if [ -f /action/ros2_ws/install/setup.bash ]; then
    source /action/ros2_ws/install/setup.bash
fi

# Set up ROS environment variables
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# If no command is provided, start bash
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    # Execute the provided command
    exec "$@"
fi 