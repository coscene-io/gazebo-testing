#!/bin/bash

export ROS_DISTRO=humble
export TURTLEBOT3_MODEL=waffle_pi

mkdir -p /action/packages
tar xzf /cos/bundles/install.tar.gz -C /action/packages
source /action/packages/setup.bash

ros2 launch nav_test test_nav.launch.py
