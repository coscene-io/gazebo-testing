#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
mkdir -p /cos/bundles
ls -la /cos/bundles
tar czf /cos/bundles/install.tar.gz -C ./install .
