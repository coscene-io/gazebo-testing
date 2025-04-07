#!/bin/bash

mkdir -p /action/packages
tar xzf /cos/bundles/install.tar.gz -C /action/packages
source /action/packages/setup.bash

ros2 launch nav_test test_nav.launch.py
