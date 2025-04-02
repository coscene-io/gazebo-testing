#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coscene_recorder")

    config_file = os.path.join(pkg_dir, "config", "recorder_config.yaml")

    return LaunchDescription([
        Node(
            package="coscene_recorder",
            executable="coscene_recorder",
            name="recorder",
            output="screen",
            parameters=[
                {"config_file": config_file}
            ],
        )
    ])
