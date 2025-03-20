#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("coscene_recorder"),
        "config",
        "recorder_config.yaml",
    )

    recorder_node = Node(
        package="coscene_recorder",
        executable="coscene_recorder",
        name="recorder",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    return LaunchDescription([recorder_node])
