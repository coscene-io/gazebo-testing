#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node  # type: ignore
from ament_index_python.packages import get_package_share_directory  # type: ignore
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coscene_recorder")
    config_file = os.path.join(pkg_dir, "config", "recorder_config.yaml")

    recorder_node = Node(
        package="coscene_recorder",
        executable="coscene_recorder",
        name="recorder",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    return LaunchDescription([recorder_node])
