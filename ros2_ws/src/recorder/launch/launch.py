#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the current script directory
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Set parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    output_path = LaunchConfiguration("output_path", default="/outputs")

    # Include the navigation2.launch.py from turtlebot3_navigation2 package
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("turtlebot3_navigation2"),
                        "launch",
                        "navigation2.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Include the record launch file with event handler
    record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([current_dir, "/record.launch.py"]),
        launch_arguments={"output_path": output_path}.items(),
    )

    # Add Gazebo simulator launch (if needed)
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        PathJoinSubstitution("turtlebot3_gazebo"),
                        "launch",
                        "turtlebot3_dqn_stage2.launch.py",
                    ]
                ),
            ]
        )
    )

    return LaunchDescription(
        [
            nav2_launch,
            record_launch,
            # Uncomment to add Gazebo launch
            gazebo_world,
        ]
    )
