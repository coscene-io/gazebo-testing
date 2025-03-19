from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package path
    pkg_dir = get_package_share_directory("nav_test")

    # Build config path
    config_path = os.path.join(pkg_dir, "config", "test_points.yaml")

    return LaunchDescription(
        [
            Node(
                package="nav_test",
                executable="nav_controller",
                name="nav_controller",
                output="screen",
                parameters=[
                    {"config_path": config_path},
                    {"report_path": "/outputs/nav_reports"},
                ],
            )
        ]
    )
