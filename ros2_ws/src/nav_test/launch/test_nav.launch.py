from collections.abc import Iterable
import os
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node


def include_pkg_py_launch(package, launch_file, launch_arguments=None, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                "launch",
                launch_file,
            )
        ),
        launch_arguments=launch_arguments,
        **kwargs,
    )


def include_pkg_xml_launch(package, launch_file, launch_arguments=None, **kwargs):
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                "launch",
                launch_file,
            )
        ),
        launch_arguments=launch_arguments,
        **kwargs,
    )


def path_join(*path_list: str) -> str:
    return PathJoinSubstitution(path_list)


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_test")
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    data_root = LaunchConfiguration("data_root")

    # data_root:=$DATA_ROOT
    default_data_root = "/input"

    # turtlebot3_model:=waffle_pi
    default_turtlebot3_model = "waffle_pi"

    # test_points:=$DATA_ROOT/config/test_points.yaml
    default_test_points_path = path_join(data_root, "config", "test_points.yaml")

    # report_path:=/output/reports
    default_report_path = "/output/reports"

    # world:=$DATA_ROOT/worlds/RMUL2024_world_dynamic_obstacles.world
    default_world = path_join(
        data_root, "worlds", "RMUL2024_world_dynamic_obstacles.world"
    )

    # map:=$DATA_ROOT/maps/map.yaml
    default_map = path_join(data_root, "maps", "map.yaml")

    # nav2_param:=$PKG_DIR/param/nav2_params.yaml
    default_nav2_param_file = path_join(pkg_dir, "param", "waffle_pi.yaml")

    env_gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        [
            PathJoinSubstitution([data_root, "models"]),
            ":",
            PathJoinSubstitution([pkg_turtlebot3_gazebo, "models"]),
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    env_gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        [
            PathJoinSubstitution([data_root, "worlds"]),
            ":",
            PathJoinSubstitution([pkg_turtlebot3_gazebo, "worlds"]),
            ":",
            EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "data_root",
                default_value=default_data_root,
                description="Full path to data directory",
            ),
            DeclareLaunchArgument(
                "turtlebot3_model",
                default_value=default_turtlebot3_model,
                description="Turtlebot3 model to use",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Full path to world file to load",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "nav2_param_file",
                default_value=default_nav2_param_file,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "test_points",
                default_value=default_test_points_path,
                description="Full path to config file to load",
            ),
            DeclareLaunchArgument(
                "report_path",
                default_value=default_report_path,
                description="Full path to report directory",
            ),
            # export GAZEBO_MODEL_PATH=$DATA_ROOT/models:$GAZEBO_MODEL_PATH
            env_gazebo_model_path,
            # export GAZEBO_RESOURCE_PATH=$DATA_ROOT/worlds:$GAZEBO_RESOURCE_PATH
            env_gazebo_resource_path,
            # ros2 launch coscene_recorder record.launch.py
            include_pkg_py_launch("coscene_recorder", "record.launch.py"),
            # ros2 launch cobridge cobridge_launch.xml
            include_pkg_xml_launch("cobridge", "cobridge_launch.xml"),
            # ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
            include_pkg_py_launch(
                "nav_test",
                "turtlebot3_world.launch.py",
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                }.items(),
            ),
            # ros2 launch turtlebot3_navigation2 navigation2.launch.py
            include_pkg_py_launch(
                "turtlebot3_navigation2",
                "navigation2.launch.py",
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_param_file"),
                    "use_sim_time": "true",
                }.items(),
            ),
            # ros2 run nav_test nav_controller
            Node(
                package="nav_test",
                executable="nav_controller",
                name="nav_controller",
                output="screen",
                parameters=[
                    {"config_path": LaunchConfiguration("test_points")},
                    {"report_path": LaunchConfiguration("report_path")},
                ],
            ),
        ]
    )
