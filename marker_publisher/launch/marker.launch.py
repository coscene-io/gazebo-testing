import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("marker_publisher")

    # param_file:=$PKG_DIR/param/default.yaml
    default_params_file = os.path.join(pkg_dir, "param", "default.yaml")

    ##################################################################
    # launch arguments | environment variables | launch files | nodes
    ##################################################################
    param_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to parameter file",
    )

    mesh_server_url_arg = DeclareLaunchArgument(
        "mesh_server_url",
        default_value=EnvironmentVariable("COS_HTTP_FORWARD_URL", default_value=""),
        description="Mesh server URL from COS_HTTP_FORWARD_URL env or override manually",
    )

    # marker_publisher
    marker_publisher_node = Node(
        package="marker_publisher",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "mesh_server_url": LaunchConfiguration("mesh_server_url"),
            },
        ],
    )

    return LaunchDescription([
        # Launch arguments
        param_file_arg,
        mesh_server_url_arg,
        # Node
        marker_publisher_node,
    ])
