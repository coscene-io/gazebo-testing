import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def include_pkg_launch(package, launch_file, launch_arguments=None, **kwargs):
    launch_path = os.path.join(
        get_package_share_directory(package),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        (
            PythonLaunchDescriptionSource(launch_path)
            if launch_file.endswith(".py")
            else AnyLaunchDescriptionSource(launch_path)
        ),
        launch_arguments=launch_arguments,
        **kwargs,
    )


def set_path_env(name, paths):
    """
    Sets environment variables by concatenating multiple paths with colons.
    For example, if you pass ['path1', 'path2', 'path3'],
    it will set the environment variable to 'path1:path2:path3'.
    """
    value = []
    for i, path in enumerate(paths):
        if i > 0:
            value.append(":")  # 添加分隔符
        value.append(path)  # 添加路径
    return SetEnvironmentVariable(name, value)


def path_join(*path_list: str) -> str:
    return PathJoinSubstitution(path_list)


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_test")
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    data_root = LaunchConfiguration("data_root")

    ##################################################################
    # region default values
    ##################################################################

    # data_root:=$DATA_ROOT
    default_data_root = "/cos/files"

    # turtlebot3_model:=waffle_pi
    default_turtlebot3_model = "waffle_pi"

    # world:=$DATA_ROOT/worlds/RMUL2024_world_dynamic_obstacles.world
    default_world = path_join(
        data_root, "worlds", "RMUL2024_world_dynamic_obstacles.world"
    )

    # --params-file $DATA_ROOT/param/case.yaml
    default_params = path_join(data_root, "param", "case.yaml")

    # map:=$DATA_ROOT/maps/map.yaml
    default_map = path_join(data_root, "maps", "map.yaml")

    # nav2_param:=$PKG_DIR/param/waffle_pi.yaml
    default_nav2_params = path_join(pkg_dir, "param", "waffle_pi.yaml")
    # endregion

    ##################################################################
    # region launch arguments
    ##################################################################

    data_root_arg = DeclareLaunchArgument(
        "data_root",
        default_value=default_data_root,
        description="Full path to data directory",
    )
    turtlebot3_model_arg = DeclareLaunchArgument(
        "turtlebot3_model",
        default_value=default_turtlebot3_model,
        description="Turtlebot3 model to use",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to world file to load",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map file to load",
    )
    mesh_server_url_arg = DeclareLaunchArgument(
        "mesh_server_url",
        default_value="",
        description="Full path to mesh server url",
    )
    main_params_arg = DeclareLaunchArgument(
        "main_params",
        default_value=default_params,
        description="Full path to params file to load",
    )
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=default_nav2_params,
        description="Full path to params file to load",
    )
    # endregion
    
    ##################################################################
    # region environment variables
    ##################################################################

    # export GAZEBO_MODEL_PATH=$DATA_ROOT/models:$GAZEBO_MODEL_PATH
    gazebo_model_path_env = set_path_env(
        "GAZEBO_MODEL_PATH",
        [
            path_join(data_root, "models"),
            path_join(pkg_turtlebot3_gazebo, "models"),
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    # export GAZEBO_RESOURCE_PATH=$DATA_ROOT/worlds:$GAZEBO_RESOURCE_PATH
    gazebo_resource_path_env = set_path_env(
        "GAZEBO_RESOURCE_PATH",
        [
            path_join(data_root, "worlds"),
            # path_join(pkg_turtlebot3_gazebo, "worlds"),
            EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    # export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL
    turtlebot3_model_env = SetEnvironmentVariable(
        "TURTLEBOT3_MODEL",
        LaunchConfiguration("turtlebot3_model"),
    )
    # endregion

    ##################################################################
    # region launch files
    ##################################################################
    
    # ros2 launch coscene_recorder record.launch.py
    record_launch = include_pkg_launch("coscene_recorder", "record.launch.py")
    # ros2 launch cobridge cobridge_launch.xml
    cobridge_launch = include_pkg_launch("cobridge", "cobridge_launch.xml")

    # ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    turtlebot3_world_launch = include_pkg_launch(
        "nav_test",
        "turtlebot3_world.launch.py",
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    # ros2 launch turtlebot3_navigation2 navigation2.launch.py
    turtlebot3_navigation2_launch = include_pkg_launch(
        "turtlebot3_navigation2",
        "navigation2.launch.py",
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("nav2_params"),
            "use_sim_time": "true",
        }.items(),
    )
    # endregion

    ##################################################################
    # region nodes
    ##################################################################

    # marker_publisher
    marker_publisher_node = Node(
        package="marker_publisher",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("main_params"),
            {
                "mesh_server_url": LaunchConfiguration("mesh_server_url"),
            },
        ],
    )

    # ros2 run nav_test nav_controller
    nav_controller_node = Node(
        package="nav_test",
        executable="nav_controller",
        name="nav_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("main_params"),
        ],
    )

    # ros2 run nav_test node_lister
    node_lister_node = Node(
        package="node_lister",
        executable="node_lister",
        name="node_lister",
        output="screen",
    )
    # endregion

    return LaunchDescription(
        [
            # arguments
            data_root_arg,
            turtlebot3_model_arg,
            world_arg,
            map_arg,
            nav2_params_arg,
            main_params_arg,
            mesh_server_url_arg,
            # environment variables
            gazebo_model_path_env,
            gazebo_resource_path_env,
            # launch files
            turtlebot3_world_launch,
            turtlebot3_navigation2_launch,
            record_launch,
            cobridge_launch,
            # nodes
            node_lister_node,
            nav_controller_node,
            marker_publisher_node,
        ]
    )
