from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stl_visualizer',
            executable='stl_publisher',
            name='stl_visualizer',
            output='screen',
            parameters=[{
                'mesh_path': '/input/models/RMUL2024_world/meshes/RMUL_2024.stl',
                'scale_factor': 0.001
            }]
        )
    ])

