from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('nav_test')
    
    # 构建配置路径
    config_path = os.path.join(pkg_dir, 'config', 'test_points.yaml')
    
    return LaunchDescription([
        Node(
            package='nav_test',
            executable='nav_controller',
            name='nav_controller',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'report_path': '/home/qingyu/nav_reports'}
            ]
        )
    ])
