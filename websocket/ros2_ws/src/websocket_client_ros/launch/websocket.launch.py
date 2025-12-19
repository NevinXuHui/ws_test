from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(pkg_dir, 'config', 'config.json')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=config_path,
            description='WebSocket配置文件路径'
        ),
        Node(
            package='websocket_client_ros',
            executable='websocket_node',
            name='websocket_node',
            parameters=[{
                'config_path': LaunchConfiguration('config_path')
            }],
            output='screen'
        )
    ])
