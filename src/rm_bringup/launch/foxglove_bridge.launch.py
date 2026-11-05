import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动支持节点图、话题、参数与 TF 查看功能的 Foxglove Bridge。"""
    config_path = os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'node_params',
        'foxglove_bridge_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8765'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config_path, {
                'address': LaunchConfiguration('address'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            }],
            output='screen',
        ),
    ])
