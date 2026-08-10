import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('motion_control'), 'config', 'eevee_config.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Full path to a filter config YAML'
    )

    filter_runner = Node(
        package='lowpass_filter',
        executable='main',
        arguments=[LaunchConfiguration('config')],
        output='screen'
    )

    return LaunchDescription([config_arg, filter_runner])