import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('command_listener')
    
    command_listener_input_dev_arg = DeclareLaunchArgument("input_dev", default_value='24')
    command_listener_sample_rate_arg = DeclareLaunchArgument("sample_rate", default_value='16000')

    command_listener_input_dev = LaunchConfiguration("input_dev")
    command_listener_sample_rate = LaunchConfiguration("sample_rate")

    command_listener_node = Node(
            package='command_listener', 
            executable='command_listener', 
            name='command_listener_node',
            output='screen',
            arguments=['--input-device', command_listener_input_dev,
                       '--sample-rate-hz', command_listener_sample_rate]
    )
    
    return LaunchDescription([
        command_listener_input_dev_arg,
        command_listener_sample_rate_arg,
        command_listener_node,
    ])
