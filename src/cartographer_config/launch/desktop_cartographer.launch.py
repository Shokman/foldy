import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('cartographer_ros')
    
    path_rviz = os.path.join(pkg_share_dir, 'configuration_files/demo_2d.rviz')

    rviz_node = Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', path_rviz]
    )

    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            name='teleop_twist_keyboard',    
    )
    
    return LaunchDescription([
        rviz_node,
        teleop_twist_keyboard_node
    ])
