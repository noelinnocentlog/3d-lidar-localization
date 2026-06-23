from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='opensource_2d_mapping',
            executable='save_map.py',
            name='save_map',
            output='screen'
        )
    ])