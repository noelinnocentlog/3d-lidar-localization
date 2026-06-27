from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_dir',
            default_value='',
            description='Directory where maps will be saved. '
                        'Leave empty to save to maps/ at the workspace root.',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='Base name for the saved map (a number suffix is appended automatically)',
        ),
        Node(
            package='opensource_2d_mapping',
            executable='save_map.py',
            name='save_map',
            output='screen',
            parameters=[{
                'map_dir': LaunchConfiguration('map_dir'),
                'map_name': LaunchConfiguration('map_name'),
            }],
        ),
    ])
