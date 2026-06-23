from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'opensource_2d_mapping'
    )

    config_dir = os.path.join(
        pkg_share,
        'config'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=[
            '-configuration_directory',
            config_dir,
            '-configuration_basename',
            'cartographer.lua'
        ],
        remappings=[
            ('scan', '/P1_robot0/lidar_2d'),
            ('odom', '/P1_robot0/wheel_odom'),
            ('imu',  '/P1_robot0/imu'),     # <-- ADDED
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=[
            '-resolution',
            '0.05'
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])