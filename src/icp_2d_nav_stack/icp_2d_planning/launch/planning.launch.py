#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('icp_2d_planning')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='icp_2d_planning',
            executable='hybrid_astar_planner',
            name='hybrid_astar_planner',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'planner.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
