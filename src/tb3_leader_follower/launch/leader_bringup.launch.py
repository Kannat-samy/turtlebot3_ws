#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    tb3_nav_dir = get_package_share_directory('turtlebot3_navigation2')

    params_file = os.path.join(
        tb3_nav_dir,
        'param',
        'burger.yaml'
    )

    rviz_config = os.path.join(
        tb3_nav_dir,
        'rviz',
        'tb3_navigation2.rviz'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_nav_dir,
                'launch',
                'navigation2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': 'True',
            'params_file': params_file,
            'rviz_config': rviz_config
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(nav2)

    return ld
