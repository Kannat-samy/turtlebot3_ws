#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        gazebo_pkg,
        'worlds',
        'turtlebot3_world.world'
    )

    # --- Gazebo (exactement comme le launch officiel qui marche)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # --- Robot 1
    tb3_1 = GroupAction([
        PushRosNamespace('tb3_1'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'tb3_1',
                '-database', 'turtlebot3_burger',
                '-robot_namespace', 'tb3_1',
                '-x', '2.0',
                '-y', '0.0',
                '-z', '0.01'
            ],
            output='screen'
        ),
    ])

    # --- Robot 2
    tb3_2 = GroupAction([
        PushRosNamespace('tb3_2'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'tb3_2',
                '-database', 'turtlebot3_burger',
                '-robot_namespace', 'tb3_2',
                '-x', '2.0',
                '-y', '0.5',
                '-z', '0.01'
            ],
            output='screen'
        ),
    ])

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0, actions=[tb3_1]),
        TimerAction(period=6.0, actions=[tb3_2]),
    ])
