from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():

    return LaunchDescription([

        GroupAction([
            PushRosNamespace('tb3_1'),

            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': True,

                    # Frames
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'map_frame': 'map',

                    # Lidar (RELATIF au namespace)
                    'scan_topic': 'scan'
                }]
            )
        ])
    ])
