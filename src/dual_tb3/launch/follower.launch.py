from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    follower_node = Node(
        package='tb3_leader_follower',
        executable='follower_node',
        name='tb3_follower',
        namespace='tb3_2',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('/cmd_vel', '/tb3_2/cmd_vel'),
            ('/leader_odom', '/tb3_1/odom')
        ]
    )

    return LaunchDescription([
        follower_node
    ])
