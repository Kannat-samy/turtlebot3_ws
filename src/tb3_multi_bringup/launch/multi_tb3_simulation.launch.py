import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():

    # =========================
    # Packages
    # =========================
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    this_pkg_dir = get_package_share_directory('tb3_multi_bringup')

    # =========================
    # Robots
    # =========================
    robots = [
        {'name': 'robot1', 'x': 0.0,  'y': 0.5},
        {'name': 'robot2', 'x': 0.0,  'y': -0.5},
    ]

    # =========================
    # Launch configs
    # =========================
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # =========================
    # Declare arguments
    # =========================
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(nav2_dir, 'worlds', 'world_only.model')
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_dir, 'maps', 'turtlebot3_world.yaml')
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true'
    )

    declare_use_rsp = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true'
    )

    declare_robot1_params = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(nav2_dir, 'params', 'nav2_multirobot_params_1.yaml')
    )

    # =========================
    # Gazebo
    # =========================
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world
        ],
        output='screen'
    )

    # =========================
    # RViz (UNE SEULE FOIS)
    # =========================
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': 'robot1',
            'use_namespace': 'True',
            'rviz_config': os.path.join(
                this_pkg_dir,
                'rviz',
                'robot1_nav2.rviz'
            )
        }.items()
    )

    # =========================
    # Spawn robots (Gazebo + TF)
    # =========================
    spawn_cmds = []

    for r in robots:
        spawn_cmds.append(
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_dir, 'launch', 'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': r['name'],
                        'use_namespace': 'True',
                        'map': map_yaml,
                        'use_sim_time': 'True',
                        'params_file': LaunchConfiguration(f"{r['name']}_params_file")
                            if r['name'] == 'robot1'
                            else os.path.join(nav2_dir, 'params', 'nav2_multirobot_params_2.yaml'),
                        'autostart': autostart,
                        'use_rviz': 'False',
                        'use_simulator': 'False',
                        'headless': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(r['x'])),
                        'y_pose': TextSubstitution(text=str(r['y'])),
                        'z_pose': '0.01',
                        'roll': '0.0',
                        'pitch': '0.0',
                        'yaw': '0.0',
                        'robot_name': r['name'],
                    }.items()
                ),
                LogInfo(msg=[f"Spawned {r['name']}"])
            ])
        )

    # =========================
    # Nav2 UNIQUEMENT robot1
    # =========================
    nav2_robot1 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': 'robot1',
                'use_namespace': 'True',
                'map': map_yaml,
                'use_sim_time': 'True',
                'params_file': LaunchConfiguration('robot1_params_file'),
                'autostart': autostart,
                'use_rviz': 'False',
            }.items()
        )
    ])

    # =========================
    # Launch description
    # =========================
    ld = LaunchDescription()

    ld.add_action(declare_world)
    ld.add_action(declare_map)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_rsp)
    ld.add_action(declare_robot1_params)

    ld.add_action(gazebo)
    ld.add_action(rviz)

    for c in spawn_cmds:
        ld.add_action(c)

    ld.add_action(nav2_robot1)

    return ld
