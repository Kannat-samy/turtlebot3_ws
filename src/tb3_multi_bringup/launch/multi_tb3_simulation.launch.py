import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():

    # =========================
    # Packages
    # =========================
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    this_pkg_dir = get_package_share_directory('tb3_multi_bringup')




    # 1. On localise le dossier des modèles
    try:
        turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
        models_dir = os.path.join(turtlebot3_gazebo_dir, 'models')
    except:
        models_dir = "" # Juste au cas où, pour pas que ça plante

    # 2. On crée l'action pour dire à Gazebo où ils sont
    # C'est ÇA qui fait apparaitre les murs
    set_model_path_cmd = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir
    )




    # =========================
    # Robots
    # =========================
    robots = [
        {'name': 'robot1', 'x': -2.0,  'y': -0.5},
        {'name': 'robot2', 'x': -2.0,  'y': 0.0},
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
        default_value=os.path.join(
            turtlebot3_gazebo_dir, 
            'worlds', 
            'turtlebot3_dqn_stage4.world' # Attention: pas d'espace à la fin !
        ),
        description='Full path to world model file to load'
    )
    
    #declare_world = DeclareLaunchArgument(
    #    'world',
    #    default_value=os.path.join(this_pkg_dir, 'worlds', 'world_only.model')
    #)

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(this_pkg_dir, 'maps', 'map_stage4.yaml')
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
        default_value=os.path.join(this_pkg_dir, 'params', 'robot1_nav2.yaml')
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
                        os.path.join(this_pkg_dir, 'launch', 'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': r['name'],
                        'use_namespace': 'True',
                        'map': map_yaml,
                        'use_sim_time': 'True',
                        # ON FORCE LE CHEMIN ABSOLU
                        'params_file': '/home/samy/turtlebot3_ws/src/tb3_multi_bringup/params/robot1_nav2.yaml'
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
    # Launch description
    # =========================
    ld = LaunchDescription()

    ld.add_action(set_model_path_cmd)
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



    return ld
