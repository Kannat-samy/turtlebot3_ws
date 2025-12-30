import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. Dossiers des packages
    pkg_gazebo_ros = get_package_share_directory('turtlebot3_gazebo')
    pkg_cartographer = get_package_share_directory('turtlebot3_cartographer')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # --- CORRECTION 1 : Fichier params Nav2 (Anti-Crash) ---
    nav2_params_path = os.path.join(pkg_nav2, 'params', 'nav2_params.yaml')
    
    # 2. Définition des lancements
    
    # A. Gazebo (Ton monde Stage 4)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'turtlebot3_dqn_stage4.launch.py')
        )
    )

    # B. Cartographer (SLAM)
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # C. Nav2 (Navigation)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True', 
            'params_file': nav2_params_path # On utilise le fichier par défaut
        }.items() 
    )

    # D. Ton script Auto Explorer
    # --- CORRECTION 2 : Le chemin vers ton script à la racine ---
    explorer_cmd = ExecuteProcess(
        cmd=['python3', '/home/samy/turtlebot3_ws/auto_explorer.py'],
        output='screen'
    )

    # 3. Chronologie (Timers)
    ld = LaunchDescription()

    # Lancement immédiat de Gazebo
    ld.add_action(gazebo_cmd)

    # +10s : Cartographer
    ld.add_action(TimerAction(period=10.0, actions=[cartographer_cmd]))

    # +20s : Nav2 (Navigation)
    ld.add_action(TimerAction(period=20.0, actions=[nav2_cmd]))

    # +30s : Ton script (Explorer)
    ld.add_action(TimerAction(period=30.0, actions=[explorer_cmd]))

    return ld