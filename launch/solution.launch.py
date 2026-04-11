import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    # Nový lokalizační uzel
    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Uzel pro planovani
    planning_node = Node(
        package='mpc_rbt_student',
        executable='planning',  
        name='planning_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # NOVÝ: Uzel pro řízení pohybu (Motion Control)
    motion_control_node = Node(
        package='mpc_rbt_student',
        executable='motion_control', 
        name='motion_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz pro vizualizaci
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Uzel pro ovládání klávesnicí 
    keyboard_node = Node(
        package='mpc_rbt_student',
        executable='keyboard_control',
        name='keyboard_control_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'linear_speed': 1.3,
            'angular_speed': 1.0,
            'use_sim_time': True
        }]
    )
    
    # Tady ROSu říkáme, co všechno má reálně spustit
    return LaunchDescription([
        localization_node,
        planning_node,   
        motion_control_node,
        rviz_node,
        keyboard_node,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0.0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])
