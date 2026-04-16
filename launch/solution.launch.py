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

    # Uzel pro řízení pohybu (Motion Control)
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
            'linear_speed': 1.0,
            'angular_speed': 0.35,
            'use_sim_time': True
        }]
    )

    # PŘESUNUTO SEM DOVNITŘ: Warehouse manager
    warehouse_manager = Node(
        package='mpc_rbt_student', # Opraveno ze 'solution' na 'student'
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # PŘESUNUTO SEM DOVNITŘ: Behavior Tree server
    bt_server = Node(
        package='mpc_rbt_student', # Opraveno ze 'solution' na 'student'
        executable='bt_server',
        name='bt_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(package_dir, 'config', 'bt_server.yaml')
        ]
    )
    
    # Tady ROSu říkáme, co všechno má reálně spustit (přidány nové uzly nakonec)
    return LaunchDescription([
        localization_node,
        planning_node,   
        motion_control_node,
        rviz_node,
        keyboard_node,
        warehouse_manager,
        bt_server,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        )
    ])
