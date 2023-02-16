# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    head_robot = LaunchConfiguration('head_robot', default='False')
    use_amcl = LaunchConfiguration('use_amcl', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    autostart = LaunchConfiguration('autostart', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_mpo_700-2'),
            'configs/navigation/maps',
            'test1.yaml'))

    param_file_name = 'navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('neo_mpo_700-2'),
            'configs/navigation',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    ld = LaunchDescription()

    # Start navigation and push namespace if and only if the multi robot scenario is set to true. 
    start_navigation = GroupAction([
        PushRosNamespace(
            namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir}.items()),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_dir},
                        {'use_sim_time': use_sim_time},
                        {'frame_id':"mpo_7000map"}]
            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['map_server']}])
        ]
    )

    ld.add_action(start_navigation)

    return ld
