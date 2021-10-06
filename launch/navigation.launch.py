#Neobotix
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_amcl = LaunchConfiguration('use_amcl', default='False')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    namespace = LaunchConfiguration('namespace', default='')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_mpo_700-2'),
            'configs/navigation/maps',
            'test.yaml'))

    param_file_name = 'navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('neo_mpo_700-2'),
            'configs/navigation',
            param_file_name))

    robot_dir = os.path.join(get_package_share_directory('neo_mpo_700-2'), 'configs/navigation/launch')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir}.items()),
    ])
