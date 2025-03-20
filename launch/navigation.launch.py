# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  GroupAction,
  OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext


def execution_stage(
        context: LaunchContext,
        use_sim_time,
        autostart, namespace, use_multi_robots,
        head_robot, use_amcl, map_dir, param_dir, use_rviz):
    
    launches = []

    params = str(param_dir.perform(context))

    # If the parameter file is not provided, use the default one based on the robot type
    if not params:
        params = os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs', 'navigation',
                'navigation.yaml')
        
    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    # Start navigation and push namespace if and only if the multi robot scenario is set to true. 
    start_navigation = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_multi_robots),
            namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': params,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': params,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': params,
                              'use_rviz': use_rviz,
                              }.items()),
    ])

    # Start map_server if this robot is assigned as the head robot and if there is no multi-robot,
    # neo_bringup handles the map_server
    start_map_server = GroupAction(
        condition=IfCondition(head_robot),
        actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_dir},
                        {'use_sim_time': use_sim_time}]
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

    launches.append(start_navigation)
    launches.append(start_map_server)

    return launches

def generate_launch_description():
    launch_desc = LaunchDescription()
    use_multi_robots = LaunchConfiguration('use_multi_robots')
    head_robot = LaunchConfiguration('head_robot')
    use_amcl = LaunchConfiguration('use_amcl')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('robot_namespace')
    map_dir = LaunchConfiguration('map')
    param_dir = LaunchConfiguration('nav2_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'
        )
    
    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically start the nav2 stack'
        )
    
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='',
            description='Top-level namespace'
        )
    
    declare_use_multi_robots_cmd = DeclareLaunchArgument(
            'use_multi_robots', default_value='False',
            description='Use multi robots'
        )
    
    declare_head_robot_cmd = DeclareLaunchArgument(
            'head_robot', default_value='False',
            description='Head robot'
        )
    
    declare_use_amcl_cmd = DeclareLaunchArgument(
            'use_amcl', default_value='False',
            description='Use AMCL'
        )
    
    declare_map_cmd = DeclareLaunchArgument(
            'map', default_value=os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs',
                'navigation',
                'maps',
                'neo_workshop.yaml'),
            description='Full path to map file to load'
        )
    
    declare_nav2_param_file_cmd = DeclareLaunchArgument(
            'nav2_params_file', default_value="",
            description='Full path to the Nav2 parameters file to load.\n'
                        '\tLeave empty to use the default file based on the robot type'
        )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Launch RViz for visualization'
        )
    
    # Adding all the necessary launch description actions
    launch_desc.add_action(declare_use_sim_time_cmd)
    launch_desc.add_action(declare_autostart_cmd)
    launch_desc.add_action(declare_namespace_cmd)
    launch_desc.add_action(declare_use_multi_robots_cmd)
    launch_desc.add_action(declare_head_robot_cmd)
    launch_desc.add_action(declare_use_amcl_cmd)
    launch_desc.add_action(declare_map_cmd)
    launch_desc.add_action(declare_nav2_param_file_cmd)
    launch_desc.add_action(declare_use_rviz_cmd)

    context_arguments = [use_sim_time, autostart, namespace,
                         use_multi_robots, head_robot, use_amcl, map_dir, param_dir, use_rviz]

    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)

    launch_desc.add_action(opq_function)

    return launch_desc