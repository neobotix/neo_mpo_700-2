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
from neo_nav2_bringup.param_file_utils import generate_final_yaml


def execution_stage(
        context: LaunchContext,
        use_sim_time,
        autostart, namespace, use_multi_robots,
        head_robot, use_amcl, map_dir, 
        param_dir, use_rviz,
        graph_filepath, use_route, route_param_file):

    launch_actions = []

    params = str(param_dir.perform(context))
    use_rout = str(use_route.perform(context)) in ('true', 'True')

    # Set BT paths into the launch context so parameter substitutions can resolve them
    bt_base_path = os.path.join(
        get_package_share_directory('neo_mpo_700-2'),
        'configs',
        'navigation',
        'behavior_trees'
    )

    bt_paths = {
        'nav_to_pose_bt': os.path.join(
            bt_base_path,
            "navigate_w_routing_global_planning_and_control_w_recovery.xml" if use_rout else "navigate_to_pose_w_replanning_and_recovery.xml"
        ),
        'nav_through_poses_bt': os.path.join(
            bt_base_path,
            "navigate_on_route_graph_w_recovery.xml" if use_rout else "navigate_through_poses_w_replanning_and_recovery.xml"
        )
    }
    context.launch_configurations.update(bt_paths)

    # If the parameter file is not provided, use the default one based on the robot type
    if not params:
        params = os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs', 'navigation',
                'navigation.yaml')

    # Generates a final YAML parameter file from the controllers template (with substitutions applied),
    substituted_params, shutdown_handler = generate_final_yaml(
        context,
        params,
        file_name='nav2_params_final.yaml',
        cleanup_enabled=False)

    launch_actions.extend(shutdown_handler)

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
                'params_file': substituted_params,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': substituted_params,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': substituted_params,
                'use_rviz': use_rviz,
                'graph_filepath': graph_filepath,
                'use_route': use_route}.items()
        )
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
                        {'node_names': ['map_server']}]
            )
        ]
    )

    launch_actions.append(start_navigation)
    launch_actions.append(start_map_server)

    # Start the service-controlled route follower if use_route is set to true.
    start_neo_route = Node(
        package='neo_nav2_bringup',
        condition=IfCondition(use_route),
        executable='neo_route.py',
        name='neo_route_node',
        output='screen',
        parameters=[str(route_param_file.perform(context)), {'use_sim_time': use_sim_time}]
    )

    launch_actions.append(start_neo_route)

    return launch_actions

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
    graph_filepath = LaunchConfiguration('graph_filepath')
    use_route = LaunchConfiguration('use_route')
    route_param_file = LaunchConfiguration('route_config')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if true'
        )

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='True',
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

    declare_graph_filepath_cmd = DeclareLaunchArgument(
            'graph_filepath', default_value=os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs',
                'navigation',
                'graphs',
                'neo_workshop.geojson'),
            description='Full path to the graph file for route planning'
        )

    declare_use_route_cmd = DeclareLaunchArgument(
            'use_route', default_value='False',
            description='Enable route navigation and launch the idle, service-controlled route follower'
        )

    declare_route_param_cmd = DeclareLaunchArgument(
            'route_config', default_value=os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs',
                'navigation',
                'route.yaml'),
            description='YAML file containing neo_route parameters'
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
    launch_desc.add_action(declare_graph_filepath_cmd)
    launch_desc.add_action(declare_use_route_cmd)
    launch_desc.add_action(declare_route_param_cmd)

    context_arguments = [use_sim_time, autostart, namespace,
                         use_multi_robots, head_robot, use_amcl, map_dir, param_dir,
                         use_rviz, graph_filepath, use_route, route_param_file]

    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)

    launch_desc.add_action(opq_function)

    return launch_desc
