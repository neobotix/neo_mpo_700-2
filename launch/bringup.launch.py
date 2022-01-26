# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    neo_mpo_700 = get_package_share_directory('neo_mpo_700-2')
    robot_namespace = ""

    urdf = os.path.join(get_package_share_directory('neo_mpo_700-2'), 'robot_model/mpo_700', 'mpo_700.urdf')

    with open(urdf, 'r') as infp:  
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,
        parameters=[rsp_params])

    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    kinematics = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mpo_700, 'configs/kinematics', 'kinematics.launch.py')
             ),
             launch_arguments={
                'namespace': robot_namespace
            }.items()
         )

    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mpo_700, 'configs/teleop', 'teleop.launch.py')
             ),
             launch_arguments={
                'namespace': robot_namespace
            }.items()
         )

    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/lidar/sick/s300', 'sick_s300.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    relay_topic = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
            output='screen',
            parameters=[{'input_topic': robot_namespace + "/lidar_1/scan_filtered",'output_topic': robot_namespace + "/scan"},
                        {'input_topic': robot_namespace + "/lidar_2/scan_filtered",'output_topic': robot_namespace + "/scan"}])

    return LaunchDescription([relayboard, start_robot_state_publisher_cmd, laser, kinematics, teleop, relay_topic])
