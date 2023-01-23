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
from launch.launch_context import LaunchContext

def generate_launch_description():
    neo_mpo_700 = get_package_share_directory('neo_mpo_700-2')
    robot_namespace = LaunchConfiguration('robot_namespace', default='')
    context = LaunchContext()
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    urdf = os.path.join(get_package_share_directory('neo_mpo_700-2'), 'robot_model/mpo_700', 'mpo_700.urdf')

    with open(urdf, 'r') as infp:  
        robot_desc = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace="mpo_7000",
        remappings=remappings,
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf])
    
    # Launch can be set just once, does not matter if you set it for other launch files. 
    # The arguments should certainly have different meaning if there is a bigger launch file
    # Leaving this comment here for a clarity thereof and thereforth. 
    # https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/

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
             )
         )

    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mpo_700, 'configs/teleop', 'teleop.launch.py')
             )
         )

    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/lidar/sick/s300', 'sick_s300.launch.py')
            )
        )

    relay_topic_lidar1 = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
            namespace =  robot_namespace,
            output='screen',
            parameters=[{'input_topic': robot_namespace.perform(context) + "lidar_1/scan_filtered",'output_topic': robot_namespace.perform(context) + "scan"}])

    relay_topic_lidar2 = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
            namespace =  robot_namespace,
            output='screen',
            parameters=[{'input_topic': robot_namespace.perform(context) + "lidar_2/scan_filtered",'output_topic': robot_namespace.perform(context) + "scan"}])

    return LaunchDescription([relayboard, start_robot_state_publisher_cmd, laser, kinematics, teleop, relay_topic_lidar1, relay_topic_lidar2])