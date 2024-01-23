# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
import xacro
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  ExecuteProcess,
  OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.launch_context import LaunchContext
from launch.conditions import IfCondition

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    imu_enable,
                    d435_enable,
                    arm_type):
    
    arm_typ = str(arm_type.perform(context))
    imu_enabl = str(imu_enable.perform(context))
    d435_enabl = str(d435_enable.perform(context))

    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"

    launches = []

    # Setting up the URDF
    urdf = os.path.join(get_package_share_directory('neo_mpo_700-2'),
        'robot_model/mpo_700',
        'mpo_700.urdf.xacro')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': Command([
            "xacro", " ", urdf, " ", 'arm_type:=',
            arm_typ,
            " ", 'use_imu:=',
            imu_enabl,
            " ", 'use_d435:=',
            d435_enabl
            ]), 'frame_prefix': rp_ns}],
		arguments=[urdf])

    launches.append(start_robot_state_publisher_cmd)

    # 5. IMU
    imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_mpo_700-2'),
                    'configs/phidget_imu',
                    'imu_launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=IfCondition(imu_enable)
        )
    
    launches.append(imu)

    # 6. D435
    # TODO: Add support for namespacing
    d435 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py')
            ),
            condition=IfCondition(d435_enable)
        )

    launches.append(d435)

    # 7. Arm - Bringing up drivers for Universal Arm
    # TODO: Add support for Elite Robots
    # TODO: Add support for namespacing
    if (arm_typ == "ur5" or
        arm_typ == "ur10" or
        arm_typ == "ur5e" or
        arm_typ == "ur10e"):
        
        ur_arm = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ur_robot_driver'),
                        'launch',
                        'ur_control.launch.py')
                ),
                launch_arguments={
                    'ur_type': arm_typ,
                    'robot_ip': "192.168.0.101",
                    'launch_rviz': False
                }.items()
            )

        launches.append(ur_arm)

    # Relaying lidar data to /scan topic
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

    launches.append(relay_topic_lidar1)
    launches.append(relay_topic_lidar2)

    return launches

def generate_launch_description():
    neo_mpo_700 = get_package_share_directory('neo_mpo_700-2')

    # Launch configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    imu_enable = LaunchConfiguration('imu_enable')
    realsense_enable = LaunchConfiguration('d435_enable')
    arm_type = LaunchConfiguration('arm_type')

    context_arguments = [robot_namespace, imu_enable, realsense_enable, arm_type]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )
    
    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
    
    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Realsense - Options: True/False'
        )
    
    declare_arm_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            description='Arm used in the robot - currently only support universal'
        )
    
    #  Launch hardware nodes
    # 1. Relayboard
    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 2. Kinematics
    kinematics = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/kinematics', 'kinematics.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 3. Teleop
    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mpo_700, 'configs/teleop', 'teleop.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 4. Laser
    # TODO: Add support for MicroScan3
    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/lidar/sick/s300', 'sick_s300.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )
    
    # Opaque function for configuring URDF, IMU, Realsense and the Arm
    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_imu_cmd)
    ld.add_action(declare_realsense_cmd)
    ld.add_action(declare_arm_cmd)
    ld.add_action(relayboard)
    ld.add_action(kinematics)
    ld.add_action(teleop)
    ld.add_action(opq_function)
    ld.add_action(laser)

    return ld
