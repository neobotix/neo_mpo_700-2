# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    OpaqueFunction
    )
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    world,
                    arm_type,
                    imu_enable,
                    d435_enable,
                    scanner_type,
                    gripper_type,
                    docking_adapter):    

    launch_actions = []

    robot_type = "mpo_700"
    world_name = str(world.perform(context))

    # Launch bringup_sim file from mp_bringup package
    bringup_sim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mp_bringup'), 'launch', 'bringup_sim.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'robot_type': robot_type,
            'world': world_name,
            'arm_type': arm_type,
            'imu_enable': imu_enable,
            'd435_enable': d435_enable,
            'scanner_type': scanner_type,
            'gripper_type': gripper_type,
            'use_docking_adapter': docking_adapter
        }.items(),
    )

    # Add the launch command to the launch actions
    launch_actions.append(bringup_sim_launch_cmd)

    return launch_actions

def generate_launch_description():

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )

    declare_world_name_arg = DeclareLaunchArgument(
            'world',
            default_value='neo_workshop',
            choices=['', 'neo_workshop'],
            description='Simulation world to load'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Arm Types\n\t'        
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Intel RealSense D435 camera if true'
        )

    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )

    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Gripper Types\n\t'
        )

    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='False',
            description='Enable docking adapter if true'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('world'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('use_docking_adapter')
        ])

    ld = LaunchDescription([
        declare_namespace_cmd,
        declare_world_name_arg,
        declare_arm_type_cmd,
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_scanner_type_cmd,
        declare_gripper_type_cmd,
        declare_use_docking_adapter_cmd,
        opq_function
    ])
    return ld
