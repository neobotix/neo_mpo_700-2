# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
import xacro
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition, UnlessCondition

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    imu_enable,
                    d435_enable,
                    scanner_type,
                    docking_adapter,
                    arm_type,
                    gripper_type,
                    mock_arm,
                    initial_joint_controller,
                    robot_ip,
                    controllers_yaml):

    neo_mpo_700 = get_package_share_directory('neo_mpo_700-2')

    imu_enabl = str(imu_enable.perform(context))
    d435_enabl = str(d435_enable.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    arm_typ = str(arm_type.perform(context))
    gripper_typ = str(gripper_type.perform(context))
    use_docking_adapter = str(docking_adapter.perform(context))
    use_mock = str(mock_arm.perform(context))

    launches = []

    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"

    # Setting up the URDF
    urdf = os.path.join(neo_mpo_700,
        'robot_model',
        'mpo_700.urdf.xacro')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                "xacro", " ", urdf,
                " ", 'arm_type:=', arm_typ,
                " ", 'robot_ip:=', "yyy.yyy.yyy.yyy",
                " ", 'gripper_type:=', gripper_typ,
                " ", 'use_mock_hardware:=', use_mock,  # experimental
                " ", 'use_mock_sensor_commands:=', use_mock,
                " ", 'use_imu:=', imu_enabl,
                " ", 'use_d435:=', d435_enabl,
                " ", 'scanner_type:=', scanner_typ,
                " ", 'use_docking_adapter:=', use_docking_adapter
            ]),
            value_type=str
            ),
            'frame_prefix': rp_ns
        }],
        arguments=[urdf]
    )

    launches.append(start_robot_state_publisher_cmd)

    #  Launch hardware nodes
    # 1. Relayboard
    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )

    launches.append(relayboard)

    # 2. Kinematics
    kinematics = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, 'configs/kinematics', 'kinematics.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )

    launches.append(kinematics)

    # 3. Teleop
    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mpo_700, 'configs/teleop', 'teleop.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )

    launches.append(teleop)

    # 4. Laser
    scanner_model = scanner_typ.split('_')[1] if '_' in scanner_typ else scanner_typ
    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mpo_700, f'configs/lidar/{scanner_model}', f'{scanner_typ}.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )

    launches.append(laser)

    # 5. IMU
    if imu_enabl.lower == 'true':
        imu = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(neo_mpo_700,
                        'configs/phidget_imu',
                        'imu_launch.py')
                ),
                launch_arguments={
                    'namespace': robot_namespace
                }.items(),
                condition=UnlessCondition(mock_arm)
            )

        launches.append(imu)

    # 6. D435
    # TODO: Add support for namespacing
    if d435_enabl.lower == 'true':
        d435 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(neo_mpo_700,
                        'configs/realsense',
                        'rs_launch.py')
                ),
                condition=UnlessCondition(mock_arm)
            )

        launches.append(d435)

    # 7. Arm - Bringing up drivers for Universal Arm
    # TODO: Add support for Elite Robots
    # TODO: Add support for namespacing
    if (arm_typ == "ur5" or
        arm_typ == "ur10" or
        arm_typ == "ur5e" or
        arm_typ == "ur10e"):

        initial_joint_controller = "scaled_joint_trajectory_controller"
        if use_mock:
            initial_joint_controller = "joint_trajectory_controller"
        ur_arm = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(neo_mpo_700,
                        'configs/ur',
                        'ur_control.launch.py')
                ),
                launch_arguments={
                    'ur_type': arm_typ,
                    'robot_ip': robot_ip,
                    'tf_prefix': arm_typ,
                    'use_mock_hardware': use_mock,
                    'mock_sensor_commands': use_mock,
                    'initial_joint_controller': initial_joint_controller,
                    'gripper_type': gripper_typ,
                    'controllers_file': controllers_yaml,
                }.items()
            )

        launches.append(ur_arm)

        # For 2f_140
        if (gripper_typ == "2f_140"):
            gripper_2f_140 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('neo_mpo_700-2'),
                            'configs/robotiq',
                            'robotiq_control.launch.py')
                    )
                )

            launches.append(gripper_2f_140)

        # For Epick
        elif (gripper_typ == "epick"):
            gripper_epick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('neo_mpo_700-2'),
                            'configs/robotiq',
                            'robotiq_epick_control.launch.py')
                    )
                )

            launches.append(gripper_epick)

    # Relaying lidar data to /scan topic
    relay_topic_lidar1 = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
			namespace =  robot_namespace,
            output='screen',
            parameters=[{'input_topic': robot_namespace.perform(context) + "lidar_1/scan_filtered",'output_topic': robot_namespace.perform(context) + "scan"}],
            condition=UnlessCondition(mock_arm)
            )

    relay_topic_lidar2 = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
			namespace =  robot_namespace,
            output='screen',
            parameters=[{'input_topic': robot_namespace.perform(context) + "lidar_2/scan_filtered",'output_topic': robot_namespace.perform(context) + "scan"}],
            condition=UnlessCondition(mock_arm)
            )

    launches.append(relay_topic_lidar1)
    launches.append(relay_topic_lidar2)

    return launches

def generate_launch_description():

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
    
    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use'
        )
    
    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='False',
            description='Enable docking adapter - Options: True/False'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Arm Types\n\t'        
        )

    declare_robotiq_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', 'epick'],
            description="Enables gripper and it's controllers"
        )

    declare_mock_arm_cmd = DeclareLaunchArgument(
            'use_mock_arm', default_value='False',
            description="Mock arm and gripper (if available)"
        )
    
    declare_initial_joint_controller_cmd = DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='scaled_joint_trajectory_controller',
            choices=[
                'scaled_joint_trajectory_controller',
                'joint_trajectory_controller',
            ],
            description='Initially loaded robot controller.'
        )

    declare_robot_ip_cmd = DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.102',
            description='IP address of the robot arm.'
        )

    declare_controllers_file_cmd = DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(
                get_package_share_directory('neo_mpo_700-2'),
                'configs/ur/ur_controllers.yaml'
            ),
            description='YAML file with the controllers configuration.',
        )

    # Opaque function for configuring URDF, IMU, Realsense and the Arm
    opq_function = OpaqueFunction(
        function=execution_stage, 
        args=[
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('use_docking_adapter'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('use_mock_arm'),
            LaunchConfiguration('initial_joint_controller'),
            LaunchConfiguration('robot_ip'),
            LaunchConfiguration('controllers_file')
            ])

    ld = LaunchDescription([
        declare_namespace_cmd,
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_scanner_type_cmd,
        declare_use_docking_adapter_cmd,
        declare_arm_type_cmd,
        declare_robotiq_cmd,
        declare_mock_arm_cmd,
        declare_initial_joint_controller_cmd,
        declare_robot_ip_cmd,
        declare_controllers_file_cmd,
        opq_function
    ])
    return ld
