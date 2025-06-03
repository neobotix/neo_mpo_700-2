# Neobotix GmbH
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_context import LaunchContext
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of the robot in Rviz. 
"""

def execution_stage(context: LaunchContext, 
                    use_sim_time, 
                    use_joint_state_publisher_gui, 
                    arm_type,
                    imu_enable,
                    d435_enable,
                    scanner_type,
                    gripper_type,
                    docking_adapter,
                    display_mode,
                    ur_dc,
                    rviz_config):

    launch_actions = []

    neo_mpo_700 = get_package_share_directory('neo_mpo_700-2')

    # Resolve launch arguments
    arm_typ = str(arm_type.perform(context))
    use_ur_dc = ur_dc.perform(context)
    imu_enabl = str(imu_enable.perform(context))
    d435_enabl = str(d435_enable.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    gripper_typ = str(gripper_type.perform(context))
    use_docking_adapter = str(docking_adapter.perform(context))
    use_sim_tim = use_sim_time.perform(context)
    rviz_config_file = str(rviz_config.perform(context))
    use_joint_state_publisher_gui = use_joint_state_publisher_gui.perform(context)
    display_mod = display_mode.perform(context)

    # Robot description package for mpo_700 robot
    urdf = os.path.join(neo_mpo_700,
        'robot_model',
        'mpo_700.urdf.xacro')

    xacro_args = [
        "xacro", " ", urdf,
        " ", 'use_gz:=true',
        " ", 'arm_type:=', arm_typ,
        " ", 'use_ur_dc:=', use_ur_dc,
        " ", 'force_abs_path:=true',
        " ", 'gripper_type:=', gripper_typ,
        " ", 'use_imu:=', imu_enabl,
        " ", 'use_d435:=', d435_enabl,
        " ", 'scanner_type:=', scanner_typ,
        " ", 'use_docking_adapter:=', use_docking_adapter,
    ]

    # Launch bringup_sim file from mp_bringup package
    rviz_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mp_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
        'use_sim_time': use_sim_tim,
        'use_joint_state_publisher_gui': use_joint_state_publisher_gui,
        'display_mode': display_mod,
        'rviz_config': rviz_config_file,
        'robot_description_content': Command(xacro_args)
        }.items(),
    )

    # Add the launch command to the launch actions
    launch_actions.append(rviz_bringup_cmd)

    return launch_actions

def generate_launch_description():

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )

    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Arm Types'
        )

    declare_use_imu_cmd = DeclareLaunchArgument(
            'use_imu', default_value='False',
            description='Enable IMU sensors if true'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'use_d435', default_value='False',
            description='Enable Intel RealSense D435 camera if true'
        )
    
    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )

    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Gripper Types'
        )

    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='False',
            description='Enable docking adapter if true'
        )

    declare_use_display_mode_cmd = DeclareLaunchArgument(
            'display_mode', default_value='True',
            description='Disable robot and joint state publishers if true (True/False)'
        )

    declare_ur_pwr_variant_cmd = DeclareLaunchArgument(
            'use_ur_dc', default_value='False',
            description='Set this argument to True if you have an UR arm with DC variant'
        )

    declare_rviz_cfg_arg = DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
            get_package_share_directory('neo_mpo_700-2'),
            'configs', 'rviz', 'robot_description_rviz.rviz'),
            description='Full path to an RViz config file'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('use_joint_state_publisher_gui'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('use_imu'),
            LaunchConfiguration('use_d435'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('use_docking_adapter'),
            LaunchConfiguration('display_mode'),
            LaunchConfiguration('use_ur_dc'),
            LaunchConfiguration('rviz_config')
        ])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_joint_state_publisher_gui_arg,
        declare_arm_type_cmd,
        declare_use_imu_cmd,
        declare_realsense_cmd,
        declare_scanner_type_cmd,
        declare_gripper_type_cmd,
        declare_use_docking_adapter_cmd,
        declare_use_display_mode_cmd,
        declare_ur_pwr_variant_cmd,
        declare_rviz_cfg_arg,
        opq_function
    ])
