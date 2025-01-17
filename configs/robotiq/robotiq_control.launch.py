# Copyright (c) 2022 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robotiq_description"
    ).find("robotiq_description")
    default_model_path = os.path.join(
        description_pkg_share, "urdf", "robotiq_2f_140_gripper.urdf.xacro"
    )
    default_rviz_config_path = os.path.join(
        description_pkg_share, "rviz", "view_urdf.rviz"
    )

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to gripper URDF file",
        )
    )

    args.append(
        launch.actions.DeclareLaunchArgument(
            name="use_fake_hardware",
            default_value="false",
            description="Mock gripper",
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware")
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    update_rate_config_file = PathJoinSubstitution(
        [
            description_pkg_share,
            "config",
            "robotiq_update_rate.yaml",
        ]
    )

    controllers_file = "robotiq_controllers.yaml"
    initial_joint_controllers = os.path.join(get_package_share_directory("robotiq_description"), 'config', controllers_file)

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="robotiq_gripper",
        parameters=[
            robot_description_param,
            update_rate_config_file,
            initial_joint_controllers,
        ],
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        namespace="robotiq_gripper",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
        remappings=[
            ("/robotiq_gripper/joint_states", "/joint_states"),  # Correctly remap the topic
        ],
        output="screen",
    )

    robotiq_gripper_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        namespace="robotiq_gripper",
        arguments=["robotiq_gripper_controller", "-c", 
            "controller_manager"]
    )

    robotiq_activation_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        namespace="robotiq_gripper",
        arguments=["robotiq_activation_controller", "-c", 
            "controller_manager"],
    )

    remapping_launch = launch_ros.actions.Node(
            package="topic_tools",  # A package for relaying or remapping topics
            executable="relay",          # Relay tool for remapping topics
            name="joint_states_relay",
            parameters=[{'input_topic': "/robotiq_gripper/joint_states",'output_topic': "/joint_states"}]
            )

    nodes = [
        control_node,
	    joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        remapping_launch
    ]

    return launch.LaunchDescription(args + nodes)
