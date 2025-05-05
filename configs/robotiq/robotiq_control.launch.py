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

# Contributors: Pradheep Padmanabhan, Adarsh Karan K P

import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.conditions import UnlessCondition
import launch_ros
import os
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robotiq_description"
    ).find("robotiq_description")

    default_model_path = os.path.join(
        description_pkg_share, "urdf", "robotiq_2f_140_gripper.urdf.xacro"
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
            name="use_mock_hardware",
            default_value="false",
            description="Mock gripper",
        )
    )

    default_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " ",
            "use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " ",
            "mock_sensor_commands:=",
            LaunchConfiguration("use_mock_hardware"),
        ]
    )

    args.append(
        launch.actions.DeclareLaunchArgument(
            name="robot_description_content",
            default_value=default_robot_description_content,
            description="Robot description XML content",
        )
    )

    args.append(
        launch.actions.DeclareLaunchArgument(
            name="controllers_file",
            default_value="robotiq_2f_140_controllers.yaml",
            description="Gripper controllers file name",
        )
    )

    robot_description_param = {
        "robot_description": ParameterValue(
            LaunchConfiguration("robot_description_content"), value_type=str
        )
    }

    robot_state_pub_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_param],
        namespace="robotiq_gripper",
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    controllers_file = LaunchConfiguration("controllers_file")
    initial_joint_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("robotiq_description"),
            "config",
            controllers_file
        ]),
        allow_substs=True
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="robotiq_gripper",
        parameters=[
            initial_joint_controllers,
        ],
        remappings=[
            ('/robotiq_gripper/robot_description', 'robot_description'),
            ('/robotiq_gripper/joint_states','/joint_states'),
            ('/robotiq_gripper/dynamic_joint_states','/dynamic_joint_states'),
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
        condition=UnlessCondition(
            LaunchConfiguration("use_mock_hardware"),
        ),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
    ]

    return launch.LaunchDescription(args + nodes)
