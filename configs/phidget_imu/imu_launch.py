# Author: Pradheep Padmanabhan

import launch
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = launch.substitutions.LaunchConfiguration('namespace', default="")

    config = os.path.join(get_package_share_directory('neo_mpo_700-2'),'configs/phidget_imu','imu.yaml') 

    imu_node = ComposableNodeContainer(
            name='phidget_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    plugin='phidgets::SpatialRosI',
                    name='phidgets_spatial'),
            ],
            output='both',
            parameters=[config],
    )

    return LaunchDescription([
        imu_node
    ])