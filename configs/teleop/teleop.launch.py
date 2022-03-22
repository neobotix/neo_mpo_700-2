import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    robot_namespace = launch.substitutions.LaunchConfiguration('namespace', default="")

    config = os.path.join(get_package_share_directory('neo_mpo_700-2'),'configs/teleop','teleop.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_teleop2',
            executable='neo_teleop2_node',
            output='screen',
            name='neo_teleop2_node',
            namespace=robot_namespace,
            parameters = [config])
    ,
     launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            output='screen',
            name='joy_node',
            namespace=robot_namespace,
            parameters = [{'dev': "/dev/input/js0"}, {'deadzone':0.12}])
    ])
