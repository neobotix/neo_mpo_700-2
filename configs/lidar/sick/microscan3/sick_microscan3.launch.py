from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "lidar_1_link",
                 "sensor_ip": "192.168.1.30",
                 "host_ip": "192.168.1.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 6060,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 1,
                 "angle_start": -1.48,
                 "angle_end": 1.48,
                 "time_offset": -0.07,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[
                ('/scan', '/lidar_1/scan_filtered'),
            ]
        ), Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node1",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "lidar_2_link",
                 "sensor_ip": "192.168.1.31",
                 "host_ip": "192.168.1.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 6061,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 1,
                 "angle_start": -1.48,
                 "angle_end": 1.48,
                 "time_offset": -0.07,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[
                ('/scan', '/lidar_2/scan_filtered'),
            ]
        )
    ])
