from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ld19_udp_viz',
            executable='udp_ld19_receiver',
            name='ld19_udp',
            output='screen',
            parameters=[{'udp_port': 6001, 'frame_id': 'ld19'}]
        ),
    ])
