from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # micro-ROS agents for ESP32 communication
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            name='micro_ros_agent_robot1',
            output='screen'
        ),
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8889'],
            name='micro_ros_agent_robot2',
            output='screen'
        ),
        
        # Robot coordinator - manages exploration and coordination
        Node(
            package='multirobot_nav',
            executable='robot_coordinator',
            name='robot_coordinator',
            parameters=[{
                'coordination_strategy': 'frontier_based',
                'min_robot_distance': 1.5,
                'exploration_complete_threshold': 0.90
            }],
            output='screen'
        ),
        
        # Robot controllers - handle individual robot navigation
        Node(
            package='multirobot_nav',
            executable='robot_controller',
            name='robot1_controller',
            parameters=[{
                'robot_name': 'robot1',
                'linear_speed': 0.15,
                'angular_speed': 0.3,
                'safe_distance': 0.4,
                'goal_tolerance': 0.3
            }],
            output='screen'
        ),
        
        Node(
            package='multirobot_nav',
            executable='robot_controller',
            name='robot2_controller',
            parameters=[{
                'robot_name': 'robot2',
                'linear_speed': 0.15,
                'angular_speed': 0.3,
                'safe_distance': 0.4,
                'goal_tolerance': 0.3
            }],
            output='screen'
        ),
        
        # Multi-robot SLAM - builds unified map from both robots
        Node(
            package='multirobot_nav',
            executable='multi_robot_slam',
            name='multi_robot_slam',
            parameters=[{
                'map_resolution': 0.05,
                'map_width': 2000,
                'map_height': 2000,
                'update_rate': 5.0
            }],
            output='screen'
        ),
        
        # TF Static transforms - Define coordinate frame relationships
        # Map frame to robot odometry frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot1/odom'],
            name='map_to_robot1_odom'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot2/odom'],
            name='map_to_robot2_odom'
        ),
        
        # CRITICAL: Robot odometry to base_link frames (These were missing!)
        # The ESP32 publishes odometry with these frame relationships
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot1/base_link'],
            name='robot1_odom_to_base_link'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot2/odom', 'robot2/base_link'],
            name='robot2_odom_to_base_link'
        ),
        
        # Robot base_link to LiDAR sensor frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'robot1/base_link', 'robot1/lidar_link'],
            name='robot1_base_to_lidar'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'robot2/base_link', 'robot2/lidar_link'],
            name='robot2_base_to_lidar'
        )
    ])