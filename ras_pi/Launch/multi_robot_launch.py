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
                'exploration_complete_threshold': 0.90,
                'auto_start_exploration': True,
                'frontier_min_size': 5,
                'frontier_cluster_distance': 1.0,
                'goal_assignment_interval': 3.0,
                'max_exploration_range': 10.0
            }],
            output='screen'
        ),
        
        # Robot controllers - handle individual robot navigation with dynamic awareness
        Node(
            package='multirobot_nav',
            executable='robot_controller',
            name='robot1_controller',
            parameters=[{
                'robot_name': 'robot1',
                'linear_speed': 0.15,
                'angular_speed': 0.3,
                'safe_distance': 0.4,
                'goal_tolerance': 0.3,
                'obstacle_threshold': 0.5,
                'safety_distance': 0.3,
                'reaction_time': 0.2,
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'auto_explore': True,
                'exploration_goal_timeout': 30.0
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
                'goal_tolerance': 0.3,
                'obstacle_threshold': 0.5,
                'safety_distance': 0.3,
                'reaction_time': 0.2,
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'auto_explore': True,
                'exploration_goal_timeout': 30.0
            }],
            output='screen'
        ),
        
        # Multi-robot SLAM - uses real-time odometry data for localization and mapping
        Node(
            package='multirobot_nav',
            executable='multirobot_slam',
            name='multi_robot_slam',
            parameters=[{
                'map_resolution': 0.05,
                'map_width': 2000,
                'map_height': 2000,
                'update_rate': 5.0,
                'decay_rate': 0.95,
                'min_observations': 3,
                'temporal_window': 30.0,
                'dynamic_threshold': 0.3,
                'occupied_threshold': 0.7,
                'free_threshold': 0.3,
                'prior_probability': 0.5
            }],
            output='screen'
        ),
        
        # TF Static transforms - Only sensor transforms needed
        # Map->base_link transforms now published by SLAM system using odometry data
        
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