#!/usr/bin/env python3
"""
Robot Pose Initializer
Sets the initial poses for both robots at system startup
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class RobotPoseInitializer(Node):
    def __init__(self):
        super().__init__('robot_pose_initializer')
        
        # Publishers for initial poses
        self.robot1_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot1/initialpose', 10)
        self.robot2_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot2/initialpose', 10)
        
        # Wait for publishers to be ready
        time.sleep(2)
        
        # Timer to publish poses multiple times for reliability
        self.pose_timer = self.create_timer(1.0, self.publish_poses)
        self.publish_count = 0
        self.max_publishes = 10
        
        self.get_logger().info('Robot Pose Initializer started')

    def publish_poses(self):
        if self.publish_count >= self.max_publishes:
            self.get_logger().info('Initial poses published successfully')
            self.pose_timer.cancel()
            rclpy.shutdown()
            return
        
        # Robot 1: Position (0,0,0) with orientation facing +X (0 degrees)
        pose1_msg = PoseWithCovarianceStamped()
        pose1_msg.header.stamp = self.get_clock().now().to_msg()
        pose1_msg.header.frame_id = 'map'
        
        # Position
        pose1_msg.pose.pose.position.x = 0.0
        pose1_msg.pose.pose.position.y = 0.0
        pose1_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing +X, 0 degrees)
        pose1_msg.pose.pose.orientation.x = 0.0
        pose1_msg.pose.pose.orientation.y = 0.0
        pose1_msg.pose.pose.orientation.z = 0.0
        pose1_msg.pose.pose.orientation.w = 1.0
        
        # Covariance (confidence in pose)
        pose1_msg.pose.covariance = [0.0] * 36
        pose1_msg.pose.covariance[0] = 0.1  # x variance
        pose1_msg.pose.covariance[7] = 0.1  # y variance
        pose1_msg.pose.covariance[35] = 0.1 # yaw variance
        
        # Robot 2: Position (1,0,0) with orientation facing -X (180 degrees)
        pose2_msg = PoseWithCovarianceStamped()
        pose2_msg.header.stamp = self.get_clock().now().to_msg()
        pose2_msg.header.frame_id = 'map'
        
        # Position
        pose2_msg.pose.pose.position.x = 1.0
        pose2_msg.pose.pose.position.y = 0.0
        pose2_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing -X, 180 degrees)
        yaw = math.pi  # 180 degrees in radians
        pose2_msg.pose.pose.orientation.x = 0.0
        pose2_msg.pose.pose.orientation.y = 0.0
        pose2_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose2_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance
        pose2_msg.pose.covariance = [0.0] * 36
        pose2_msg.pose.covariance[0] = 0.1  # x variance
        pose2_msg.pose.covariance[7] = 0.1  # y variance
        pose2_msg.pose.covariance[35] = 0.1 # yaw variance
        
        # Publish poses
        self.robot1_pose_pub.publish(pose1_msg)
        self.robot2_pose_pub.publish(pose2_msg)
        
        self.publish_count += 1
        self.get_logger().info(f'Published initial poses (attempt {self.publish_count})')
        self.get_logger().info(f'Robot1: (0.0, 0.0, 0°) | Robot2: (1.0, 0.0, 180°)')

def main(args=None):
    rclpy.init(args=args)
    
    initializer = RobotPoseInitializer()
    
    try:
        rclpy.spin(initializer)
    except KeyboardInterrupt:
        pass
    finally:
        initializer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()