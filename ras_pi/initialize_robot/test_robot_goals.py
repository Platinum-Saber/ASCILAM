#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalTester(Node):
    def __init__(self):
        super().__init__('goal_tester')
        
        # Publishers for robot goals
        self.robot1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        self.get_logger().info('Goal Tester initialized')
        
        # Wait a bit for publishers to be ready
        time.sleep(2)
        
        self.send_test_goals()
        
    def send_test_goals(self):
        """Send simple test goals to both robots"""
        
        # Goal for robot1 - move forward 2 meters in +X
        goal1 = PoseStamped()
        goal1.header.stamp = self.get_clock().now().to_msg()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 2.0
        goal1.pose.position.y = 0.0
        goal1.pose.position.z = 0.0
        goal1.pose.orientation.w = 1.0
        
        # Goal for robot2 - move backward 2 meters in -X
        goal2 = PoseStamped()
        goal2.header.stamp = self.get_clock().now().to_msg()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = -1.0
        goal2.pose.position.y = 0.0
        goal2.pose.position.z = 0.0
        goal2.pose.orientation.w = 1.0
        
        # Publish goals
        self.robot1_goal_pub.publish(goal1)
        self.robot2_goal_pub.publish(goal2)
        
        self.get_logger().info('Published test goals:')
        self.get_logger().info('  Robot1: (2.0, 0.0)')
        self.get_logger().info('  Robot2: (-1.0, 0.0)')
        self.get_logger().info('Check if robots start moving!')

def main(args=None):
    rclpy.init(args=args)
    node = GoalTester()
    
    # Keep node alive for a few seconds
    time.sleep(5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()