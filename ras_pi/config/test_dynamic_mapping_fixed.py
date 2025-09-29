#!/usr/bin/env python3
"""
Dynamic Mapping Test Suite
Tests the dynamic mapping capabilities with both robots
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import time
import numpy as np

class DynamicMappingTester(Node):
    def __init__(self):
        super().__init__('dynamic_mapping_tester')
        
        # Test parameters
        self.test_duration = 300  # 5 minutes
        self.test_start_time = time.time()
        
        # Publishers for robot commands
        self.robot1_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.robot2_cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.robot1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        # Subscribers for monitoring
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Robot states
        self.robot1_pose = None
        self.robot2_pose = None
        self.robot1_scan = None
        self.robot2_scan = None
        self.current_map = None
        
        # Test statistics
        self.stats = {
            'robot1_scans': 0,
            'robot2_scans': 0,
            'map_updates': 0,
            'dynamic_detections': 0,
            'collision_avoidances': 0
        }
        
        # Test phases
        self.current_phase = 0
        self.phase_start_time = time.time()
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test_phase)
        
        self.get_logger().info('Dynamic Mapping Test Suite started')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')

    def robot1_odom_callback(self, msg):
        self.robot1_pose = msg.pose.pose

    def robot2_odom_callback(self, msg):
        self.robot2_pose = msg.pose.pose

    def robot1_scan_callback(self, msg):
        self.robot1_scan = msg
        self.stats['robot1_scans'] += 1
        self.check_collision_avoidance('robot1', msg)

    def robot2_scan_callback(self, msg):
        self.robot2_scan = msg
        self.stats['robot2_scans'] += 1
        self.check_collision_avoidance('robot2', msg)

    def map_callback(self, msg):
        if self.current_map is not None:
            self.stats['map_updates'] += 1
            self.analyze_map_changes(self.current_map, msg)
        self.current_map = msg

    def check_collision_avoidance(self, robot_name, scan):
        """Check if robot is performing collision avoidance"""
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        
        # Check front sector for obstacles
        front_ranges = ranges[len(ranges)//3:2*len(ranges)//3]
        min_front_dist = np.min(front_ranges)
        
        if min_front_dist < 0.5:  # Close obstacle detected
            self.stats['collision_avoidances'] += 1
            self.get_logger().info(f'{robot_name}: Collision avoidance triggered (dist: {min_front_dist:.2f}m)')

    def analyze_map_changes(self, old_map, new_map):
        """Analyze changes between map updates"""
        if (old_map.info.width != new_map.info.width or 
            old_map.info.height != new_map.info.height):
            return
        
        old_data = np.array(old_map.data)
        new_data = np.array(new_map.data)
        
        changes = np.sum(old_data != new_data)
        if changes > 10:  # Significant changes
            self.stats['dynamic_detections'] += 1
            self.get_logger().info(f'Significant map changes detected: {changes} cells')

    def run_test_phase(self):
        """Execute different test phases"""
        elapsed_total = time.time() - self.test_start_time
        elapsed_phase = time.time() - self.phase_start_time
        
        if elapsed_total > self.test_duration:
            self.finish_test()
            return
        
        # Phase 0: Stationary robots (30s) - test static mapping
        if self.current_phase == 0:
            if elapsed_phase < 30:
                self.stationary_phase()
            else:
                self.next_phase("Static mapping test completed")
        
        # Phase 1: Single robot movement (60s) - test basic dynamic detection
        elif self.current_phase == 1:
            if elapsed_phase < 60:
                self.single_robot_movement_phase()
            else:
                self.next_phase("Single robot movement test completed")
        
        # Phase 2: Both robots moving (90s) - test multi-robot dynamic mapping
        elif self.current_phase == 2:
            if elapsed_phase < 90:
                self.multi_robot_movement_phase()
            else:
                self.next_phase("Multi-robot movement test completed")
        
        # Phase 3: Coordinated exploration (60s) - test coordination
        elif self.current_phase == 3:
            if elapsed_phase < 60:
                self.coordinated_exploration_phase()
            else:
                self.next_phase("Coordinated exploration test completed")
        
        # Phase 4: Stress test (60s) - rapid movements
        elif self.current_phase == 4:
            if elapsed_phase < 60:
                self.stress_test_phase()
            else:
                self.next_phase("Stress test completed")
        
        # Log statistics every 10 seconds
        if int(elapsed_total) % 10 == 0 and int(elapsed_total) != int(elapsed_total - 1):
            self.log_statistics()

    def stationary_phase(self):
        """Phase 0: Keep robots stationary to test static mapping"""
        cmd = Twist()
        self.robot1_cmd_pub.publish(cmd)
        self.robot2_cmd_pub.publish(cmd)

    def single_robot_movement_phase(self):
        """Phase 1: Move only robot1 in a pattern"""
        elapsed = time.time() - self.phase_start_time
        
        # Robot1: Circle pattern
        cmd1 = Twist()
        cmd1.linear.x = 0.2
        cmd1.angular.z = 0.3 * math.sin(elapsed * 0.5)
        self.robot1_cmd_pub.publish(cmd1)
        
        # Robot2: Stationary
        cmd2 = Twist()
        self.robot2_cmd_pub.publish(cmd2)

    def multi_robot_movement_phase(self):
        """Phase 2: Move both robots in different patterns"""
        elapsed = time.time() - self.phase_start_time
        
        # Robot1: Figure-8 pattern
        cmd1 = Twist()
        cmd1.linear.x = 0.2
        cmd1.angular.z = 0.4 * math.sin(elapsed * 0.3)
        self.robot1_cmd_pub.publish(cmd1)
        
        # Robot2: Square pattern (alternating movements)
        cmd2 = Twist()
        phase = int(elapsed / 10) % 4
        if phase == 0:  # Forward
            cmd2.linear.x = 0.15
        elif phase == 1:  # Turn left
            cmd2.angular.z = 0.5
        elif phase == 2:  # Forward
            cmd2.linear.x = 0.15
        else:  # Turn left
            cmd2.angular.z = 0.5
        
        self.robot2_cmd_pub.publish(cmd2)

    def coordinated_exploration_phase(self):
        """Phase 3: Send robots to different goal positions"""
        elapsed = time.time() - self.phase_start_time
        
        # Send new goals every 20 seconds
        if int(elapsed) % 20 == 0 and int(elapsed) != int(elapsed - 1):
            # Robot1 goals
            goal1 = PoseStamped()
            goal1.header.frame_id = 'map'
            goal1.header.stamp = self.get_clock().now().to_msg()
            goal1.pose.position.x = 2.0 * math.cos(elapsed * 0.1)
            goal1.pose.position.y = 2.0 * math.sin(elapsed * 0.1)
            goal1.pose.orientation.w = 1.0
            self.robot1_goal_pub.publish(goal1)
            
            # Robot2 goals (opposite direction)
            goal2 = PoseStamped()
            goal2.header.frame_id = 'map'
            goal2.header.stamp = self.get_clock().now().to_msg()
            goal2.pose.position.x = -2.0 * math.cos(elapsed * 0.1)
            goal2.pose.position.y = -2.0 * math.sin(elapsed * 0.1)
            goal2.pose.orientation.w = 1.0
            self.robot2_goal_pub.publish(goal2)

    def stress_test_phase(self):
        """Phase 4: Rapid movements to stress test the system"""
        elapsed = time.time() - self.phase_start_time
        
        # Robot1: Rapid direction changes
        cmd1 = Twist()
        cmd1.linear.x = 0.3 * (1 if int(elapsed * 2) % 2 == 0 else -1)
        cmd1.angular.z = 1.0 * math.sin(elapsed * 3)
        self.robot1_cmd_pub.publish(cmd1)
        
        # Robot2: Rapid spinning with forward motion
        cmd2 = Twist()
        cmd2.linear.x = 0.2
        cmd2.angular.z = 2.0 * math.sin(elapsed * 4)
        self.robot2_cmd_pub.publish(cmd2)

    def next_phase(self, message):
        """Move to the next test phase"""
        self.get_logger().info(f"Phase {self.current_phase}: {message}")
        self.current_phase += 1
        self.phase_start_time = time.time()
        self.get_logger().info(f"Starting Phase {self.current_phase}")
        
        # Stop robots between phases
        cmd = Twist()
        self.robot1_cmd_pub.publish(cmd)
        self.robot2_cmd_pub.publish(cmd)
        time.sleep(2)  # Brief pause

    def log_statistics(self):
        """Log current test statistics"""
        elapsed = time.time() - self.test_start_time
        self.get_logger().info(f"=== Test Statistics (Phase {self.current_phase}, {elapsed:.0f}s) ===")
        self.get_logger().info(f"Robot1 scans: {self.stats['robot1_scans']}")
        self.get_logger().info(f"Robot2 scans: {self.stats['robot2_scans']}")
        self.get_logger().info(f"Map updates: {self.stats['map_updates']}")
        self.get_logger().info(f"Dynamic detections: {self.stats['dynamic_detections']}")
        self.get_logger().info(f"Collision avoidances: {self.stats['collision_avoidances']}")
        
        # Calculate rates
        if elapsed > 0:
            scan_rate = (self.stats['robot1_scans'] + self.stats['robot2_scans']) / elapsed
            map_rate = self.stats['map_updates'] / elapsed
            self.get_logger().info(f"Combined scan rate: {scan_rate:.1f} Hz")
            self.get_logger().info(f"Map update rate: {map_rate:.1f} Hz")

    def finish_test(self):
        """Finish the test and generate report"""
        # Stop all robots
        cmd = Twist()
        self.robot1_cmd_pub.publish(cmd)
        self.robot2_cmd_pub.publish(cmd)
        
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("DYNAMIC MAPPING TEST COMPLETED")
        self.get_logger().info("="*50)
        
        elapsed = time.time() - self.test_start_time
        self.get_logger().info(f"Total test duration: {elapsed:.1f}s")
        
        # Final statistics
        self.log_statistics()
        
        # Performance analysis
        self.analyze_performance()
        
        # Shutdown
        self.get_logger().info("Shutting down test suite...")
        rclpy.shutdown()

    def analyze_performance(self):
        """Analyze overall test performance"""
        self.get_logger().info("\n=== Performance Analysis ===")
        
        elapsed = time.time() - self.test_start_time
        total_scans = self.stats['robot1_scans'] + self.stats['robot2_scans']
        
        if total_scans > 0:
            avg_scan_rate = total_scans / elapsed
            expected_rate = 20.0  # Expected ~10Hz per robot
            performance = (avg_scan_rate / expected_rate) * 100
            
            self.get_logger().info(f"Scan performance: {performance:.1f}% of expected")
            
            if performance > 80:
                self.get_logger().info("✓ Excellent scan performance")
            elif performance > 60:
                self.get_logger().info("⚠ Good scan performance")
            else:
                self.get_logger().info("✗ Poor scan performance - check connectivity")
        
        # Map update analysis
        if self.stats['map_updates'] > 0:
            map_rate = self.stats['map_updates'] / elapsed
            if map_rate > 3:
                self.get_logger().info("✓ Active map updates detected")
            else:
                self.get_logger().info("⚠ Low map update rate - check dynamic detection")
        
        # Dynamic detection analysis
        if self.stats['dynamic_detections'] > 5:
            self.get_logger().info("✓ Dynamic environment detection working")
        elif self.stats['dynamic_detections'] > 0:
            self.get_logger().info("⚠ Limited dynamic detection - check parameters")
        else:
            self.get_logger().info("✗ No dynamic detection - check implementation")
        
        # Collision avoidance analysis
        if self.stats['collision_avoidances'] > 0:
            self.get_logger().info("✓ Collision avoidance system active")
        else:
            self.get_logger().info("⚠ No collision avoidance triggered - check obstacle detection")

def main(args=None):
    rclpy.init(args=args)
    
    tester = DynamicMappingTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
        tester.finish_test()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()