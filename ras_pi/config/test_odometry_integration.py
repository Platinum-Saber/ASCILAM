#!/usr/bin/env python3
"""
Odometry Integration Test Script
Verifies that the SLAM system is properly receiving and using odometry data
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

class OdometryTester(Node):
    def __init__(self):
        super().__init__('odometry_tester')
        
        # Subscribers
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        
        # Data tracking
        self.robot1_data = {'odom': None, 'scan': None, 'last_odom': 0, 'last_scan': 0}
        self.robot2_data = {'odom': None, 'scan': None, 'last_odom': 0, 'last_scan': 0}
        
        # Status timer
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Odometry Integration Tester started')

    def robot1_odom_callback(self, msg):
        pose = msg.pose.pose
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot1_data['odom'] = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
            'yaw_deg': math.degrees(yaw)
        }
        self.robot1_data['last_odom'] = time.time()

    def robot2_odom_callback(self, msg):
        pose = msg.pose.pose
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot2_data['odom'] = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
            'yaw_deg': math.degrees(yaw)
        }
        self.robot2_data['last_odom'] = time.time()

    def robot1_scan_callback(self, msg):
        self.robot1_data['scan'] = {
            'ranges_count': len(msg.ranges),
            'angle_min': math.degrees(msg.angle_min),
            'angle_max': math.degrees(msg.angle_max),
            'min_range': msg.range_min,
            'max_range': msg.range_max
        }
        self.robot1_data['last_scan'] = time.time()

    def robot2_scan_callback(self, msg):
        self.robot2_data['scan'] = {
            'ranges_count': len(msg.ranges),
            'angle_min': math.degrees(msg.angle_min),
            'angle_max': math.degrees(msg.angle_max),
            'min_range': msg.range_min,
            'max_range': msg.range_max
        }
        self.robot2_data['last_scan'] = time.time()

    def print_status(self):
        current_time = time.time()
        
        print("\n" + "="*80)
        print("ODOMETRY INTEGRATION TEST STATUS")
        print("="*80)
        
        # Robot 1 Status
        print(f"\nROBOT 1:")
        if self.robot1_data['odom']:
            odom_age = current_time - self.robot1_data['last_odom']
            print(f"  Odometry: ({self.robot1_data['odom']['x']:.3f}, {self.robot1_data['odom']['y']:.3f}, {self.robot1_data['odom']['yaw_deg']:.1f}°) - Age: {odom_age:.1f}s")
            if odom_age > 2.0:
                print("  ⚠️  WARNING: Odometry data is stale!")
        else:
            print("  ❌ NO ODOMETRY DATA")
            
        if self.robot1_data['scan']:
            scan_age = current_time - self.robot1_data['last_scan']
            print(f"  LiDAR: {self.robot1_data['scan']['ranges_count']} points, {self.robot1_data['scan']['angle_min']:.0f}° to {self.robot1_data['scan']['angle_max']:.0f}° - Age: {scan_age:.1f}s")
        else:
            print("  ❌ NO SCAN DATA")
        
        # Robot 2 Status
        print(f"\nROBOT 2:")
        if self.robot2_data['odom']:
            odom_age = current_time - self.robot2_data['last_odom']
            print(f"  Odometry: ({self.robot2_data['odom']['x']:.3f}, {self.robot2_data['odom']['y']:.3f}, {self.robot2_data['odom']['yaw_deg']:.1f}°) - Age: {odom_age:.1f}s")
            if odom_age > 2.0:
                print("  ⚠️  WARNING: Odometry data is stale!")
        else:
            print("  ❌ NO ODOMETRY DATA")
            
        if self.robot2_data['scan']:
            scan_age = current_time - self.robot2_data['last_scan']
            print(f"  LiDAR: {self.robot2_data['scan']['ranges_count']} points, {self.robot2_data['scan']['angle_min']:.0f}° to {self.robot2_data['scan']['angle_max']:.0f}° - Age: {scan_age:.1f}s")
        else:
            print("  ❌ NO SCAN DATA")
            
        # Integration Status
        print(f"\nINTEGRATION STATUS:")
        both_odom = self.robot1_data['odom'] and self.robot2_data['odom']
        both_scan = self.robot1_data['scan'] and self.robot2_data['scan']
        
        if both_odom and both_scan:
            print("  ✅ SLAM ready - Both robots publishing odometry and scan data")
        elif both_odom:
            print("  ⚠️  Odometry ready but waiting for scan data")
        elif both_scan:
            print("  ⚠️  Scan data ready but waiting for odometry")
        else:
            print("  ❌ Waiting for robot data...")
        
        print("="*80)

def main(args=None):
    rclpy.init(args=args)
    
    tester = OdometryTester()
    
    try:
        print("\nOdometry Integration Test")
        print("Press Ctrl+C to stop")
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()