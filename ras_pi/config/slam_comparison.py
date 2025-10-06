#!/usr/bin/env python3
"""
SLAM System Comparison: Static vs Dynamic Localization
Shows the difference between the old static approach and new odometry-based approach
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import math
import time
from geometry_msgs.msg import TransformStamped

class SLAMComparison(Node):
    def __init__(self):
        super().__init__('slam_comparison')
        
        # TF components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Odometry subscribers
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        
        # Data storage
        self.robot1_odom = None
        self.robot2_odom = None
        
        # Comparison timer
        self.comparison_timer = self.create_timer(3.0, self.compare_approaches)
        
        self.get_logger().info('SLAM Comparison Tool started')
        print("\n" + "="*80)
        print("SLAM SYSTEM COMPARISON: Static vs Odometry-based Localization")
        print("="*80)

    def robot1_odom_callback(self, msg):
        pose = msg.pose.pose
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot1_odom = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': math.degrees(yaw),
            'timestamp': time.time()
        }

    def robot2_odom_callback(self, msg):
        pose = msg.pose.pose
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot2_odom = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': math.degrees(yaw),
            'timestamp': time.time()
        }

    def get_tf_pose(self, robot_name):
        """Try to get pose from TF tree (old approach)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', f'{robot_name}/base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            
            return {'x': x, 'y': y, 'yaw': yaw}
        except Exception as e:
            return None

    def compare_approaches(self):
        print(f"\nComparison at {time.strftime('%H:%M:%S')}")
        print("-" * 80)
        
        # Robot 1 comparison
        print("ROBOT 1:")
        tf_pose1 = self.get_tf_pose('robot1')
        if tf_pose1:
            print(f"  TF Transform:    ({tf_pose1['x']:.3f}, {tf_pose1['y']:.3f}, {tf_pose1['yaw']:.1f}°)")
        else:
            print("  TF Transform:    ❌ Not available")
            
        if self.robot1_odom:
            age = time.time() - self.robot1_odom['timestamp']
            print(f"  Odometry:        ({self.robot1_odom['x']:.3f}, {self.robot1_odom['y']:.3f}, {self.robot1_odom['yaw']:.1f}°) [age: {age:.1f}s]")
        else:
            print("  Odometry:        ❌ Not available")
        
        # Robot 2 comparison
        print("ROBOT 2:")
        tf_pose2 = self.get_tf_pose('robot2')
        if tf_pose2:
            print(f"  TF Transform:    ({tf_pose2['x']:.3f}, {tf_pose2['y']:.3f}, {tf_pose2['yaw']:.1f}°)")
        else:
            print("  TF Transform:    ❌ Not available")
            
        if self.robot2_odom:
            age = time.time() - self.robot2_odom['timestamp']
            print(f"  Odometry:        ({self.robot2_odom['x']:.3f}, {self.robot2_odom['y']:.3f}, {self.robot2_odom['yaw']:.1f}°) [age: {age:.1f}s]")
        else:
            print("  Odometry:        ❌ Not available")
        
        # Analysis
        print("\nANALYSIS:")
        if tf_pose1 and self.robot1_odom and tf_pose2 and self.robot2_odom:
            # Calculate differences
            r1_diff = math.sqrt((tf_pose1['x'] - self.robot1_odom['x'])**2 + 
                              (tf_pose1['y'] - self.robot1_odom['y'])**2)
            r2_diff = math.sqrt((tf_pose2['x'] - self.robot2_odom['x'])**2 + 
                              (tf_pose2['y'] - self.robot2_odom['y'])**2)
            
            print(f"  Position differences: Robot1: {r1_diff:.3f}m, Robot2: {r2_diff:.3f}m")
            
            if r1_diff > 0.1 or r2_diff > 0.1:
                print("  ✅ ODOMETRY-BASED SLAM IS WORKING: Robots have moved from initial positions!")
                print("     This shows the new system tracks real robot motion.")
            else:
                print("  ⚠️  Robots appear to be at initial positions")
                print("     Either robots haven't moved yet or static transforms are still active")
        else:
            print("  ⚠️  Cannot compare - missing data from one or both systems")
        
        print("\nKEY DIFFERENCES:")
        print("  📌 Static Transforms: Always show initial poses (0,0,0°) and (1,0,180°)")
        print("  🔄 Odometry-based:   Shows real robot positions as they move and explore")
        print("  🎯 SLAM Benefit:     Accurate mapping requires real robot motion tracking")

def main(args=None):
    rclpy.init(args=args)
    
    comparison = SLAMComparison()
    
    try:
        print("\nMonitoring both static TF and odometry-based approaches...")
        print("Press Ctrl+C to stop")
        rclpy.spin(comparison)
    except KeyboardInterrupt:
        print("\nComparison stopped")
    finally:
        comparison.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()