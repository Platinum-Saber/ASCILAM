#!/usr/bin/env python3
"""
Dynamic Mapping Parameter Tuning Script
Helps tune dynamic mapping parameters for different environments
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json
import time
from datetime import datetime

class DynamicMappingTuner(Node):
    def __init__(self):
        super().__init__('dynamic_mapping_tuner')
        
        # Current parameter set
        self.current_params = {
            'decay_rate': 0.95,
            'min_observations': 3,
            'temporal_window': 30.0,
            'dynamic_threshold': 0.3,
            'occupied_threshold': 0.7,
            'free_threshold': 0.3
        }
        
        # Performance metrics
        self.metrics = {
            'dynamic_cells_detected': 0,
            'map_updates': 0,
            'false_positives': 0,
            'false_negatives': 0,
            'computational_load': []
        }
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        
        # Publisher for parameter updates
        self.param_pub = self.create_publisher(String, '/dynamic_params', 10)
        
        # Timer for periodic evaluation
        self.eval_timer = self.create_timer(10.0, self.evaluate_performance)
        
        # Data storage
        self.scan_count = {'robot1': 0, 'robot2': 0}
        self.last_map = None
        self.start_time = time.time()
        
        self.get_logger().info('Dynamic Mapping Tuner initialized')
        self.log_current_params()

    def map_callback(self, msg):
        """Process incoming map updates"""
        start_time = time.time()
        
        if self.last_map is not None:
            # Calculate map changes
            current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            last_map = np.array(self.last_map.data).reshape(
                self.last_map.info.height, self.last_map.info.width)
            
            changes = np.sum(current_map != last_map)
            self.metrics['map_updates'] += 1
            
            if changes > 0:
                self.get_logger().info(f'Map changes detected: {changes} cells')
        
        self.last_map = msg
        
        # Record computational load
        processing_time = time.time() - start_time
        self.metrics['computational_load'].append(processing_time)
        
        # Keep only last 100 measurements
        if len(self.metrics['computational_load']) > 100:
            self.metrics['computational_load'] = self.metrics['computational_load'][-100:]

    def robot1_scan_callback(self, msg):
        self.scan_count['robot1'] += 1

    def robot2_scan_callback(self, msg):
        self.scan_count['robot2'] += 1

    def evaluate_performance(self):
        """Evaluate current performance and suggest parameter adjustments"""
        runtime = time.time() - self.start_time
        
        # Calculate performance metrics
        avg_computational_load = np.mean(self.metrics['computational_load']) if self.metrics['computational_load'] else 0
        scan_rate = (self.scan_count['robot1'] + self.scan_count['robot2']) / runtime
        
        self.get_logger().info("=== Performance Evaluation ===")
        self.get_logger().info(f"Runtime: {runtime:.1f}s")
        self.get_logger().info(f"Map updates: {self.metrics['map_updates']}")
        self.get_logger().info(f"Avg computational load: {avg_computational_load:.4f}s")
        self.get_logger().info(f"Combined scan rate: {scan_rate:.1f} Hz")
        self.get_logger().info(f"Robot1 scans: {self.scan_count['robot1']}")
        self.get_logger().info(f"Robot2 scans: {self.scan_count['robot2']}")
        
        # Suggest parameter adjustments based on performance
        suggestions = self.suggest_parameter_adjustments(avg_computational_load, scan_rate)
        
        if suggestions:
            self.get_logger().info("=== Parameter Suggestions ===")
            for suggestion in suggestions:
                self.get_logger().info(suggestion)
        
        # Save metrics to file
        self.save_metrics()

    def suggest_parameter_adjustments(self, computational_load, scan_rate):
        """Suggest parameter adjustments based on performance metrics"""
        suggestions = []
        
        # High computational load - reduce processing
        if computational_load > 0.1:
            suggestions.append("High computational load detected:")
            suggestions.append("- Consider increasing decay_rate (current: {:.3f})".format(
                self.current_params['decay_rate']))
            suggestions.append("- Consider increasing temporal_window (current: {:.1f}s)".format(
                self.current_params['temporal_window']))
        
        # Low scan rate - check system performance
        if scan_rate < 15.0:  # Expected ~20Hz from both robots
            suggestions.append("Low scan rate detected:")
            suggestions.append("- Check robot connectivity and sensor performance")
            suggestions.append("- Consider reducing map update rate")
        
        # Map update frequency analysis
        update_rate = self.metrics['map_updates'] / (time.time() - self.start_time) if (time.time() - self.start_time) > 0 else 0
        if update_rate > 10.0:
            suggestions.append("High map update rate ({:.1f} Hz):".format(update_rate))
            suggestions.append("- Environment might be highly dynamic")
            suggestions.append("- Consider lowering dynamic_threshold (current: {:.2f})".format(
                self.current_params['dynamic_threshold']))
        elif update_rate < 1.0:
            suggestions.append("Low map update rate ({:.1f} Hz):".format(update_rate))
            suggestions.append("- Environment might be mostly static") 
            suggestions.append("- Consider increasing dynamic_threshold (current: {:.2f})".format(
                self.current_params['dynamic_threshold']))
        
        return suggestions

    def save_metrics(self):
        """Save current metrics and parameters to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/tmp/dynamic_mapping_metrics_{timestamp}.json"
        
        data = {
            'timestamp': timestamp,
            'parameters': self.current_params,
            'metrics': {
                'runtime': time.time() - self.start_time,
                'map_updates': self.metrics['map_updates'],
                'scan_counts': self.scan_count,
                'avg_computational_load': np.mean(self.metrics['computational_load']) if self.metrics['computational_load'] else 0
            }
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Metrics saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save metrics: {e}")

    def log_current_params(self):
        """Log current parameter settings"""
        self.get_logger().info("=== Current Dynamic Mapping Parameters ===")
        for param, value in self.current_params.items():
            self.get_logger().info(f"{param}: {value}")

    def update_parameter(self, param_name, new_value):
        """Update a parameter and publish the change"""
        if param_name in self.current_params:
            old_value = self.current_params[param_name]
            self.current_params[param_name] = new_value
            
            self.get_logger().info(f"Parameter updated: {param_name} = {new_value} (was {old_value})")
            
            # Publish parameter update
            msg = String()
            msg.data = json.dumps({param_name: new_value})
            self.param_pub.publish(msg)
        else:
            self.get_logger().error(f"Unknown parameter: {param_name}")

def main(args=None):
    rclpy.init(args=args)
    
    tuner = DynamicMappingTuner()
    
    try:
        rclpy.spin(tuner)
    except KeyboardInterrupt:
        tuner.get_logger().info("Shutting down tuner...")
    finally:
        tuner.save_metrics()
        tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()