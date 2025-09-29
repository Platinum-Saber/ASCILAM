import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import numpy as np
import math
from collections import deque
import time

class MultiRobotSLAM(Node):
    def __init__(self):
        super().__init__('multi_robot_slam')
        
        # Parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 2000)
        self.declare_parameter('map_height', 2000)
        self.declare_parameter('update_rate', 5.0)
        
        # Dynamic environment parameters
        self.declare_parameter('decay_rate', 0.95)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('temporal_window', 30.0)
        self.declare_parameter('dynamic_threshold', 0.3)
        self.declare_parameter('occupied_threshold', 0.7)
        self.declare_parameter('free_threshold', 0.3)
        self.declare_parameter('prior_probability', 0.5)
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.min_observations = self.get_parameter('min_observations').value
        self.temporal_window = self.get_parameter('temporal_window').value
        self.dynamic_threshold = self.get_parameter('dynamic_threshold').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.prior_probability = self.get_parameter('prior_probability').value
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Subscribers
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Map data - probabilistic mapping
        self.occupancy_prob = np.full((self.map_height, self.map_width), self.prior_probability, dtype=np.float32)
        self.observation_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.last_observation_time = np.zeros((self.map_height, self.map_width), dtype=np.float64)
        self.observation_history = {}  # Store recent observations for dynamic detection
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2
        
        # Update timer
        self.update_timer = self.create_timer(1.0/self.update_rate, self.publish_map)
        
        # Temporal decay timer for dynamic environments
        self.decay_timer = self.create_timer(1.0, self.apply_temporal_decay)
        
        self.get_logger().info('Multi-Robot Dynamic SLAM initialized')

    def robot1_scan_callback(self, msg):
        self.process_scan(msg, 'robot1')

    def robot2_scan_callback(self, msg):
        self.process_scan(msg, 'robot2')

    def process_scan(self, scan_msg, robot_name):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', f'{robot_name}/base_link', rclpy.time.Time())
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.update_map_with_scan(scan_msg, robot_x, robot_y, robot_yaw)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to process scan from {robot_name}: {e}')

    def update_map_with_scan(self, scan, robot_x, robot_y, robot_yaw):
        current_time = time.time()
        
        for i, range_val in enumerate(scan.ranges):
            if not (scan.range_min <= range_val <= scan.range_max):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment + robot_yaw
            
            end_x = robot_x + range_val * math.cos(angle)
            end_y = robot_y + range_val * math.sin(angle)
            
            # Probabilistic ray tracing
            self.probabilistic_ray_trace(robot_x, robot_y, end_x, end_y, current_time)
            self.update_obstacle_probability(end_x, end_y, current_time, True)

    def probabilistic_ray_trace(self, x0, y0, x1, y1, timestamp):
        """Update free space probabilities along ray"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        steps = max(int(dx / self.map_resolution), int(dy / self.map_resolution))
        
        if steps == 0:
            return
        
        x_step = (x1 - x0) / steps
        y_step = (y1 - y0) / steps
        
        for i in range(steps - 1):  # Don't mark endpoint as free
            x = x0 + i * x_step
            y = y0 + i * y_step
            
            self.update_obstacle_probability(x, y, timestamp, False)

    def update_obstacle_probability(self, x, y, timestamp, is_obstacle):
        """Update cell probability using Bayesian updates"""
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
            return
        
        # Store observation in history for dynamic detection
        cell_key = (grid_x, grid_y)
        if cell_key not in self.observation_history:
            self.observation_history[cell_key] = deque()
        
        # Add new observation
        self.observation_history[cell_key].append((timestamp, is_obstacle))
        
        # Remove old observations outside temporal window
        while (self.observation_history[cell_key] and 
               timestamp - self.observation_history[cell_key][0][0] > self.temporal_window):
            self.observation_history[cell_key].popleft()
        
        # Bayesian update
        if is_obstacle:
            # P(occupied | sensor reading) = 0.9
            sensor_prob = 0.9
        else:
            # P(free | sensor reading) = 0.1
            sensor_prob = 0.1
        
        prior = self.occupancy_prob[grid_y, grid_x]
        
        # Bayesian update formula
        posterior = (sensor_prob * prior) / (
            sensor_prob * prior + (1 - sensor_prob) * (1 - prior)
        )
        
        self.occupancy_prob[grid_y, grid_x] = posterior
        self.observation_count[grid_y, grid_x] += 1
        self.last_observation_time[grid_y, grid_x] = timestamp

    def apply_temporal_decay(self):
        """Apply temporal decay to reduce confidence in old observations"""
        current_time = time.time()
        
        # Apply decay to cells not recently observed (older than 5 seconds)
        mask = (current_time - self.last_observation_time) > 5.0
        
        # Decay towards unknown (prior probability)
        self.occupancy_prob[mask] = (
            self.occupancy_prob[mask] * self.decay_rate + 
            self.prior_probability * (1 - self.decay_rate)
        )

    def detect_dynamic_objects(self):
        """Detect cells that change frequently (dynamic objects)"""
        dynamic_mask = np.zeros((self.map_height, self.map_width), dtype=bool)
        
        for (grid_x, grid_y), observations in self.observation_history.items():
            if len(observations) < self.min_observations:
                continue
            
            # Calculate variance in observations
            obs_values = [obs[1] for obs in observations]
            variance = np.var(obs_values)
            
            if variance > self.dynamic_threshold:
                dynamic_mask[grid_y, grid_x] = True
        
        return dynamic_mask

    def publish_map(self):
        """Convert probabilistic map to occupancy grid"""
        # Convert probabilities to occupancy values
        occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Mark as occupied if probability > occupied_threshold
        occupied_mask = self.occupancy_prob > self.occupied_threshold
        occupancy_grid[occupied_mask] = 100
        
        # Mark as free if probability < free_threshold and observed enough times
        free_mask = (self.occupancy_prob < self.free_threshold) & (self.observation_count > 2)
        occupancy_grid[free_mask] = 0
        
        # Create and publish map message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
        # Log dynamic detection statistics
        dynamic_cells = self.detect_dynamic_objects()
        if np.any(dynamic_cells):
            self.get_logger().info(f'Detected {np.sum(dynamic_cells)} dynamic cells')

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()