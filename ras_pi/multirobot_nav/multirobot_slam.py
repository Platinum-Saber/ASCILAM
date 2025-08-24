import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import numpy as np
import math

class MultiRobotSLAM(Node):
    def __init__(self):
        super().__init__('multi_robot_slam')
        
        # Parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 2000)
        self.declare_parameter('map_height', 2000)
        self.declare_parameter('update_rate', 5.0)
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        
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
        
        # Map data
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2
        
        # Update timer
        self.update_timer = self.create_timer(1.0/self.update_rate, self.publish_map)
        
        self.get_logger().info('Multi-Robot SLAM initialized')

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
        for i, range_val in enumerate(scan.ranges):
            if not (scan.range_min <= range_val <= scan.range_max):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment + robot_yaw
            
            end_x = robot_x + range_val * math.cos(angle)
            end_y = robot_y + range_val * math.sin(angle)
            
            self.ray_trace(robot_x, robot_y, end_x, end_y)
            self.mark_obstacle(end_x, end_y)

    def ray_trace(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        steps = max(int(dx / self.map_resolution), int(dy / self.map_resolution))
        
        if steps == 0:
            return
        
        x_step = (x1 - x0) / steps
        y_step = (y1 - y0) / steps
        
        for i in range(steps):
            x = x0 + i * x_step
            y = y0 + i * y_step
            
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                if self.occupancy_grid[grid_y, grid_x] == -1:
                    self.occupancy_grid[grid_y, grid_x] = 0

    def mark_obstacle(self, x, y):
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            self.occupancy_grid[grid_y, grid_x] = 100

    def publish_map(self):
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
        
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()