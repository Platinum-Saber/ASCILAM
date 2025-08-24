import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import numpy as np

class RobotCoordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')
        
        # Parameters
        self.declare_parameter('coordination_strategy', 'frontier_based')
        self.declare_parameter('min_robot_distance', 1.5)
        self.declare_parameter('exploration_complete_threshold', 0.90)
        
        self.coordination_strategy = self.get_parameter('coordination_strategy').value
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        self.exploration_threshold = self.get_parameter('exploration_complete_threshold').value
        
        # Publishers for robot commands
        self.robot1_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.robot2_cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.robot1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        # Subscribers for robot states
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
        self.exploration_frontiers = []
        self.robot1_goal = None
        self.robot2_goal = None
        
        # Coordination timer
        self.coordination_timer = self.create_timer(2.0, self.coordinate_robots)
        
        # Safety timer for collision avoidance
        self.safety_timer = self.create_timer(0.2, self.safety_check)
        
        self.get_logger().info('Robot Coordinator initialized')

    def robot1_odom_callback(self, msg):
        self.robot1_pose = msg.pose.pose

    def robot2_odom_callback(self, msg):
        self.robot2_pose = msg.pose.pose

    def robot1_scan_callback(self, msg):
        self.robot1_scan = msg

    def robot2_scan_callback(self, msg):
        self.robot2_scan = msg

    def map_callback(self, msg):
        self.current_map = msg
        self.find_exploration_frontiers()

    def find_exploration_frontiers(self):
        if self.current_map is None:
            return
        
        frontiers = []
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Find frontier cells
        for y in range(1, height-1):
            for x in range(1, width-1):
                index = y * width + x
                if self.current_map.data[index] == -1:  # Unknown cell
                    # Check if adjacent to free space
                    adjacent_free = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            adj_index = (y+dy) * width + (x+dx)
                            if 0 <= adj_index < len(self.current_map.data):
                                if self.current_map.data[adj_index] == 0:
                                    adjacent_free = True
                                    break
                        if adjacent_free:
                            break
                    
                    if adjacent_free:
                        world_x = x * resolution + origin_x
                        world_y = y * resolution + origin_y
                        frontiers.append((world_x, world_y))
        
        self.exploration_frontiers = self.cluster_frontiers(frontiers)

    def cluster_frontiers(self, frontiers, cluster_distance=1.0):
        if not frontiers:
            return []
        
        clusters = []
        used = set()
        
        for i, frontier in enumerate(frontiers):
            if i in used:
                continue
            
            cluster = [frontier]
            used.add(i)
            
            for j, other_frontier in enumerate(frontiers):
                if j in used:
                    continue
                
                dist = math.sqrt((frontier[0] - other_frontier[0])**2 + 
                               (frontier[1] - other_frontier[1])**2)
                if dist < cluster_distance:
                    cluster.append(other_frontier)
                    used.add(j)
            
            if len(cluster) >= 3:
                center_x = sum(f[0] for f in cluster) / len(cluster)
                center_y = sum(f[1] for f in cluster) / len(cluster)
                clusters.append((center_x, center_y))
        
        return clusters

    def coordinate_robots(self):
        if not all([self.robot1_pose, self.robot2_pose]):
            return
        
        if self.is_exploration_complete():
            self.stop_robots()
            self.get_logger().info('Exploration complete!')
            return
        
        if self.coordination_strategy == 'frontier_based':
            self.assign_frontier_goals()

    def assign_frontier_goals(self):
        if len(self.exploration_frontiers) < 2:
            return
        
        robot1_pos = (self.robot1_pose.position.x, self.robot1_pose.position.y)
        robot2_pos = (self.robot2_pose.position.x, self.robot2_pose.position.y)
        
        best_assignment = None
        min_cost = float('inf')
        
        for i, frontier1 in enumerate(self.exploration_frontiers):
            for j, frontier2 in enumerate(self.exploration_frontiers):
                if i == j:
                    continue
                
                dist1_to_f1 = math.sqrt((robot1_pos[0] - frontier1[0])**2 + 
                                      (robot1_pos[1] - frontier1[1])**2)
                dist2_to_f2 = math.sqrt((robot2_pos[0] - frontier2[0])**2 + 
                                      (robot2_pos[1] - frontier2[1])**2)
                
                frontier_dist = math.sqrt((frontier1[0] - frontier2[0])**2 + 
                                        (frontier1[1] - frontier2[1])**2)
                
                if frontier_dist < self.min_robot_distance:
                    continue
                
                total_cost = dist1_to_f1 + dist2_to_f2
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_assignment = (frontier1, frontier2)
        
        if best_assignment:
            self.send_goal_to_robot('robot1', best_assignment[0])
            self.send_goal_to_robot('robot2', best_assignment[1])

    def send_goal_to_robot(self, robot_name, goal_pos):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_pos[0]
        goal_msg.pose.position.y = goal_pos[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        if robot_name == 'robot1':
            self.robot1_goal_pub.publish(goal_msg)
            self.robot1_goal = goal_pos
        elif robot_name == 'robot2':
            self.robot2_goal_pub.publish(goal_msg)
            self.robot2_goal = goal_pos
        
        self.get_logger().info(f'Sent goal to {robot_name}: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})')

    def safety_check(self):
        if not all([self.robot1_pose, self.robot2_pose]):
            return
        
        dist = math.sqrt((self.robot1_pose.position.x - self.robot2_pose.position.x)**2 + 
                        (self.robot1_pose.position.y - self.robot2_pose.position.y)**2)
        
        if dist < self.min_robot_distance:
            self.emergency_stop()
            self.get_logger().warn(f'Robots too close ({dist:.2f}m)! Emergency stop.')

    def emergency_stop(self):
        stop_msg = Twist()
        self.robot1_cmd_pub.publish(stop_msg)
        self.robot2_cmd_pub.publish(stop_msg)

    def stop_robots(self):
        stop_msg = Twist()
        self.robot1_cmd_pub.publish(stop_msg)
        self.robot2_cmd_pub.publish(stop_msg)

    def is_exploration_complete(self):
        if self.current_map is None:
            return False
        
        total_cells = len(self.current_map.data)
        known_cells = sum(1 for cell in self.current_map.data if cell != -1)
        exploration_ratio = known_cells / total_cells if total_cells > 0 else 0
        
        return exploration_ratio >= self.exploration_threshold

def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()