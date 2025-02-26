#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_path_tracker')
        
        # Parameters
        self.lookahead_distance = 1.0  # Lookahead distance
        self.max_linear_speed = 0.5  # Max speed
        self.max_angular_speed = 1.0  # Max angular speed
        
        # Load path from YAML file
        self.path = self.load_yaml_path('/home/tadtawan/mobile_robot/lab1_ws/src/lab1_2/config/path.yaml')
        self.target_index = 0
        
        # Publishers & Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
    def load_yaml_path(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    
    def find_target_index(self):
        for i in range(self.target_index, len(self.path)):
            dx = self.path[i]['x'] - self.current_x
            dy = self.path[i]['y'] - self.current_y
            dist = np.sqrt(dx**2 + dy**2)
            if dist > self.lookahead_distance:
                return i
        return len(self.path) - 1
    
    def control_loop(self):
        if self.target_index >= len(self.path):
            self.get_logger().info('Path tracking completed.')
            return
        
        self.target_index = self.find_target_index()
        target = self.path[self.target_index]
        target_x, target_y = target['x'], target['y']
        
        # Calculate lookahead vector
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        target_yaw = np.arctan2(dy, dx)
        
        # Compute curvature
        alpha = target_yaw - self.current_yaw
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalize angle
        curvature = 2 * np.sin(alpha) / self.lookahead_distance
        
        # Compute control commands
        linear_speed = self.max_linear_speed
        angular_speed = curvature * linear_speed
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
        
        self.cmd_vel_publisher.publish(cmd)
        
        self.get_logger().info(f'Target: {target_x}, {target_y} | Pos: {self.current_x}, {self.current_y} | Cmd: {cmd.linear.x}, {cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()