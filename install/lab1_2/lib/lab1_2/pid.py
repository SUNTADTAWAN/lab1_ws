#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_path_tracker')
        
        # PID gains
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        
        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.2
        
        # Error accumulators
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        
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
    
    def control_loop(self):
        if self.target_index >= len(self.path):
            self.get_logger().info('Path tracking completed.')
            return
        
        target = self.path[self.target_index]
        target_x, target_y = target['x'], target['y']
        
        # Calculate distance and angle error
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_error = np.sqrt(dx**2 + dy**2)
        target_yaw = np.arctan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        
        # PID for linear velocity
        linear_p = self.kp_linear * distance_error
        linear_d = self.kd_linear * (distance_error - self.prev_linear_error)
        linear_speed = linear_p + linear_d
        
        # PID for angular velocity
        angular_p = self.kp_angular * yaw_error
        angular_d = self.kd_angular * (yaw_error - self.prev_angular_error)
        angular_speed = angular_p + angular_d
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = max(min(linear_speed, 0.5), -0.5)  # Limit speed
        cmd.angular.z = max(min(angular_speed, 1.0), -1.0)
        
        self.cmd_vel_publisher.publish(cmd)
        
        # Update errors
        self.prev_linear_error = distance_error
        self.prev_angular_error = yaw_error
        
        # Check if target reached
        if distance_error < 0.1:
            self.target_index += 1

        self.get_logger().info(f'Target: {target_x}, {target_y} | Pos: {self.current_x}, {self.current_y} | Cmd: {cmd.linear.x}, {cmd.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
