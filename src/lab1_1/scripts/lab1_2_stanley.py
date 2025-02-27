#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import os
from ament_index_python.packages import get_package_share_directory

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # Load path.yaml
        package_name = 'lab1_1'
        path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')
        if not os.path.exists(path_file):
            self.get_logger().error(f"Path file not found: {path_file}")
            return

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        if not self.path:
            self.get_logger().error("Path file is empty or incorrectly formatted.")
            return

        # Convert path to list of waypoints (x, y, yaw)
        self.waypoints = [(wp['x'], wp['y'], wp['yaw']) for wp in self.path]

        # Initialize pose and waypoint index
        self.current_pose = None
        self.current_index = 0

        # Stanley Parameters
        self.stanley_k = 0.8         # 🔥 Increased gain for better turns
        self.wheel_base = 0.2       # Distance between front and rear axles
        self.min_speed = 0.2         # Minimum speed (m/s)
        self.max_speed = 1.2         # Maximum speed (m/s)
        self.forward_speed = 1.0     # Base speed

        # Adaptive steering threshold
        self.adaptive_k_factor = 1.2 # Adaptive gain based on speed

        # Subscribers & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """ Reads odometry and updates current pose (x, y, yaw) """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.current_pose = (x, y, yaw)

    def normalize_angle(self, angle):
        """ Normalize angle to [-π, π] """
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def find_closest_waypoint(self):
        """ Find the closest waypoint to the robot's current position """
        if self.current_pose is None:
            return None, None

        robot_x, robot_y, _ = self.current_pose
        closest_dist = float('inf')
        closest_index = 0

        for i, (px, py, pyaw) in enumerate(self.waypoints):
            dist = math.hypot(px - robot_x, py - robot_y)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        return closest_index, closest_dist

    def control_loop(self):
        """ Stanley Control Loop """
        if self.current_pose is None:
            return

        # Check if waypoints are exhausted
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ All waypoints reached. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        robot_x, robot_y, robot_yaw = self.current_pose

        # Find the closest waypoint
        closest_index, closest_dist = self.find_closest_waypoint()
        if closest_index is None:
            self.get_logger().info("⚠️ No closest waypoint found. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # If the robot is close enough to the waypoint, move to the next one
        if closest_dist < 0.15 and closest_index >= self.current_index:
            self.current_index = min(closest_index + 1, len(self.waypoints) - 1)
            self.get_logger().info(f"📍 Reached waypoint #{closest_index}, moving to next.")

        # Use the reference waypoint
        target_x, target_y, target_yaw = self.waypoints[self.current_index]

        # 1) Compute Heading Error
        heading_error = target_yaw - robot_yaw
        heading_error = self.normalize_angle(heading_error)

        # 2) Compute Cross Track Error (cte)
        path_vec_x = math.cos(target_yaw)
        path_vec_y = math.sin(target_yaw)
        dx = robot_x - target_x
        dy = robot_y - target_y
        cross_product = dx * path_vec_y - dy * path_vec_x  # Cross track error
        cte = cross_product

        # 🔥 **Adaptive Stanley Gain for Turns**
        speed = self.forward_speed
        adaptive_k = self.stanley_k * (1.0 + self.adaptive_k_factor / (speed + 0.1))  # Avoid div by zero

        # **Use atan2 for better CTE handling**
        cte_term = math.atan2(adaptive_k * cte, speed)

        # 3) Apply Stanley Control Law
        steer_angle = 1.2 * heading_error + cte_term  # 🔥 Increase yaw contribution

        # 🔥 **Increase Steering Sensitivity**
        max_steer = 1.57  # 🔥 More aggressive turns (57°)
        steer_angle = max(min(steer_angle, max_steer), -max_steer)

        # 4) Compute Angular Velocity (ω)
        omega = speed * math.tan(steer_angle) / self.wheel_base

        # 🔥 **Dynamically Adjust Speed in Sharp Turns**
        if abs(steer_angle) > 0.5:
            speed *= 0.4  # Reduce speed significantly for sharper turns
        elif abs(steer_angle) > 0.3:
            speed *= 0.7  # Reduce speed moderately for medium turns

        # Debug log
        self.get_logger().info(f"[Stanley] Index={self.current_index}, CTE={cte:.3f}, HeadingErr={heading_error:.3f}, Steer={steer_angle:.3f}, Omega={omega:.3f}")

        # Publish control command
        self.publish_cmd_vel(speed, omega)

    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

