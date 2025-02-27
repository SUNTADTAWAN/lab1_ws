#!/usr/bin/env python3

###PP
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

class AckermanPurePursuit(Node):
    def __init__(self):
        super().__init__('ackerman_pure_pursuit')

        # Load path.yaml from package
        package_name = 'lab1_1'  # Change this to match your package name
        path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')

        if not os.path.exists(path_file):
            self.get_logger().error(f"Path file not found: {path_file}")
            return

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        if not self.path:
            self.get_logger().error("Path file is empty or incorrectly formatted.")
            return

        self.current_pose = None
        self.target_index = 0

        # Pure Pursuit Parameters
        self.lookahead_distance = 25.0  # Increase to smooth turns
        self.wheel_base = 0.2  # Distance between front and rear wheels

        # PID gains for speed control (optional)
        self.kp_speed = 2.0
        self.max_speed = 2.0
        self.min_speed = 0.5

        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """ Converts Odometry quaternion to yaw angle """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.current_pose = (x, y, yaw)

    def normalize_angle(self, angle):
        """ Normalize angle between -π and π """
        return math.atan2(math.sin(angle), math.cos(angle))

    def find_lookahead_point(self):
        """ Find the point ahead of the current position to follow """
        if self.current_pose is None:
            return None

        x, y, _ = self.current_pose
        closest_index = 0
        min_dist = float('inf')

        # Find the closest point in the path
        for i, point in enumerate(self.path):
            px, py = point['x'], point['y']
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        # Look ahead further in the path
        lookahead_index = min(closest_index + 3, len(self.path) - 1)
        return self.path[lookahead_index]

    def control_loop(self):
        """ Pure Pursuit Path Tracking """
        if self.current_pose is None:
            return

        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            self.get_logger().info("No more target points. Stopping.")
            return

        target_x, target_y = lookahead_point['x'], lookahead_point['y']

        # Compute the angle between the robot's heading and the target point
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        lookahead_dist = math.sqrt(dx**2 + dy**2)

        # Angle to the lookahead point
        alpha = self.normalize_angle(math.atan2(dy, dx) - self.current_pose[2])

        # Compute curvature (γ = 2*sin(α) / Ld)
        if lookahead_dist > 0:
            curvature = 2.0 * math.sin(alpha) / lookahead_dist
        else:
            curvature = 0.0

        # Compute steering angle (δ = atan(γ * L))
        steering_angle = math.atan(curvature * self.wheel_base)

        # Compute speed (optional PID-based speed control)
        speed = self.kp_speed * lookahead_dist
        speed = max(self.min_speed, min(self.max_speed, speed))  # Clamp speed

        # Publish command
        velocity = Twist()
        velocity.linear.x = speed
        velocity.angular.z = steering_angle  # Set angular velocity

        self.cmd_pub.publish(velocity)

        # Check if target is reached
        if lookahead_dist < self.lookahead_distance:
            self.target_index += 1
            self.get_logger().info(f"Reached target {self.target_index}/{len(self.path)}")

def main(args=None):
    rclpy.init(args=args)
    node = AckermanPurePursuit()
    if node.path:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()