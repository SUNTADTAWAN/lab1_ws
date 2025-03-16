#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
import tf_transformations

class NoisyGPSNode(Node):
    def __init__(self):
        super().__init__('noisy_gps_node')
        
        # Subscription to ground truth odometry
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        
        # Publisher to GPS topic (simulated noisy odometry)
        self.gps_pub = self.create_publisher(Odometry, '/gps', 10)
        
        # Set noise parameters (adjust as needed)
        self.position_noise_std_dev = 0.05  # Standard deviation of position noise (meters)
        self.yaw_noise_std_dev = 0.02  # Standard deviation of yaw noise (radians)

    def odom_callback(self, msg):
        # Extract true position
        true_x = msg.pose.pose.position.x
        true_y = msg.pose.pose.position.y

        # Add Gaussian noise to x, y
        noisy_x = true_x + np.random.normal(0, self.position_noise_std_dev)
        noisy_y = true_y + np.random.normal(0, self.position_noise_std_dev)
        
        # Extract yaw from quaternion
        quaternion = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        true_yaw = euler[2]  # Yaw angle (rotation around Z-axis)

        # Add noise to yaw
        noisy_yaw = true_yaw + np.random.normal(0, self.yaw_noise_std_dev)

        # Create new noisy odometry message
        noisy_odom = Odometry()
        noisy_odom.header.stamp = self.get_clock().now().to_msg()
        noisy_odom.header.frame_id = 'map'  # Set frame_id based on usage
        noisy_odom.pose.pose.position.x = noisy_x
        noisy_odom.pose.pose.position.y = noisy_y
        
        # Convert noisy yaw back to quaternion
        noisy_quat = tf_transformations.quaternion_from_euler(0, 0, noisy_yaw)
        noisy_odom.pose.pose.orientation.x = noisy_quat[0]
        noisy_odom.pose.pose.orientation.y = noisy_quat[1]
        noisy_odom.pose.pose.orientation.z = noisy_quat[2]
        noisy_odom.pose.pose.orientation.w = noisy_quat[3]

        # Publish noisy odometry
        self.gps_pub.publish(noisy_odom)
        self.get_logger().info(f"Published noisy GPS Odometry: x={noisy_x}, y={noisy_y}, yaw={noisy_yaw} rad")

def main(args=None):
    rclpy.init(args=args)
    node = NoisyGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
