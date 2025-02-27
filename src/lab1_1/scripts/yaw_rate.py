#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
import tf_transformations
import math


class YawRateOdometry(Node):
    def __init__(self):
        super().__init__('yaw_rate_odometry')

        # Declare and get parameters
        self.declare_parameter('track_width', 0.2)  # Distance between left and right wheels (m)
        self.declare_parameter('wheel_radius', 0.045)  # Wheel radius (m)

        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Subscribe to wheel speed topic (expects 2 values: [left_wheel_speed, right_wheel_speed])
        self.create_subscription(Float64MultiArray, "/wheel_speeds", self.wheel_speed_callback, 10)

        # Publish odometry
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        # Initialize robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last timestamp
        self.last_time = self.get_clock().now()

        self.get_logger().info("Yaw Rate Odometry Node Started...")

    def wheel_speed_callback(self, msg):
        """
        Callback function to compute odometry from wheel speeds using Yaw Rate Model.
        """
        if len(msg.data) < 2:
            self.get_logger().error("Invalid wheel speed data! Expected [left, right] speeds.")
            return

        left_wheel_speed = msg.data[0] * self.wheel_radius  # Convert to linear velocity
        right_wheel_speed = msg.data[1] * self.wheel_radius  # Convert to linear velocity

        # Compute linear velocity and yaw rate
        linear_velocity = (right_wheel_speed + left_wheel_speed) / 2.0
        yaw_rate = (right_wheel_speed - left_wheel_speed) / self.track_width

        # Compute time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        self.last_time = current_time

        # Update robot pose
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = yaw_rate * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert yaw to quaternion)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)

        # Velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = yaw_rate

        # Publish odometry message
        self.odom_publisher.publish(odom_msg)

        # Log info
        self.get_logger().info(
            f"Odom -> x: {self.x:.3f}, y: {self.y:.3f}, θ: {self.theta:.3f} rad, v: {linear_velocity:.3f} m/s, ω: {yaw_rate:.3f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YawRateOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
