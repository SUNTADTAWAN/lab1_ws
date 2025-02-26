#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class BasicModel(Node):
    def __init__(self):
        super().__init__('basic_model_node')

        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.045)  # Wheel radius (m)
        self.declare_parameter('wheel_base', 0.35)  # Distance between front and rear axles (m)
        self.declare_parameter('track_width', 0.2)  # Distance between left and right wheels (m)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value

        # Subscribe to /cmd_vel topic
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publishers for steering and velocity controllers
        self.pub_wheel_velocity = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_steering_angle = self.create_publisher(Float64MultiArray, "/joint_trajectory_position_controller/commands", 10)

    def cmd_vel_callback(self, msg):
        """
        Convert linear.x and angular.z velocities into steering angles and wheel velocities.
        """
        linear_velocity = msg.linear.x  # Forward velocity (m/s)
        angular_velocity = msg.angular.z  # Rotation speed (rad/s)

        if linear_velocity == 0 and angular_velocity == 0:
            # Stop the robot if no command is given
            self.publish_commands(0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # Compute equal steering angle based on the bicycle model
        if angular_velocity != 0:
            steer_angle = math.atan(self.wheel_base * angular_velocity / linear_velocity) if linear_velocity != 0 else 0.0
        else:
            steer_angle = 0.0  # Move straight

        # Compute wheel velocities (same for all wheels)
        wheel_velocity = linear_velocity / self.wheel_radius

        # Publish steering angles and wheel velocities
        self.publish_commands(wheel_velocity, wheel_velocity, wheel_velocity, wheel_velocity, steer_angle)

        self.get_logger().info(
            f"Cmd Vel -> Linear: {linear_velocity:.2f} m/s, Angular: {angular_velocity:.2f} rad/s"
        )
        self.get_logger().info(
            f"Steering Angle -> {steer_angle:.2f} rad"
        )
        self.get_logger().info(
            f"Wheel Velocities -> {wheel_velocity:.2f} rad/s"
        )

    def publish_commands(self, left_front, right_front, left_rear, right_rear, steer_angle):
        """
        Publish wheel velocities and steering angles.
        """
        # Publish equal steering angles to joint_trajectory_position_controller
        steering_msg = Float64MultiArray()
        steering_msg.data = [-steer_angle, -steer_angle]
        self.pub_steering_angle.publish(steering_msg)

        # Publish wheel velocities to velocity_controllers
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [left_front, -right_front, left_rear, -right_rear]
        self.pub_wheel_velocity.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
