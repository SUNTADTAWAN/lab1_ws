#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class BicycleModelKinematics(Node):
    def __init__(self):
        super().__init__('bicycle_model_kinematics')

        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.045)  # Radius of the wheels (meters, 4.5 cm)
        self.declare_parameter('wheel_base', 0.065)    # Distance between front and rear wheels (meters, 6.5 cm)
        self.declare_parameter('robot_height', 0.1)   # Height of the robot (meters, 10 cm)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # Subscription to cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publisher for front and rear wheel velocities
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)

    def cmd_vel_callback(self, msg):
        """
        Callback for /cmd_vel topic.
        Converts linear and angular velocities into front and rear wheel speeds using the Bicycle Model.
        """
        linear_velocity = msg.linear.x  # Linear velocity (m/s)
        angular_velocity = msg.angular.z  # Angular velocity (rad/s)

        # Compute the steering angle for the rear wheel
        steer_angle = math.atan2(self.wheel_base * angular_velocity, linear_velocity) if linear_velocity != 0 else 0.0

        # Calculate the angular velocity of the front wheel (front-wheel drive)
        front_wheel_angular_velocity = linear_velocity / self.wheel_radius

        # Prepare and publish wheel speed command
        wheel_speeds = Float64MultiArray()
        wheel_speeds.data = [front_wheel_angular_velocity, steer_angle]
        self.pub_wheel_spd.publish(wheel_speeds)

        self.get_logger().info(
            f"Front Wheel Speed: {front_wheel_angular_velocity:.2f} rad/s, Steer Angle: {steer_angle:.2f} rad"
        )

    def calculate_forward_kine(self, front_wheel_velocity, steer_angle):
        """
        Calculate linear and angular velocities of the robot using forward kinematics.
        """
        linear_velocity = front_wheel_velocity * self.wheel_radius  # Linear velocity (m/s)
        angular_velocity = (linear_velocity * math.tan(steer_angle)) / self.wheel_base  # Angular velocity (rad/s)

        return linear_velocity, angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
