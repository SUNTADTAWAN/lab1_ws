import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class AckermannKinematicsNode(Node):
    def __init__(self):
        super().__init__('ackermann_kinematics_node')

        # Robot parameters
        self.wheel_radius = 0.045  # meters (4.5 cm)
        self.wheel_base = 0.35     # meters (distance between front and rear axles)
        self.track_width = 0.2     # meters (distance between left and right wheels)

        # Subscriber for cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Input velocity topic
            self.cmd_vel_callback,
            10
        )

        # Publisher for wheel velocities (rear wheels)
        self.wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controllers/commands',  # Output topic
            10
        )

        # Publisher for steering angles (front wheels)
        self.steering_angle_publisher = self.create_publisher(
            Float64MultiArray,
            '/steering_controllers/commands',  # Steering angles topic
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function to compute Ackermann steering and velocity
        """
        linear_velocity = msg.linear.x  # Forward velocity (m/s)
        angular_velocity = msg.angular.z  # Yaw rate (rad/s)

        if angular_velocity == 0.0:
            # Moving straight, no steering
            front_left_angle = 0.0
            front_right_angle = 0.0
            rear_left_velocity = linear_velocity
            rear_right_velocity = linear_velocity
        else:
            # Compute turning radius
            turning_radius = linear_velocity / angular_velocity

            # Compute steering angles for front wheels
            front_left_angle = math.atan(self.wheel_base / (turning_radius - (self.track_width / 2)))
            front_right_angle = math.atan(self.wheel_base / (turning_radius + (self.track_width / 2)))

            # Compute rear wheel velocities (constant for both rear wheels)
            rear_left_velocity = linear_velocity
            rear_right_velocity = linear_velocity

        # Publish rear wheel velocities
        self.publish_wheel_velocities([rear_left_velocity, rear_right_velocity])

        # Publish front wheel steering angles
        self.publish_steering_angles([front_left_angle, front_right_angle])

    def publish_wheel_velocities(self, velocities):
        """
        Publish the rear wheel velocities
        """
        velocity_msg = Float64MultiArray()
        velocity_msg.data = velocities
        self.wheel_velocity_publisher.publish(velocity_msg)

        self.get_logger().info(f"Rear Wheel Velocities: {velocities}")

    def publish_steering_angles(self, angles):
        """
        Publish the steering angles for the front wheels
        """
        angle_msg = Float64MultiArray()
        angle_msg.data = angles
        self.steering_angle_publisher.publish(angle_msg)

        self.get_logger().info(f"Front Steering Angles: {angles}")


def main(args=None):
    rclpy.init(args=args)

    # Start the Ackermann Kinematics Node
    node = AckermannKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
