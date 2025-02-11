import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class FourWheelInverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('four_wheel_inverse_kinematics_node')

        # Robot parameters
        self.wheel_radius = 0.045  # meter (4.5 cm)
        self.wheel_base = 0.13      # meter (distance between left and right wheels)
        self.track_width = 0.1   # meter (distance between front and rear wheels)

        # Subscriber for cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Input velocity topic
            self.cmd_vel_callback,
            10
        )

        # Publisher for wheel velocities
        self.wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controllers/commands',  # Output wheel velocity topic
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function to compute the velocities for all 4 wheels
        """
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x  # m/s
        angular_velocity = msg.angular.z  # rad/s

        # Compute individual wheel velocities using the basic kinematic model
        # Front-left, Front-right, Rear-left, Rear-right
        front_left_velocity = (linear_velocity - angular_velocity * (self.wheel_base / 2 + self.track_width / 2)) / self.wheel_radius
        front_right_velocity = (linear_velocity + angular_velocity * (self.wheel_base / 2 + self.track_width / 2)) / self.wheel_radius
        rear_left_velocity = (linear_velocity - angular_velocity * (self.wheel_base / 2 + self.track_width / 2)) / self.wheel_radius
        rear_right_velocity = (linear_velocity + angular_velocity * (self.wheel_base / 2 + self.track_width / 2)) / self.wheel_radius

        # Publish the computed velocities
        self.publish_wheel_velocities([front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity])

    def publish_wheel_velocities(self, velocities):
        """
        Publish the computed velocities to the /velocity_controllers/commands topic
        """
        velocity_msg = Float64MultiArray()
        velocity_msg.data = velocities

        self.wheel_velocity_publisher.publish(velocity_msg)

        # Logging for debugging
        self.get_logger().info(f"Wheel Velocities: {velocities}")


def main(args=None):
    rclpy.init(args=args)

    # Start the Four-Wheel Inverse Kinematics Node
    node = FourWheelInverseKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
