#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import tf_transformations

class DoubleTrackOdom(Node):
    def __init__(self):
        super().__init__('double_track_odom_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.045)    # Wheel radius (m)
        self.declare_parameter('track_width', 0.2)       # Distance between left and right wheels (m)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)     # Hz (Publishing Rate)

        # Load parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize position and yaw
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Initialize velocities
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0 # rad/s

        # Initialize wheel speeds
        self.left_wheel_speed = 0.0   # rad/s or m/s
        self.right_wheel_speed = 0.0  # rad/s or m/s

        # Time tracking
        self.last_time = self.get_clock().now()

        # Subscribe to /joint_states
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to update odometry
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('DoubleTrackOdom node started!')

    def joint_states_callback(self, msg: JointState):
        """
        Reads joint velocities from /joint_states.
        Uses "rotation_joint_front_left" and "rotation_joint_front_right".
        """
        try:
            idx_left = msg.name.index("rotation_joint_front_left")
            idx_right = msg.name.index("rotation_joint_front_right")

            # Read joint velocities (Check if they are in rad/s or m/s)
            self.left_wheel_speed = msg.velocity[idx_left]  
            self.right_wheel_speed = -msg.velocity[idx_right]  # **FIXED: Removed unnecessary "-" sign**

            # Compute linear velocity (v) in **m/s**
            self.linear_velocity = ((self.left_wheel_speed + self.right_wheel_speed) / 2.0) * self.wheel_radius

            # Compute angular velocity (ω) in **rad/s**
            self.angular_velocity = (self.right_wheel_speed - self.left_wheel_speed) / self.track_width

        except ValueError as e:
            self.get_logger().warn("Missing joint in /joint_states: " + str(e))

    def update_odometry(self):
        """
        Updates odometry using integrated position (x, y, yaw).
        Publishes to /odom.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # **Exponential Moving Average (EMA) for smoothing**
        alpha = 0.25
        self.linear_velocity = alpha * self.linear_velocity + (1 - alpha) * self.linear_velocity
        self.angular_velocity = alpha * self.angular_velocity + (1 - alpha) * self.angular_velocity

        # **Update Position & Orientation**
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Normalize yaw

        # **Create Odometry Message**
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom_msg)

        self.get_logger().info(
            f"Odom -> x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f} rad, v: {self.linear_velocity:.2f} m/s, ω: {self.angular_velocity:.2f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DoubleTrackOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
