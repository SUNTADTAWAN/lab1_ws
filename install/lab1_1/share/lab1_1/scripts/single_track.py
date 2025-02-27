#!/usr/bin/env python3
###single track
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
import math
import tf_transformations


class SingleTrack(Node):
    def __init__(self):
        super().__init__('single_track_node')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.35)     # Distance between front & rear axles
        self.declare_parameter('track_width', 0.2)    # Distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.045) # Wheel radius
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # Load parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Steering and wheel speed state
        self.steering_angle = 0.0
        self.wheel_speed = 0.0

        self.last_time = self.get_clock().now()

        # Subscribe to /joint_states
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Timer for odometry updates
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info("YawRateOdom node started!")

    def joint_states_callback(self, msg: JointState):
        """
        Reads joint states for steering angle and wheel speed.
        """
        try:
            # Find indices of the joints in the received message
            idx_fl_steer = msg.name.index("steering_joint_front_left")
            idx_fr_steer = msg.name.index("steering_joint_front_right")
            idx_fl_wheel = msg.name.index("rotation_joint_front_left")
            idx_fr_wheel = msg.name.index("rotation_joint_front_right")

            # Compute the **average steering angle** (Single-Track Model assumption)
            steer_left = msg.position[idx_fl_steer]
            steer_right = msg.position[idx_fr_steer]
            self.steering_angle = (steer_left + steer_right) / 2.0  # Use average steering

            # Compute the **average wheel speed**
            speed_left = msg.velocity[idx_fl_wheel] * self.wheel_radius  # m/s
            speed_right = msg.velocity[idx_fr_wheel] * self.wheel_radius  # m/s
            self.wheel_speed = (speed_left + (-speed_right)) / 2.0  # Average speed

            # Compute yaw rate from Ackermann kinematics
            self.angular_velocity = self.compute_yaw_rate()

        except ValueError as e:
            self.get_logger().warn("Joint names mismatch: " + str(e))

    def compute_yaw_rate(self):
        """
        Computes yaw rate using Ackermann Single-Track Model.
        """
        if abs(self.steering_angle) < 1e-6:
            return 0.0  # Moving straight

        # Compute turning radius
        turning_radius = self.wheel_base / math.tan(self.steering_angle)
        yaw_rate = self.wheel_speed / turning_radius

        return yaw_rate

    def update_odometry(self):
        """
        Updates odometry and publishes it.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Update position and yaw using integration
        self.x += self.wheel_speed * math.cos(self.yaw) * dt
        self.y += self.wheel_speed * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.wheel_speed
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom_msg)

        self.get_logger().info(
            f"Odom -> x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f} rad, v: {self.wheel_speed:.2f} m/s, ω: {self.angular_velocity:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SingleTrack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
