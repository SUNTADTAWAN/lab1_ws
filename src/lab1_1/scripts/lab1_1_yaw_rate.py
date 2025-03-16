#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import tf_transformations

class AckermannOdom(Node):
    def __init__(self):
        super().__init__('ackermann_odom_node')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.35)      # Distance between front & rear axles
        self.declare_parameter('track_width', 0.2)     # Distance between left & right wheels
        self.declare_parameter('wheel_radius', 0.045)  # Wheel radius
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)   # Hz

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

        # Joint state storage
        self.joint_positions = {}
        self.joint_velocities = {}

        # Timestamp tracking
        self.last_time = self.get_clock().now()

        # Subscribe to /joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to update odometry
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info("AckermannOdom node started!")

    def joint_state_callback(self, msg: JointState):
        """
        Reads joint states from /joint_states and stores them.
        """
        for i, name in enumerate(msg.name):
            position = msg.position[i] if len(msg.position) > i else 0.0
            velocity = msg.velocity[i] if len(msg.velocity) > i else 0.0

            self.joint_positions[name] = position
            self.joint_velocities[name] = velocity

    def update_odometry(self):
        """
        Computes (x, y, yaw) using Ackermann Model and updates odometry.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # 1) Read front wheel steering angles
        steering_left = self.joint_positions.get('steering_joint_front_left', 0.0)
        steering_right = self.joint_positions.get('steering_joint_front_right', 0.0)

        # Compute average steering angle and clamp within [-0.5, 0.5] rad
        steer_angle = (steering_left + steering_right) / 2.0
        steer_angle = max(-0.5, min(steer_angle, 0.5))  # Clamping

        # 2) Read wheel rotational velocities
        v_left = self.joint_velocities.get('rotation_joint_front_left', 0.0)
        v_right = -self.joint_velocities.get('rotation_joint_front_right', 0.0)

        # Convert wheel angular velocity (rad/s) to linear velocity (m/s)
        v_forward = ((v_left + v_right) / 2.0) * self.wheel_radius

        # 3) Compute yaw rate from Ackermann steering geometry
        if abs(math.tan(steer_angle)) < 1e-6:
            yaw_rate = 0.0  # Moving straight
        else:
            turning_radius = self.wheel_base / math.tan(steer_angle)
            yaw_rate = v_forward / turning_radius

        # 4) Update (x, y, yaw) position
        self.x += v_forward * math.cos(self.yaw) * dt
        self.y += v_forward * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 5) Publish Odometry
        self.publish_odometry(current_time, v_forward, yaw_rate)

    def publish_odometry(self, current_time, v_forward, yaw_rate):
        """
        Creates and publishes the Odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Set position (x, y)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation (yaw -> quaternion)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set velocity (linear & angular)
        odom_msg.twist.twist.linear.x = v_forward
        odom_msg.twist.twist.angular.z = yaw_rate

        # Publish the message
        self.odom_pub.publish(odom_msg)

        self.get_logger().info(
            f"📌 Odom: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad, v={v_forward:.2f} m/s, ω={yaw_rate:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
