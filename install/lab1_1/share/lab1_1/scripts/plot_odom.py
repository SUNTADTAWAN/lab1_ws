#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import tf_transformations
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time


class TFToOdometryVisualizer(Node):
    def __init__(self):
        super().__init__('tf_to_odom_visualizer')

        # Subscribe to /tf
        self.tf_subscriber = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)

        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Timer to enforce real-time updates (20Hz)
        self.create_timer(0.05, self.publish_latest_odom)  # 20Hz (0.05s per update)

        # Store the latest odom data
        self.latest_odom = None
        self.x_data = []
        self.y_data = []

        # Start Matplotlib in a separate thread
        threading.Thread(target=self.start_plot, daemon=True).start()

        self.get_logger().info("✅ TF to Odometry Node with Real-Time Plot Started!")

    def tf_callback(self, msg):
        """
        Extracts position and yaw from /tf and stores it.
        """
        for transform in msg.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base":
                # Extract position
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                # Extract quaternion rotation
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w

                # Convert quaternion to yaw
                _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base"

                # Set position
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = z
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw

                # Store the latest odometry message
                self.latest_odom = odom_msg

                # Store position for visualization
                self.x_data.append(x)
                self.y_data.append(y)

    def publish_latest_odom(self):
        """
        Publishes the most recent odometry message at a fixed rate.
        """
        if self.latest_odom:
            self.odom_pub.publish(self.latest_odom)
            self.get_logger().info(
                f"📡 Published Odometry -> x: {self.latest_odom.pose.pose.position.x:.2f}, "
                f"y: {self.latest_odom.pose.pose.position.y:.2f}, "
                f"yaw: {math.degrees(tf_transformations.euler_from_quaternion([self.latest_odom.pose.pose.orientation.x, self.latest_odom.pose.pose.orientation.y, self.latest_odom.pose.pose.orientation.z, self.latest_odom.pose.pose.orientation.w])[2]):.2f}°"
            )

    def start_plot(self):
        """
        Starts the real-time Matplotlib plot in a separate thread.
        """
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.set_title("Real-Time Robot Position (odom → base)")
        self.ax.grid()

        # Use FuncAnimation for smooth updates
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

        # Keep the plot window open
        plt.show(block=True)

    def update_plot(self, frame):
        """
        Updates the Matplotlib plot dynamically.
        """
        self.ax.clear()
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.set_title("Real-Time Robot Position (odom → base)")
        self.ax.grid()

        if len(self.x_data) > 0:
            self.ax.plot(self.x_data, self.y_data, 'b-', marker='o', markersize=3, label="Robot Path")
            self.ax.legend()

        # Ensure Matplotlib updates correctly
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = TFToOdometryVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
