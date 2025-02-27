#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odometry_to_tf')
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to /odometry/ground_truth
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/ground_truth',
            self.odom_callback,
            10
        )
        
        self.get_logger().info("Publishing TF from /odometry/ground_truth...")

    def odom_callback(self, msg):
        t = TransformStamped()

        # Set timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"  # Parent frame
        t.child_frame_id = "base"  # Child frame (robot frame)

        # Copy position from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z  # Usually 0 for ground robots

        # Copy orientation from odometry
        t.transform.rotation = msg.pose.pose.orientation

        # Publish transform
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion, TransformStamped
# from tf2_ros import TransformBroadcaster
# import tf_transformations

# class OdomPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_publisher')
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
#         self.x, self.y, self.theta = 0.0, 0.0, 0.0
#         self.vx, self.vtheta = 0.0, 0.0

#     def publish_odom(self):
#         current_time = self.get_clock().now().to_msg()

#         # Create odometry message
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time
#         odom_msg.header.frame_id = "odom"
#         odom_msg.child_frame_id = "base"

#         # Position
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)

#         # Correct Quaternion assignment
#         odom_msg.pose.pose.orientation.x = odom_quat[0]
#         odom_msg.pose.pose.orientation.y = odom_quat[1]
#         odom_msg.pose.pose.orientation.z = odom_quat[2]
#         odom_msg.pose.pose.orientation.w = odom_quat[3]

#         # Velocity
#         odom_msg.twist.twist.linear.x = self.vx
#         odom_msg.twist.twist.angular.z = self.vtheta

#         # Publish TF
#         t = TransformStamped()
#         t.header.stamp = current_time
#         t.header.frame_id = "odom"
#         t.child_frame_id = "base"
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y

#         # Correct Quaternion assignment
#         t.transform.rotation.x = odom_quat[0]
#         t.transform.rotation.y = odom_quat[1]
#         t.transform.rotation.z = odom_quat[2]
#         t.transform.rotation.w = odom_quat[3]

#         self.tf_broadcaster.sendTransform(t)

#         # Publish odometry
#         self.odom_pub.publish(odom_msg)

#         # Update position
#         self.x += self.vx * 0.1
#         self.theta += self.vtheta * 0.1

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

