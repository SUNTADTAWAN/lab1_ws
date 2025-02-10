# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Quaternion
# import tf_transformations
# from math import sin, cos

# class OdomPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_publisher')
        
#         # Create publisher for odometry
#         self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
#         # Create timer for publishing at 50Hz
#         self.timer = self.create_timer(0.05, self.publish_odom)
        
#         # Initialize variables for odometry
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.vx = 0.1  # Linear velocity in m/s
#         self.vth = 0.1 # Angular velocity in rad/s

#         self.last_time = self.get_clock().now()

#     def publish_odom(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # Update pose
#         delta_x = self.vx * cos(self.theta) * dt
#         delta_y = self.vx * sin(self.theta) * dt
#         delta_theta = self.vth * dt

#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta

#         # Create quaternion for orientation
#         quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)

#         # Populate Odometry message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base'

#         # Set pose
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation = Quaternion(
#             x=quat[0], y=quat[1], z=quat[2], w=quat[3]
#         )

#         # Set velocities
#         odom.twist.twist.linear.x = self.vx
#         odom.twist.twist.linear.y = 0.0
#         odom.twist.twist.angular.z = self.vth

#         # Publish odometry
#         self.odom_publisher.publish(odom)
#         self.get_logger().info("Published odometry!")

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
