#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import time
import tf_transformations

class YawRateOdom(Node):
    def __init__(self):
        super().__init__('yaw_rate_odom_node')

        # ประกาศพารามิเตอร์เริ่มต้น
        self.declare_parameter('wheel_base', 0.35)     # ระยะฐานล้อ (หน้า-หลัง)
        self.declare_parameter('track_width', 0.2)    # ระยะห่างล้อซ้าย-ขวา
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)   # Hz

        # อ่านค่าพารามิเตอร์
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ตำแหน่งและ orientation เริ่มต้น
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ความเร็วเชิงเส้นและเชิงมุม (อัปเดตจาก /cmd_vel)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Timestamp ล่าสุดที่ได้รับ cmd_vel
        self.last_time = self.get_clock().now()

        # Subscriber รับ /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher ส่ง odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # สร้าง timer เพื่อคำนวณ odom เป็นระยะ
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('YawRateOdom node started!')

    def cmd_vel_callback(self, msg: Twist):
        """
        รับค่าความเร็วเชิงเส้นและเชิงมุมจาก /cmd_vel
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_odometry(self):
        """
        คำนวณตำแหน่ง x, y, yaw จาก linear_velocity, angular_velocity แล้ว publish Odometry
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # แปลง nanosec -> sec
        self.last_time = current_time

        # -- 1) คำนวณตำแหน่งใหม่ด้วย Forward Kinematics (No-Slip Model) --
        # x_dot = v * cos(yaw)
        # y_dot = v * sin(yaw)
        # yaw_dot = w
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt

        # ปรับ yaw ให้อยู่ในช่วง -pi ถึง pi (ถ้าต้องการ)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # -- 2) สร้างข้อความ Odometry --
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id     # ปกติ = 'odom'
        odom_msg.child_frame_id = self.child_frame_id     # ปกติ = 'base_link'

        # ตำแหน่ง (x, y)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # สร้าง quaternion จาก yaw
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # กำหนด twist (v, w) ใน odom
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # -- 3) Publish odometry --
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawRateOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
