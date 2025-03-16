#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquarePathPublisher(Node):
    def __init__(self):
        super().__init__('square_path_publisher')

        # Publisher สำหรับส่งคำสั่งไปที่ /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ความเร็วของหุ่นยนต์
        self.linear_velocity = 0.5  # m/s
        self.angular_velocity = 0.5  # rad/s (ค่อยๆ เลี้ยว)

        # ระยะเวลาสำหรับการเดินทางแต่ละช่วง
        self.forward_time = 8.0   # วินาที (ไปข้างหน้า)
        self.turn_time = 3.0      # วินาที (หมุนเพื่อโค้ง)

        self.run_square_path()

    def run_square_path(self):
        twist = Twist()

        for _ in range(4):  # ทำ 4 รอบ (4 ด้านของสี่เหลี่ยม)
            # **เดินตรง**
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Moving forward")
            time.sleep(self.forward_time)  # รอให้เคลื่อนที่ตรง

            # **หมุนโค้ง**
            twist.linear.x = self.linear_velocity * 0.8  # ลดความเร็วลงขณะเลี้ยว
            twist.angular.z = self.angular_velocity
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Turning with curve")
            time.sleep(self.turn_time)  # รอให้หมุนโค้ง

        # หยุดรถหลังจากเคลื่อนที่ครบ 4 รอบ
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Stopped")

def main(args=None):
    rclpy.init(args=args)
    node = SquarePathPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
