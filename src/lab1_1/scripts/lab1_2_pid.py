# !/usr/bin/env python3

####PID

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class PIDPathTracking(Node):
    def __init__(self):
        super().__init__('pid_path_tracking')

        # โหลด path จากไฟล์ path.yaml ในโฟลเดอร์ config ของแพ็กเกจ
        package_name = 'lab1_1'  # เปลี่ยนเป็นชื่อแพ็กเกจของคุณ
        path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')
        if not os.path.exists(path_file):
            self.get_logger().error("ไม่พบไฟล์ path: {}".format(path_file))
            return

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)
        if not self.path:
            self.get_logger().error("ไฟล์ path.yaml ว่างหรือมีรูปแบบไม่ถูกต้อง")
            return

        self.current_pose = None  # (x, y, yaw) ของหุ่นยนต์จาก ground truth
        self.target_index = 0    # waypoint ปัจจุบัน

        # กำหนดพารามิเตอร์ PID สำหรับการควบคุมการเลี้ยว (steering)
        self.declare_parameter('kp', 0.25)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.09)
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # กำหนด tolerance สำหรับมุมและระยะ
        self.declare_parameter('yaw_tolerance', 0.25)         # ถ้า error < 0.05 rad ไม่แก้ไขมาก
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        self.declare_parameter('distance_tolerance', 1.0)       # ถ้าใกล้ waypoint ให้เปลี่ยน waypoint
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # กำหนดความเร็วขาไป (forward velocity)
        self.declare_parameter('forward_velocity', 0.5)
        self.forward_velocity = self.get_parameter('forward_velocity').value

        # ตัวแปรสำหรับเก็บค่าพิกัด PID
        self.prev_error = 0.0
        self.integral = 0.0

        # สร้าง subscriber รับข้อมูล odometry
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        # สร้าง publisher ส่งคำสั่งความเร็ว (cmd_vel) ไปยัง NoSlipModel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer เรียกใช้ control loop ทุก 0.1 วินาที
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """ แปลง quaternion จาก odometry เป็น yaw แล้วเก็บตำแหน่งปัจจุบัน """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_pose = (x, y, yaw)

    def normalize_angle(self, angle):
        """ ทำให้มุมอยู่ในช่วง -π ถึง π """
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        """ ควบคุมหุ่นยนต์ให้เลี้ยวตาม waypoint โดยใช้ PID จากตำแหน่ง x, y """
        # ตรวจสอบว่ามีข้อมูลตำแหน่งและ waypoint อยู่หรือไม่
        if self.current_pose is None or self.target_index >= len(self.path):
            self.get_logger().info("จบการติดตามเส้นทาง")
            return

        # รับ waypoint ปัจจุบันจาก path.yaml
        target = self.path[self.target_index]
        target_x = target['x']
        target_y = target['y']
        # คำนวณ desired_yaw จากตำแหน่ง target x,y กับตำแหน่งปัจจุบัน
        desired_yaw = math.atan2(target_y - self.current_pose[1], target_x - self.current_pose[0])
        
        # คำนวณระยะห่างระหว่างตำแหน่งปัจจุบันกับ waypoint
        error_x = target_x - self.current_pose[0]
        error_y = target_y - self.current_pose[1]
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # คำนวณ yaw error ระหว่าง desired_yaw กับ current yaw
        yaw_error = self.normalize_angle(desired_yaw - self.current_pose[2])

        # ถ้าความคลาดเคลื่อนมุมน้อยกว่า tolerance ให้ control output เป็น 0
        if abs(yaw_error) < self.yaw_tolerance:
            control_output = 0.0
        else:
            # คำนวณ PID สำหรับการเลี้ยว
            self.integral += yaw_error * 0.1   # dt = 0.1 วินาที
            derivative = (yaw_error - self.prev_error) / 0.1
            control_output = self.kp * yaw_error + self.ki * self.integral + self.kd * derivative
        self.prev_error = yaw_error

        # สร้างข้อความคำสั่ง Twist สำหรับควบคุมหุ่นยนต์
        cmd = Twist()
        # ถ้ายังไม่ถึง waypoint ให้ขับไปด้วยความเร็วที่กำหนด
        cmd.linear.x = self.forward_velocity if distance_error > self.distance_tolerance else 0.0
        cmd.angular.z = control_output

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"WP{self.target_index+1}/{len(self.path)}: Target (x,y)=({target_x:.2f}, {target_y:.2f}), Desired yaw={desired_yaw:.2f}, " +
            f"Current yaw={self.current_pose[2]:.2f}, Yaw error={yaw_error:.2f}, Dist error={distance_error:.2f}, Control={control_output:.2f}"
        )

        # ถ้าใกล้ waypoint (ระยะห่างน้อยกว่า tolerance) ให้เปลี่ยนไปยัง waypoint ถัดไป
        if distance_error < self.distance_tolerance:
            self.get_logger().info(f"ถึง waypoint {self.target_index+1}/{len(self.path)}")
            self.target_index += 1
            # รีเซ็ตค่า PID สำหรับ waypoint ถัดไป
            self.integral = 0.0
            self.prev_error = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PIDPathTracking()
    if node.path:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()