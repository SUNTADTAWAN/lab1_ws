#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading

class RobotPathPlotter(Node):
    def __init__(self):
        super().__init__('robot_path_plotter')

        # Subscribe ไปที่ topic /odometry/ground_truth และ /odom
        self.create_subscription(Odometry, '/odometry/ground_truth', self.ground_truth_callback, 10)
        self.create_subscription(Odometry, '/odom1', self.odom_callback, 10)

        # เก็บตำแหน่งของหุ่นยนต์
        self.gt_x_positions = []  # ตำแหน่งจริง (ground truth)
        self.gt_y_positions = []
        self.odom_x_positions = []  # ตำแหน่งจาก odometry คำนวณ
        self.odom_y_positions = []

        # สถานะบันทึกข้อมูล
        self.recording = True

        # สร้าง thread สำหรับ GUI
        self.plot_thread = threading.Thread(target=self.plot_graph)
        self.plot_thread.start()

    def ground_truth_callback(self, msg):
        """Callback สำหรับรับค่าตำแหน่งจริงจาก /odometry/ground_truth"""
        if self.recording:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.gt_x_positions.append(x)
            self.gt_y_positions.append(y)
            self.get_logger().info(f'Ground Truth -> X: {x:.2f}, Y: {y:.2f}')

    def odom_callback(self, msg):
        """Callback สำหรับรับค่าตำแหน่งจาก /odom (Odometry คำนวณเอง)"""
        if self.recording:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.odom_x_positions.append(x)
            self.odom_y_positions.append(y)
            self.get_logger().info(f'Odometry -> X: {x:.2f}, Y: {y:.2f}')

    def plot_graph(self):
        """แสดงกราฟตำแหน่งของหุ่นยนต์"""
        plt.ion()  # เปิด interactive mode
        fig, ax = plt.subplots()
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Robot Path")

        while rclpy.ok():
            if self.recording:
                ax.clear()
                ax.plot(self.gt_x_positions, self.gt_y_positions, 'b-', linewidth=2, label="Ground Truth Path")
                ax.plot(self.odom_x_positions, self.odom_y_positions, 'r-', linewidth=2, label="Odometry Path")
                ax.legend()
                plt.pause(0.1)  # อัปเดตทุก 0.1 วินาที

            # ตรวจจับการกดปุ่ม 'q' เพื่อหยุดบันทึก
            if plt.waitforbuttonpress(timeout=0.1):
                self.recording = False
                plt.ioff()  # ปิด interactive mode
                plt.show()
                break

def main(args=None):
    rclpy.init(args=args)
    node = RobotPathPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
