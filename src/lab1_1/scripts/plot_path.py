#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading
import numpy as np

class RobotPathPlotter(Node):
    def __init__(self):
        super().__init__('robot_path_plotter')

        # Subscribe ไปที่ topic /odom
        self.subscription = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.subscription  # ป้องกัน garbage collection

        # เก็บตำแหน่งของหุ่นยนต์
        self.x_positions = []
        self.y_positions = []

        # สถานะบันทึกข้อมูล
        self.recording = True

        # สร้าง thread สำหรับ GUI
        self.plot_thread = threading.Thread(target=self.plot_graph)
        self.plot_thread.start()

    def odom_callback(self, msg):
        if self.recording:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.x_positions.append(x)
            self.y_positions.append(y)
            self.get_logger().info(f'Received Position -> X: {x:.2f}, Y: {y:.2f}')

    def plot_graph(self):
        plt.ion()  # เปิด interactive mode
        fig, ax = plt.subplots()
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Robot Path")
        
        while rclpy.ok():
            if self.recording:
                ax.clear()
                ax.plot(self.x_positions, self.y_positions, 'b-', linewidth=2, label="Robot Path")
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
