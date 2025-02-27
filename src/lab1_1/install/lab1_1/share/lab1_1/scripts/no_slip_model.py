#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class NoSlipModel(Node):
    def __init__(self):
        super().__init__('no_slip_model_node')

        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.045)  # Wheel radius (m)
        self.declare_parameter('wheel_base', 0.35)  # Distance between front and rear axles (m)
        self.declare_parameter('track_width', 0.2)  # Distance between left and right wheels (m)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value

        # Subscribe to /cmd_vel topic
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publishers for steering and velocity controllers
        self.pub_wheel_velocity = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_steering_angle = self.create_publisher(Float64MultiArray, "/joint_trajectory_position_controller/commands", 10)

    def cmd_vel_callback(self, msg):
        """
        Convert linear.x and angular.z velocities into steering angles and wheel velocities.
        """
        linear_velocity = msg.linear.x  # Forward velocity (m/s)
        angular_velocity = msg.angular.z  # Rotation speed (rad/s)

        if linear_velocity == 0 and angular_velocity == 0:
            # Stop the robot if no command is given
            self.publish_commands(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # Compute turn radius if turning
        if angular_velocity != 0:
            turn_radius = linear_velocity / angular_velocity if linear_velocity != 0 else float('inf')
            inner_wheel_radius = turn_radius - (self.track_width / 2)
            outer_wheel_radius = turn_radius + (self.track_width / 2)

            # Compute left and right steering angles
            steer_left = math.atan(self.wheel_base / inner_wheel_radius) if abs(inner_wheel_radius) > 1e-6 else 0.0
            steer_right = math.atan(self.wheel_base / outer_wheel_radius) if abs(outer_wheel_radius) > 1e-6 else 0.0
        else:
            steer_left = steer_right = 0.0  # Move straight

        # Compute wheel velocities based on Ackermann model
        left_wheel_velocity = linear_velocity / self.wheel_radius
        right_wheel_velocity = linear_velocity / self.wheel_radius

        # Publish steering angles and wheel velocities
        self.publish_commands(left_wheel_velocity, -right_wheel_velocity, left_wheel_velocity, -right_wheel_velocity, steer_left, steer_right)

        self.get_logger().info(
            f"Cmd Vel -> Linear: {linear_velocity:.2f} m/s, Angular: {angular_velocity:.2f} rad/s"
        )
        self.get_logger().info(
            f"Steering Angles -> Left: {steer_left:.2f} rad, Right: {steer_right:.2f} rad"
        )
        self.get_logger().info(
            f"Wheel Velocities -> Left: {left_wheel_velocity:.2f} rad/s, Right: {right_wheel_velocity:.2f} rad/s"
         )


    # def cmd_vel_callback(self, msg):
    #     linear_velocity  = msg.linear.x    # v (m/s) อ้างอิงกึ่งกลางเพลาหลัง
    #     angular_velocity = msg.angular.z   # w (rad/s)

    #     # ถ้าไม่มีคำสั่งวิ่งหรือเลี้ยว ก็หยุดล้อไป
    #     if abs(linear_velocity) < 1e-6 and abs(angular_velocity) < 1e-6:
    #         self.publish_commands(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    #         return

    #     # Wheel base (L) และ Track width (T) ที่ได้จาก parameter
    #     L = self.wheel_base
    #     T = self.track_width
    #     r = self.wheel_radius

    #     if abs(angular_velocity) < 1e-6:
    #         # วิ่งตรง ไม่เลี้ยว
    #         steer_left = 0.0
    #         steer_right = 0.0
    #         # ความเร็วล้อทุกล้อเท่ากัน
    #         w_rf = w_rr = linear_velocity / r
    #         w_lr = w_lf = linear_velocity / r

    #     else:
    #         # มีการเลี้ยว
    #         R = linear_velocity / angular_velocity  # รัศมีจากกึ่งกลางเพลาหลังไปยัง ICC (ซ้ายหรือขวาขึ้นอยู่กับ sign)

    #         # คำนวณระยะจาก ICC ไปยังล้อแต่ละข้าง
    #         R_lr = R - (T/2.0)   # ล้อหลังซ้าย
    #         R_rr = R + (T/2.0)   # ล้อหลังขวา
    #         # ล้อหน้าซ้าย
    #         R_lf = math.sqrt(R_lr**2 + L**2)
    #         # ล้อหน้าขวา
    #         R_rf = math.sqrt(R_rr**2 + L**2)

    #         # คำนวณความเร็วเชิงเส้นของล้อแต่ละข้าง (สมมติ linear_velocity คือความเร็วของจุดที่ R ห่างจาก ICC)
    #         # ถ้า R<0 แปลว่าเลี้ยวอีกข้าง อาจต้องระวังเรื่องเครื่องหมาย แต่โดยทั่วไป v * (ระยะล้อ / R) ก็เพียงพอ
    #         v_lf = linear_velocity * (R_lf / abs(R))
    #         v_rf = linear_velocity * (R_rf / abs(R))
    #         v_lr = linear_velocity * (abs(R_lr) / abs(R))
    #         v_rr = linear_velocity * (abs(R_rr) / abs(R))

    #         # แปลงเป็นความเร็วเชิงมุม (rad/s)
    #         w_lf = v_lf / r
    #         w_rf = v_rf / r
    #         w_lr = v_lr / r
    #         w_rr = v_rr / r

    #         # มุมบังคับเลี้ยวล้อหน้า (ซ้าย/ขวา)
    #         steer_left  = math.atan2(L, R_lr) if abs(R_lr) > 1e-6 else 0.0
    #         steer_right = math.atan2(L, R_rr) if abs(R_rr) > 1e-6 else 0.0

    #     # หมายเหตุ: ถ้าต้องกลับด้านหรือคูณ -1 ฝั่งขวา ก็ปรับตาม joint direction
    #     # เช่น ถ้า joint ขวาหมุนกลับทิศ

    #     # ส่งออกไป publish
    #     self.publish_commands(
    #         w_lf, -w_rf, w_lr, -w_rr,
    #         steer_left, steer_right
    #     )

    def publish_commands(self, left_front, right_front, left_rear, right_rear, steer_left, steer_right):
        """
        Publish wheel velocities and steering angles.
        """
        # Publish steering angles to joint_trajectory_position_controller
        steering_msg = Float64MultiArray()
        steering_msg.data = [steer_left, steer_right]
        self.pub_steering_angle.publish(steering_msg)

        # Publish wheel velocities to velocity_controllers
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [left_front, right_front, left_rear, right_rear]
        self.pub_wheel_velocity.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoSlipModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
