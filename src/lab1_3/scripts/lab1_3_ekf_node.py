#!/usr/bin/env python3
"""
ROS2 Node for Extended Kalman Filter Localization with 15-state model using /odom and /gps measurements

State vector (15 states):
    x = [ p_x, p_y, p_z,       -- Position (3)
          roll, pitch, yaw,    -- Orientation in Euler angles (3)
          v_x, v_y, v_z,       -- Linear velocity (3)
          ω_x, ω_y, ω_z,       -- Angular velocity (3)
          a_x, a_y, a_z ]^T    -- Linear acceleration (3)

Prediction Model:
    pₖ₊₁ = pₖ + R(rₖ) * (vₖ Δt + ½ aₖ Δt²)
    rₖ₊₁ = rₖ + J(rₖ) * ωₖ Δt
    vₖ₊₁ = vₖ + aₖ Δt
    ωₖ₊₁ = ωₖ + uₖ^α Δt       (control input in angular acceleration, assumed zero here)
    aₖ₊₁ = aₖ

Measurement Sources:
  - /odom: Odometry message providing:
         z_odom = [ p_x, p_y, p_z, v_x, v_y, v_z ]^T   (6-dim, ใช้ update ความเร็ว)
  - /gps: Odometry message providing:
         z_gps = [ p_x, p_y, yaw ]^T   (3-dim, ใช้ update ตำแหน่งและ yaw)

ผลลัพธ์ที่ได้จะถูก publish เป็น PoseStamped ไปที่ "/ekf_pose"
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Timer period [s]
DT = 0.1

# Process noise covariance Q (15x15)
# ค่าต่อตัวอย่างซึ่งสามารถปรับจูนได้ตามความไม่แน่นอนของแบบจำลอง
Q = np.diag([
    0.05, 0.05, 0.05,            # noise ตำแหน่ง (meters)
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),  # noise orientation (radians)
    0.1, 0.1, 0.1,               # noise ความเร็วเชิงเส้น (m/s)
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),  # noise angular velocity (rad/s)
    0.2, 0.2, 0.2                # noise linear acceleration (m/s^2)
]) ** 2

# Measurement noise covariance สำหรับ odometry (6x6): [ตำแหน่ง (3), ความเร็ว (3)]
R_odom = np.diag([0.2, 0.2, 0.2, 0.1, 0.1, 0.1]) ** 2

# Measurement noise covariance สำหรับ GPS (3x3): [ตำแหน่ง (2) และ yaw (1)]
R_gps = np.diag([0.2, 0.2, np.deg2rad(1.0)]) ** 2

class EKFFullNode(Node):
    def __init__(self):
        super().__init__('ekf_full_node')
        self.dt = DT
        self.last_time = self.get_clock().now()
        
        # สร้าง state vector 15 มิติ: [p; r; v; ω; a]
        self.xEst = np.zeros((15, 1))
        self.PEst = np.eye(15)
        
        # ตัวแปรเก็บค่าการวัดล่าสุด
        self.z_odom = None   # จาก /odom (6 มิติ: p และ v)
        self.new_odom = False
        
        self.z_gps = None    # จาก /gps (3 มิติ: p_x, p_y, yaw)
        self.new_gps = False
        
        # ควบคุม input สำหรับ angular acceleration (u_α), ตั้งเป็นศูนย์
        self.u_alpha = np.zeros((3,1))
        
        # Subscribers สำหรับ /odom และ /gps
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.gps_sub = self.create_subscription(
            Odometry,
            '/gps',
            self.gps_callback,
            10
        )
        
        # Publisher สำหรับ estimated pose
        self.ekf_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        # Publisher สำหรับ estimated odometry
        self.odom1_pub = self.create_publisher(Odometry, '/odom1', 10)

        
        # Timer สำหรับเรียก EKF update ทุก dt วินาที
        self.timer = self.create_timer(self.dt, self.timer_callback)
    def publish_odom1(self):
        """
        Publish estimated state (x, y, yaw) to /odom1 as Odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position (x, y)
        odom_msg.pose.pose.position.x = self.xEst[0, 0]
        odom_msg.pose.pose.position.y = self.xEst[1, 0]
        odom_msg.pose.pose.position.z = 0.0  # Assume ground vehicle

        # Convert yaw to quaternion
        roll = self.xEst[3, 0]
        pitch = self.xEst[4, 0]
        yaw = self.xEst[5, 0]
        quat = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set linear velocity
        odom_msg.twist.twist.linear.x = self.xEst[6, 0]  # v_x
        odom_msg.twist.twist.linear.y = self.xEst[7, 0]  # v_y
        odom_msg.twist.twist.angular.z = self.xEst[11, 0]  # ω_z (yaw rate)

        self.odom1_pub.publish(odom_msg)

    # --- ฟังก์ชันช่วยสำหรับคำนวณ rotation matrix และ matrix แปลง angular velocity ---
    def R_from_euler(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
        return R

    def J_from_euler(self, roll, pitch, yaw):
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        tan_pitch = math.tan(pitch)
        J = np.array([
            [1, sin_roll*tan_pitch, cos_roll*tan_pitch],
            [0, cos_roll, -sin_roll],
            [0, sin_roll/math.cos(pitch), cos_roll/math.cos(pitch)]
        ])
        return J
    # --- สิ้นสุดฟังก์ชันช่วย ---
    
    def odom_callback(self, msg):
        # ดึงข้อมูลจาก /odom: ตำแหน่งและความเร็ว
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        
        self.z_odom = np.array([[px], [py], [pz], [vx], [vy], [vz]])
        self.new_odom = True
    
    def gps_callback(self, msg):
        # ดึงข้อมูลจาก /gps: ตำแหน่ง (x, y) และ yaw จาก quaternion
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.z_gps = np.array([[px], [py], [yaw]])
        self.new_gps = True
    
    def dynamic_model(self, x, dt):
        """
        แบบจำลองการคาดการณ์ state:
          pₖ₊₁ = pₖ + R(rₖ)*(vₖ*dt + 0.5*aₖ*dt²)
          rₖ₊₁ = rₖ + J(rₖ)*ωₖ*dt
          vₖ₊₁ = vₖ + aₖ*dt
          ωₖ₊₁ = ωₖ + u_α*dt  (โดย u_α = 0)
          aₖ₊₁ = aₖ
        """
        x_new = np.zeros((15,1))
        roll = x[3,0]
        pitch = x[4,0]
        yaw = x[5,0]
        R_mat = self.R_from_euler(roll, pitch, yaw)
        b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
        x_new[0:3] = x[0:3] + R_mat @ b
        
        J_mat = self.J_from_euler(roll, pitch, yaw)
        x_new[3:6] = x[3:6] + J_mat @ x[9:12] * dt
        
        x_new[6:9] = x[6:9] + x[12:15] * dt
        x_new[9:12] = x[9:12] + self.u_alpha * dt
        x_new[12:15] = x[12:15]
        
        return x_new
    
    def jacobian_F(self, x, dt):
        """
        คำนวณ Jacobian ของแบบจำลองการคาดการณ์ (ใช้สำหรับ update covariance)
        """
        F = np.eye(15)
        I3 = np.eye(3)
        
        roll = x[3,0]
        pitch = x[4,0]
        yaw = x[5,0]
        R_mat = self.R_from_euler(roll, pitch, yaw)
        
        # คำนวณอนุพันธ์ของ R ตาม roll, pitch, yaw
        dR_dr = np.zeros((3,3,3))
        dR_dr[:,:,0] = self.dR_droll(roll, pitch, yaw)
        dR_dr[:,:,1] = self.dR_dpitch(roll, pitch, yaw)
        dR_dr[:,:,2] = self.dR_dyaw(roll, pitch, yaw)
        
        b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
        L = np.zeros((3,3))
        for i in range(3):
            L[:, i] = (dR_dr[:,:,i] @ b).flatten()
        F[0:3, 3:6] = L
        F[0:3, 6:9] = R_mat * dt
        F[0:3, 12:15] = R_mat * (0.5 * dt**2)
        
        J_mat = self.J_from_euler(roll, pitch, yaw)
        dJ_droll_mat = self.dJ_droll(roll, pitch, yaw)
        dJ_dpitch_mat = self.dJ_dpitch(roll, pitch, yaw)
        dJ_dyaw_mat = self.dJ_dyaw(roll, pitch, yaw)
        omega = x[9:12]
        M = dJ_droll_mat * omega[0,0] + dJ_dpitch_mat * omega[1,0] + dJ_dyaw_mat * omega[2,0]
        F[3:6, 3:6] = np.eye(3) + M * dt
        F[3:6, 9:12] = self.J_from_euler(roll, pitch, yaw) * dt
        
        F[6:9, 12:15] = I3 * dt
        
        return F
    
    # ฟังก์ชันอนุพันธ์ของ R (สำหรับ Jacobian)
    def dR_droll(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        Rz = np.array([[cy, -sy, 0],
                       [sy, cy,  0],
                       [0,  0,   1]])
        Ry = np.array([[cp, 0, sp],
                       [0, 1,  0],
                       [-sp,0, cp]])
        dRx = np.array([[0, 0, 0],
                        [0, -sr, -cr],
                        [0, cr, -sr]])
        return Rz @ Ry @ dRx

    def dR_dpitch(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        Rz = np.array([[cy, -sy, 0],
                       [sy, cy,  0],
                       [0,  0,   1]])
        dRy = np.array([[-sp, 0, cp],
                        [0, 0, 0],
                        [-cp, 0, -sp]])
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])
        return Rz @ dRy @ Rx

    def dR_dyaw(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        dRz = np.array([[-sy, -cy, 0],
                        [cy, -sy, 0],
                        [0, 0, 0]])
        Ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])
        return dRz @ Ry @ Rx

    # ฟังก์ชันอนุพันธ์ของ J (สำหรับ Jacobian)
    def dJ_droll(self, roll, pitch, yaw):
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        tan_pitch = math.tan(pitch)
        cos_pitch = math.cos(pitch)
        dJ = np.zeros((3,3))
        dJ[0,1] = cos_roll * tan_pitch
        dJ[0,2] = -sin_roll * tan_pitch
        dJ[1,1] = -sin_roll
        dJ[1,2] = -cos_roll
        dJ[2,1] = cos_roll / cos_pitch
        dJ[2,2] = -sin_roll / cos_pitch
        return dJ

    def dJ_dpitch(self, roll, pitch, yaw):
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        sec_pitch2 = 1.0/(cos_pitch**2)
        dJ = np.zeros((3,3))
        dJ[0,1] = sin_roll * sec_pitch2
        dJ[0,2] = cos_roll * sec_pitch2
        dJ[2,1] = sin_roll * sin_pitch/(cos_pitch**2)
        dJ[2,2] = cos_roll * sin_pitch/(cos_pitch**2)
        return dJ

    def dJ_dyaw(self, roll, pitch, yaw):
        # สำหรับ ZYX Euler angles, J ไม่ขึ้นกับ yaw โดยตรง
        return np.zeros((3,3))
    
    def ekf_predict(self, dt):
        F = self.jacobian_F(self.xEst, dt)
        xPred = self.dynamic_model(self.xEst, dt)
        PPred = F @ self.PEst @ F.T + Q
        self.xEst = xPred
        self.PEst = PPred
    
    def ekf_update_odom(self, z):
        """
        อัปเดตโดยใช้การวัดจาก /odom (6 มิติ)
        โดยที่ matrix สังเกต H จะเลือกเอาเฉพาะค่า linear velocity (v_x, v_y, v_z)
        (สามารถเปิดใช้ตำแหน่งได้ถ้าต้องการ)
        """
        H = np.zeros((6, 15))
        # H[0:3, 0:3] = np.eye(3)  # อัปเดตตำแหน่ง ถ้าต้องการ
        H[3:6, 6:9] = np.eye(3)   # อัปเดตความเร็วเชิงเส้น
        
        zPred = H @ self.xEst
        y = z - zPred
        S = H @ self.PEst @ H.T + R_odom
        K = self.PEst @ H.T @ np.linalg.inv(S)
        self.xEst = self.xEst + K @ y
        self.PEst = (np.eye(15) - K @ H) @ self.PEst
    
    def ekf_update_gps(self, z):
        """
        อัปเดตโดยใช้การวัดจาก /gps (3 มิติ)
        โดย matrix สังเกต H เลือกค่า:
          p_x (index 0), p_y (index 1) และ yaw (index 5 ในส่วน orientation)
        """
        H = np.zeros((3, 15))
        H[0, 0] = 1   # p_x
        H[1, 1] = 1   # p_y
        H[2, 5] = 1   # yaw
        
        zPred = H @ self.xEst
        y = z - zPred
        S = H @ self.PEst @ H.T + R_gps
        K = self.PEst @ H.T @ np.linalg.inv(S)
        self.xEst = self.xEst + K @ y
        self.PEst = (np.eye(15) - K @ H) @ self.PEst
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        # ขั้นตอน prediction
        self.ekf_predict(dt)
        
        # ถ้ามีข้อมูล odom ใหม่ให้ update
        if self.new_odom and self.z_odom is not None:
            self.ekf_update_odom(self.z_odom)
            self.new_odom = False
        
        # ถ้ามีข้อมูล gps ใหม่ให้ update
        if self.new_gps and self.z_gps is not None:
            self.ekf_update_gps(self.z_gps)
            self.new_gps = False
        
        self.publish_estimate()
        self.publish_odom1()

    
    def publish_estimate(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = self.xEst[0, 0]
        msg.pose.position.y = self.xEst[1, 0]
        msg.pose.position.z = self.xEst[2, 0]
        
        roll = self.xEst[3, 0]
        pitch = self.xEst[4, 0]
        yaw = self.xEst[5, 0]
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFFullNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
