#!/usr/bin/env python3

#####PID

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
# import math
# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory

# class PIDPathTracking(Node):
#     def __init__(self):
#         super().__init__('pid_path_tracking')

#         # โหลด path จากไฟล์ path.yaml ในโฟลเดอร์ config ของแพ็กเกจ
#         package_name = 'lab1_1'  # เปลี่ยนเป็นชื่อแพ็กเกจของคุณ
#         path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')
#         if not os.path.exists(path_file):
#             self.get_logger().error("ไม่พบไฟล์ path: {}".format(path_file))
#             return

#         with open(path_file, 'r') as file:
#             self.path = yaml.safe_load(file)
#         if not self.path:
#             self.get_logger().error("ไฟล์ path.yaml ว่างหรือมีรูปแบบไม่ถูกต้อง")
#             return

#         self.current_pose = None  # (x, y, yaw) ของหุ่นยนต์จาก ground truth
#         self.target_index = 0    # waypoint ปัจจุบัน

#         # กำหนดพารามิเตอร์ PID สำหรับการควบคุมการเลี้ยว (steering)
#         self.declare_parameter('kp', 0.25)
#         self.declare_parameter('ki', 0.0)
#         self.declare_parameter('kd', 0.09)
#         self.kp = self.get_parameter('kp').value
#         self.ki = self.get_parameter('ki').value
#         self.kd = self.get_parameter('kd').value

#         # กำหนด tolerance สำหรับมุมและระยะ
#         self.declare_parameter('yaw_tolerance', 0.25)         # ถ้า error < 0.05 rad ไม่แก้ไขมาก
#         self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

#         self.declare_parameter('distance_tolerance', 1.0)       # ถ้าใกล้ waypoint ให้เปลี่ยน waypoint
#         self.distance_tolerance = self.get_parameter('distance_tolerance').value

#         # กำหนดความเร็วขาไป (forward velocity)
#         self.declare_parameter('forward_velocity', 0.5)
#         self.forward_velocity = self.get_parameter('forward_velocity').value

#         # ตัวแปรสำหรับเก็บค่าพิกัด PID
#         self.prev_error = 0.0
#         self.integral = 0.0

#         # สร้าง subscriber รับข้อมูล odometry
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
#         # สร้าง publisher ส่งคำสั่งความเร็ว (cmd_vel) ไปยัง NoSlipModel
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Timer เรียกใช้ control loop ทุก 0.1 วินาที
#         self.timer = self.create_timer(0.1, self.control_loop)

#     def odom_callback(self, msg):
#         """ แปลง quaternion จาก odometry เป็น yaw แล้วเก็บตำแหน่งปัจจุบัน """
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         orientation_q = msg.pose.pose.orientation
#         _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
#         self.current_pose = (x, y, yaw)

#     def normalize_angle(self, angle):
#         """ ทำให้มุมอยู่ในช่วง -π ถึง π """
#         return math.atan2(math.sin(angle), math.cos(angle))

#     def control_loop(self):
#         """ ควบคุมหุ่นยนต์ให้เลี้ยวตาม waypoint โดยใช้ PID จากตำแหน่ง x, y """
#         # ตรวจสอบว่ามีข้อมูลตำแหน่งและ waypoint อยู่หรือไม่
#         if self.current_pose is None or self.target_index >= len(self.path):
#             self.get_logger().info("จบการติดตามเส้นทาง")
#             return

#         # รับ waypoint ปัจจุบันจาก path.yaml
#         target = self.path[self.target_index]
#         target_x = target['x']
#         target_y = target['y']
#         # คำนวณ desired_yaw จากตำแหน่ง target x,y กับตำแหน่งปัจจุบัน
#         desired_yaw = math.atan2(target_y - self.current_pose[1], target_x - self.current_pose[0])
        
#         # คำนวณระยะห่างระหว่างตำแหน่งปัจจุบันกับ waypoint
#         error_x = target_x - self.current_pose[0]
#         error_y = target_y - self.current_pose[1]
#         distance_error = math.sqrt(error_x**2 + error_y**2)
        
#         # คำนวณ yaw error ระหว่าง desired_yaw กับ current yaw
#         yaw_error = self.normalize_angle(desired_yaw - self.current_pose[2])

#         # ถ้าความคลาดเคลื่อนมุมน้อยกว่า tolerance ให้ control output เป็น 0
#         if abs(yaw_error) < self.yaw_tolerance:
#             control_output = 0.0
#         else:
#             # คำนวณ PID สำหรับการเลี้ยว
#             self.integral += yaw_error * 0.1   # dt = 0.1 วินาที
#             derivative = (yaw_error - self.prev_error) / 0.1
#             control_output = self.kp * yaw_error + self.ki * self.integral + self.kd * derivative
#         self.prev_error = yaw_error

#         # สร้างข้อความคำสั่ง Twist สำหรับควบคุมหุ่นยนต์
#         cmd = Twist()
#         # ถ้ายังไม่ถึง waypoint ให้ขับไปด้วยความเร็วที่กำหนด
#         cmd.linear.x = self.forward_velocity if distance_error > self.distance_tolerance else 0.0
#         cmd.angular.z = control_output

#         self.cmd_pub.publish(cmd)

#         self.get_logger().info(
#             f"WP{self.target_index+1}/{len(self.path)}: Target (x,y)=({target_x:.2f}, {target_y:.2f}), Desired yaw={desired_yaw:.2f}, " +
#             f"Current yaw={self.current_pose[2]:.2f}, Yaw error={yaw_error:.2f}, Dist error={distance_error:.2f}, Control={control_output:.2f}"
#         )

#         # ถ้าใกล้ waypoint (ระยะห่างน้อยกว่า tolerance) ให้เปลี่ยนไปยัง waypoint ถัดไป
#         if distance_error < self.distance_tolerance:
#             self.get_logger().info(f"ถึง waypoint {self.target_index+1}/{len(self.path)}")
#             self.target_index += 1
#             # รีเซ็ตค่า PID สำหรับ waypoint ถัดไป
#             self.integral = 0.0
#             self.prev_error = 0.0

# def main(args=None):
#     rclpy.init(args=args)
#     node = PIDPathTracking()
#     if node.path:
#         rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#################################################################################################################################################

# ###PP
# import rclpy
# from rclpy.node import Node
# import yaml
# import numpy as np
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
# import math
# import os
# from ament_index_python.packages import get_package_share_directory

# class AckermanPurePursuit(Node):
#     def __init__(self):
#         super().__init__('ackerman_pure_pursuit')

#         # Load path.yaml from package
#         package_name = 'lab1_1'  # Change this to match your package name
#         path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')

#         if not os.path.exists(path_file):
#             self.get_logger().error(f"Path file not found: {path_file}")
#             return

#         with open(path_file, 'r') as file:
#             self.path = yaml.safe_load(file)

#         if not self.path:
#             self.get_logger().error("Path file is empty or incorrectly formatted.")
#             return

#         self.current_pose = None
#         self.target_index = 0

#         # Pure Pursuit Parameters
#         self.lookahead_distance = 25.0  # Increase to smooth turns
#         self.wheel_base = 0.2  # Distance between front and rear wheels

#         # PID gains for speed control (optional)
#         self.kp_speed = 2.0
#         self.max_speed = 2.0
#         self.min_speed = 0.5

#         # Subscribers and Publishers
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Timer
#         self.timer = self.create_timer(0.1, self.control_loop)

#     def odom_callback(self, msg):
#         """ Converts Odometry quaternion to yaw angle """
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         orientation_q = msg.pose.pose.orientation
#         _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

#         self.current_pose = (x, y, yaw)

#     def normalize_angle(self, angle):
#         """ Normalize angle between -π and π """
#         return math.atan2(math.sin(angle), math.cos(angle))

#     def find_lookahead_point(self):
#         """ Find the point ahead of the current position to follow """
#         if self.current_pose is None:
#             return None

#         x, y, _ = self.current_pose
#         closest_index = 0
#         min_dist = float('inf')

#         # Find the closest point in the path
#         for i, point in enumerate(self.path):
#             px, py = point['x'], point['y']
#             dist = math.sqrt((px - x)**2 + (py - y)**2)
#             if dist < min_dist:
#                 min_dist = dist
#                 closest_index = i

#         # Look ahead further in the path
#         lookahead_index = min(closest_index + 3, len(self.path) - 1)
#         return self.path[lookahead_index]

#     def control_loop(self):
#         """ Pure Pursuit Path Tracking """
#         if self.current_pose is None:
#             return

#         lookahead_point = self.find_lookahead_point()
#         if lookahead_point is None:
#             self.get_logger().info("No more target points. Stopping.")
#             return

#         target_x, target_y = lookahead_point['x'], lookahead_point['y']

#         # Compute the angle between the robot's heading and the target point
#         dx = target_x - self.current_pose[0]
#         dy = target_y - self.current_pose[1]
#         lookahead_dist = math.sqrt(dx**2 + dy**2)

#         # Angle to the lookahead point
#         alpha = self.normalize_angle(math.atan2(dy, dx) - self.current_pose[2])

#         # Compute curvature (γ = 2*sin(α) / Ld)
#         if lookahead_dist > 0:
#             curvature = 2.0 * math.sin(alpha) / lookahead_dist
#         else:
#             curvature = 0.0

#         # Compute steering angle (δ = atan(γ * L))
#         steering_angle = math.atan(curvature * self.wheel_base)

#         # Compute speed (optional PID-based speed control)
#         speed = self.kp_speed * lookahead_dist
#         speed = max(self.min_speed, min(self.max_speed, speed))  # Clamp speed

#         # Publish command
#         velocity = Twist()
#         velocity.linear.x = speed
#         velocity.angular.z = steering_angle  # Set angular velocity

#         self.cmd_pub.publish(velocity)

#         # Check if target is reached
#         if lookahead_dist < self.lookahead_distance:
#             self.target_index += 1
#             self.get_logger().info(f"Reached target {self.target_index}/{len(self.path)}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = AckermanPurePursuit()
#     if node.path:
#         rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# ################################################################################################################################################
##MPC
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import os
from ament_index_python.packages import get_package_share_directory

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # Load path.yaml
        package_name = 'lab1_1'
        path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')
        if not os.path.exists(path_file):
            self.get_logger().error(f"Path file not found: {path_file}")
            return

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        if not self.path:
            self.get_logger().error("Path file is empty or incorrectly formatted.")
            return

        # Convert path to list of waypoints (x, y, yaw)
        self.waypoints = [(wp['x'], wp['y'], wp['yaw']) for wp in self.path]

        # Initialize pose and waypoint index
        self.current_pose = None
        self.current_index = 0

        # Stanley Parameters
        self.stanley_k = 0.8         # 🔥 Increased gain for better turns
        self.wheel_base = 0.35       # Distance between front and rear axles
        self.min_speed = 0.2         # Minimum speed (m/s)
        self.max_speed = 1.2         # Maximum speed (m/s)
        self.forward_speed = 1.0     # Base speed

        # Adaptive steering threshold
        self.adaptive_k_factor = 1.2 # Adaptive gain based on speed

        # Subscribers & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """ Reads odometry and updates current pose (x, y, yaw) """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.current_pose = (x, y, yaw)

    def normalize_angle(self, angle):
        """ Normalize angle to [-π, π] """
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def find_closest_waypoint(self):
        """ Find the closest waypoint to the robot's current position """
        if self.current_pose is None:
            return None, None

        robot_x, robot_y, _ = self.current_pose
        closest_dist = float('inf')
        closest_index = 0

        for i, (px, py, pyaw) in enumerate(self.waypoints):
            dist = math.hypot(px - robot_x, py - robot_y)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        return closest_index, closest_dist

    def control_loop(self):
        """ Stanley Control Loop """
        if self.current_pose is None:
            return

        # Check if waypoints are exhausted
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ All waypoints reached. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        robot_x, robot_y, robot_yaw = self.current_pose

        # Find the closest waypoint
        closest_index, closest_dist = self.find_closest_waypoint()
        if closest_index is None:
            self.get_logger().info("⚠️ No closest waypoint found. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # If the robot is close enough to the waypoint, move to the next one
        if closest_dist < 0.15 and closest_index >= self.current_index:
            self.current_index = min(closest_index + 1, len(self.waypoints) - 1)
            self.get_logger().info(f"📍 Reached waypoint #{closest_index}, moving to next.")

        # Use the reference waypoint
        target_x, target_y, target_yaw = self.waypoints[self.current_index]

        # 1) Compute Heading Error
        heading_error = target_yaw - robot_yaw
        heading_error = self.normalize_angle(heading_error)

        # 2) Compute Cross Track Error (cte)
        path_vec_x = math.cos(target_yaw)
        path_vec_y = math.sin(target_yaw)
        dx = robot_x - target_x
        dy = robot_y - target_y
        cross_product = dx * path_vec_y - dy * path_vec_x  # Cross track error
        cte = cross_product

        # 🔥 **Adaptive Stanley Gain for Turns**
        speed = self.forward_speed
        adaptive_k = self.stanley_k * (1.0 + self.adaptive_k_factor / (speed + 0.1))  # Avoid div by zero

        # **Use atan2 for better CTE handling**
        cte_term = math.atan2(adaptive_k * cte, speed)

        # 3) Apply Stanley Control Law
        steer_angle = 1.2 * heading_error + cte_term  # 🔥 Increase yaw contribution

        # 🔥 **Increase Steering Sensitivity**
        max_steer = 1.57  # 🔥 More aggressive turns (57°)
        steer_angle = max(min(steer_angle, max_steer), -max_steer)

        # 4) Compute Angular Velocity (ω)
        omega = speed * math.tan(steer_angle) / self.wheel_base

        # 🔥 **Dynamically Adjust Speed in Sharp Turns**
        if abs(steer_angle) > 0.5:
            speed *= 0.4  # Reduce speed significantly for sharper turns
        elif abs(steer_angle) > 0.3:
            speed *= 0.7  # Reduce speed moderately for medium turns

        # Debug log
        self.get_logger().info(f"[Stanley] Index={self.current_index}, CTE={cte:.3f}, HeadingErr={heading_error:.3f}, Steer={steer_angle:.3f}, Omega={omega:.3f}")

        # Publish control command
        self.publish_cmd_vel(speed, omega)

    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
