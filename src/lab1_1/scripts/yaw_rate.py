# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Header
# import math
# import time
# import tf_transformations

# class YawRateOdom(Node):
#     def __init__(self):
#         super().__init__('yaw_rate_odom_node')

#         # ประกาศพารามิเตอร์เริ่มต้น
#         self.declare_parameter('wheel_base', 0.35)     # ระยะฐานล้อ (หน้า-หลัง)
#         self.declare_parameter('track_width', 0.2)    # ระยะห่างล้อซ้าย-ขวา
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)   # Hz

#         # อ่านค่าพารามิเตอร์
#         self.wheel_base = self.get_parameter('wheel_base').value
#         self.track_width = self.get_parameter('track_width').value
#         self.odom_frame_id = self.get_parameter('odom_frame_id').value
#         self.child_frame_id = self.get_parameter('child_frame_id').value
#         self.publish_rate = self.get_parameter('publish_rate').value

#         # ตำแหน่งและ orientation เริ่มต้น
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0

#         # ความเร็วเชิงเส้นและเชิงมุม (อัปเดตจาก /cmd_vel)
#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0

#         # Timestamp ล่าสุดที่ได้รับ cmd_vel
#         self.last_time = self.get_clock().now()

#         # Subscriber รับ /cmd_vel
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

#         # Publisher ส่ง odometry
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # สร้าง timer เพื่อคำนวณ odom เป็นระยะ
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info('YawRateOdom node started!')

#     def cmd_vel_callback(self, msg: Twist):
#         """
#         รับค่าความเร็วเชิงเส้นและเชิงมุมจาก /cmd_vel
#         """
#         self.linear_velocity = msg.linear.x
#         self.angular_velocity = msg.angular.z

#     def update_odometry(self):
#         """
#         คำนวณตำแหน่ง x, y, yaw จาก linear_velocity, angular_velocity แล้ว publish Odometry
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9  # แปลง nanosec -> sec
#         self.last_time = current_time

#         # -- 1) คำนวณตำแหน่งใหม่ด้วย Forward Kinematics (No-Slip Model) --
#         # x_dot = v * cos(yaw)
#         # y_dot = v * sin(yaw)
#         # yaw_dot = w
#         self.x += self.linear_velocity * math.cos(self.yaw) * dt
#         self.y += self.linear_velocity * math.sin(self.yaw) * dt
#         self.yaw += self.angular_velocity * dt

#         # ปรับ yaw ให้อยู่ในช่วง -pi ถึง pi (ถ้าต้องการ)
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # -- 2) สร้างข้อความ Odometry --
#         odom_msg = Odometry()
#         odom_msg.header = Header()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id     # ปกติ = 'odom'
#         odom_msg.child_frame_id = self.child_frame_id     # ปกติ = 'base_link'

#         # ตำแหน่ง (x, y)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # สร้าง quaternion จาก yaw
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # กำหนด twist (v, w) ใน odom
#         odom_msg.twist.twist.linear.x = self.linear_velocity
#         odom_msg.twist.twist.angular.z = self.angular_velocity

#         # -- 3) Publish odometry --
#         self.odom_pub.publish(odom_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = YawRateOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
############################################################################################################################################################
#!/usr/bin/env python3
####single track
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from std_msgs.msg import Header
# import math
# import tf_transformations


# class YawRateOdom(Node):
#     def __init__(self):
#         super().__init__('yaw_rate_odom_node')

#         # Declare parameters
#         self.declare_parameter('wheel_base', 0.35)     # Distance between front & rear axles
#         self.declare_parameter('track_width', 0.2)    # Distance between left and right wheels
#         self.declare_parameter('wheel_radius', 0.045) # Wheel radius
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)  # Hz

#         # Load parameters
#         self.wheel_base = self.get_parameter('wheel_base').value
#         self.track_width = self.get_parameter('track_width').value
#         self.wheel_radius = self.get_parameter('wheel_radius').value
#         self.odom_frame_id = self.get_parameter('odom_frame_id').value
#         self.child_frame_id = self.get_parameter('child_frame_id').value
#         self.publish_rate = self.get_parameter('publish_rate').value

#         # Initialize odometry variables
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0

#         # Steering and wheel speed state
#         self.steering_angle = 0.0
#         self.wheel_speed = 0.0

#         self.last_time = self.get_clock().now()

#         # Subscribe to /joint_states
#         self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

#         # Publish odometry
#         self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

#         # Timer for odometry updates
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info("YawRateOdom node started!")

#     def joint_states_callback(self, msg: JointState):
#         """
#         Reads joint states for steering angle and wheel speed.
#         """
#         try:
#             # Find indices of the joints in the received message
#             idx_fl_steer = msg.name.index("steering_joint_front_left")
#             idx_fr_steer = msg.name.index("steering_joint_front_right")
#             idx_fl_wheel = msg.name.index("rotation_joint_front_left")
#             idx_fr_wheel = msg.name.index("rotation_joint_front_right")

#             # Compute the **average steering angle** (Single-Track Model assumption)
#             steer_left = msg.position[idx_fl_steer]
#             steer_right = msg.position[idx_fr_steer]
#             self.steering_angle = (steer_left + steer_right) / 2.0  # Use average steering

#             # Compute the **average wheel speed**
#             speed_left = msg.velocity[idx_fl_wheel] * self.wheel_radius  # m/s
#             speed_right = msg.velocity[idx_fr_wheel] * self.wheel_radius  # m/s
#             self.wheel_speed = (speed_left + (-speed_right)) / 2.0  # Average speed

#             # Compute yaw rate from Ackermann kinematics
#             self.angular_velocity = self.compute_yaw_rate()

#         except ValueError as e:
#             self.get_logger().warn("Joint names mismatch: " + str(e))

#     def compute_yaw_rate(self):
#         """
#         Computes yaw rate using Ackermann Single-Track Model.
#         """
#         if abs(self.steering_angle) < 1e-6:
#             return 0.0  # Moving straight

#         # Compute turning radius
#         turning_radius = self.wheel_base / math.tan(self.steering_angle)
#         yaw_rate = self.wheel_speed / turning_radius

#         return yaw_rate

#     def update_odometry(self):
#         """
#         Updates odometry and publishes it.
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # Update position and yaw using integration
#         self.x += self.wheel_speed * math.cos(self.yaw) * dt
#         self.y += self.wheel_speed * math.sin(self.yaw) * dt
#         self.yaw += self.angular_velocity * dt

#         # Normalize yaw to [-pi, pi]
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # Create and publish odometry message
#         odom_msg = Odometry()
#         odom_msg.header = Header()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id
#         odom_msg.child_frame_id = self.child_frame_id

#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         odom_msg.twist.twist.linear.x = self.wheel_speed
#         odom_msg.twist.twist.angular.z = self.angular_velocity

#         self.odom_pub.publish(odom_msg)

#         self.get_logger().info(
#             f"Odom -> x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f} rad, v: {self.wheel_speed:.2f} m/s, ω: {self.angular_velocity:.2f} rad/s"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = YawRateOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#########################################################################################################################################################
###Yaw rate
<<<<<<< HEAD
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Header
# import math
# import tf_transformations

# class AckermannOdom(Node):
#     def __init__(self):
#         super().__init__('ackermann_odom_node')

#         # ประกาศพารามิเตอร์เริ่มต้น
#         self.declare_parameter('wheel_base', 0.35)      # ระยะฐานล้อ (หน้า-หลัง)
#         self.declare_parameter('track_width', 0.2)     # ระยะห่างล้อซ้าย-ขวา
#         self.declare_parameter('wheel_radius', 0.045)  # รัศมีล้อ
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)   # Hz

#         # อ่านค่าพารามิเตอร์
#         self.wheel_base = self.get_parameter('wheel_base').value
#         self.track_width = self.get_parameter('track_width').value
#         self.wheel_radius = self.get_parameter('wheel_radius').value
#         self.odom_frame_id = self.get_parameter('odom_frame_id').value
#         self.child_frame_id = self.get_parameter('child_frame_id').value
#         self.publish_rate = self.get_parameter('publish_rate').value

#         # ตำแหน่งและ orientation เริ่มต้น
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0

#         # เก็บค่าตำแหน่ง (มุม) และอัตราหมุนของ joint ต่าง ๆ
#         self.joint_positions = {}
#         self.joint_velocities = {}

#         # Timestamp ล่าสุด
#         self.last_time = self.get_clock().now()

#         # Subscriber รับค่าจาก /joint_states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         # Publisher ส่ง odometry
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # Timer สำหรับคำนวณ odometry
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info("AckermannOdom node started")

#     def joint_state_callback(self, msg: JointState):
#         """
#         อ่านค่าจาก /joint_states แล้วเก็บลง dict
#         ชื่อ joint ใน msg.name ต้องตรงกับใน URDF 
#         เช่น steering_joint_front_left, rotation_joint_front_left
#         """
#         for i, name in enumerate(msg.name):
#             position = msg.position[i] if len(msg.position) > i else 0.0
#             velocity = 0.0
#             if msg.velocity and len(msg.velocity) > i:
#                 velocity = msg.velocity[i]

#             self.joint_positions[name] = position
#             self.joint_velocities[name] = velocity

#     def update_odometry(self):
#         """
#         คำนวณ (x, y, yaw) จาก No-Slip Ackermann Model อย่างง่าย
#         แล้ว Publish Odometry ที่ /odom
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # 1) อ่านมุมพวงมาลัยจาก joint state
#         steering_left = self.joint_positions.get('steering_joint_front_left', 0.0)
#         steering_right = self.joint_positions.get('steering_joint_front_right', 0.0)

#         # สมมติใช้ค่าเฉลี่ยของซ้าย-ขวาเป็นมุมพวงมาลัยรวม
#         steer_angle = (steering_left + steering_right) / 2.0

#         # 2) อ่านอัตราหมุนล้อหน้าซ้าย-ขวา (rad/s)
#         v_left = self.joint_velocities.get('rotation_joint_front_left', 0.0)
#         v_right = -self.joint_velocities.get('rotation_joint_front_right', 0.0)

#         # 3) แปลงอัตราหมุนล้อ (rad/s) เป็นความเร็วเชิงเส้น (m/s)
#         # สมมติหุ่นใช้ล้อหน้าขับเคลื่อน
#         v_forward = ((v_left + v_right) / 2.0) * self.wheel_radius

#         # 4) คำนวณ yaw_rate จาก Ackermann geometry อย่างง่าย
#         #    yaw_rate = (v_forward / wheel_base) * tan(steer_angle)
#         yaw_rate = 0.0
#         if abs(steer_angle) > 1e-6:
#             yaw_rate = (v_forward / self.wheel_base) * math.tan(steer_angle)

#         # 5) อัปเดตตำแหน่ง (x, y) และ yaw
#         self.x += v_forward * math.cos(self.yaw) * dt
#         self.y += v_forward * math.sin(self.yaw) * dt
#         self.yaw += yaw_rate * dt

#         # ทำ yaw ให้อยู่ในช่วง [-pi, pi] (ถ้าต้องการ)
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # 6) สร้างข้อความ Odometry
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id   # ex: "odom"
#         odom_msg.child_frame_id = self.child_frame_id   # ex: "base_link"

#         # ตำแหน่ง (x, y)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Orientation (Quaternion)
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Twist
#         odom_msg.twist.twist.linear.x = v_forward
#         odom_msg.twist.twist.angular.z = yaw_rate

#         # 7) Publish odometry
#         self.odom_pub.publish(odom_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = AckermannOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#########################################################################################################################################
##double track model
=======
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import tf_transformations

<<<<<<< HEAD
class DoubleTrackOdom(Node):
    def __init__(self):
        super().__init__('double_track_odom_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.045)    # Wheel radius (m)
        self.declare_parameter('track_width', 0.2)       # Distance between left and right wheels (m)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)     # Hz (Publishing Rate)

        # Load parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
=======
class AckermannOdom(Node):
    def __init__(self):
        super().__init__('ackermann_odom_node')

        # ประกาศพารามิเตอร์เริ่มต้น
        self.declare_parameter('wheel_base', 0.35)      # ระยะฐานล้อ (หน้า-หลัง)
        self.declare_parameter('track_width', 0.2)     # ระยะห่างล้อซ้าย-ขวา
        self.declare_parameter('wheel_radius', 0.045)  # รัศมีล้อ
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)   # Hz

        # อ่านค่าพารามิเตอร์
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

<<<<<<< HEAD
        # Initialize position and yaw
=======
        # ตำแหน่งและ orientation เริ่มต้น
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

<<<<<<< HEAD
        # Initialize velocities
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0 # rad/s

        # Initialize wheel speeds
        self.left_wheel_speed = 0.0   # rad/s or m/s
        self.right_wheel_speed = 0.0  # rad/s or m/s

        # Time tracking
        self.last_time = self.get_clock().now()

        # Subscribe to /joint_states
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to update odometry
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('DoubleTrackOdom node started!')

    def joint_states_callback(self, msg: JointState):
        """
        Reads joint velocities from /joint_states.
        Uses "rotation_joint_front_left" and "rotation_joint_front_right".
        """
        try:
            idx_left = msg.name.index("rotation_joint_front_left")
            idx_right = msg.name.index("rotation_joint_front_right")

            # Read joint velocities (Check if they are in rad/s or m/s)
            self.left_wheel_speed = msg.velocity[idx_left]  
            self.right_wheel_speed = -msg.velocity[idx_right]  # **FIXED: Removed unnecessary "-" sign**

            # Compute linear velocity (v) in **m/s**
            self.linear_velocity = ((self.left_wheel_speed + self.right_wheel_speed) / 2.0) * self.wheel_radius

            # Compute angular velocity (ω) in **rad/s**
            self.angular_velocity = (self.right_wheel_speed - self.left_wheel_speed) / self.track_width

        except ValueError as e:
            self.get_logger().warn("Missing joint in /joint_states: " + str(e))

    def update_odometry(self):
        """
        Updates odometry using integrated position (x, y, yaw).
        Publishes to /odom.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # **Exponential Moving Average (EMA) for smoothing**
        alpha = 0.25
        self.linear_velocity = alpha * self.linear_velocity + (1 - alpha) * self.linear_velocity
        self.angular_velocity = alpha * self.angular_velocity + (1 - alpha) * self.angular_velocity

        # **Update Position & Orientation**
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Normalize yaw

        # **Create Odometry Message**
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id

=======
        # เก็บค่าตำแหน่ง (มุม) และอัตราหมุนของ joint ต่าง ๆ
        self.joint_positions = {}
        self.joint_velocities = {}

        # Timestamp ล่าสุด
        self.last_time = self.get_clock().now()

        # Subscriber รับค่าจาก /joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher ส่ง odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer สำหรับคำนวณ odometry
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info("AckermannOdom node started")

    def joint_state_callback(self, msg: JointState):
        """
        อ่านค่าจาก /joint_states แล้วเก็บลง dict
        ชื่อ joint ใน msg.name ต้องตรงกับใน URDF 
        เช่น steering_joint_front_left, rotation_joint_front_left
        """
        for i, name in enumerate(msg.name):
            position = msg.position[i] if len(msg.position) > i else 0.0
            velocity = 0.0
            if msg.velocity and len(msg.velocity) > i:
                velocity = msg.velocity[i]

            self.joint_positions[name] = position
            self.joint_velocities[name] = velocity

    def update_odometry(self):
        """
        คำนวณ (x, y, yaw) จาก No-Slip Ackermann Model อย่างง่าย
        แล้ว Publish Odometry ที่ /odom
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # 1) อ่านมุมพวงมาลัยจาก joint state
        steering_left = self.joint_positions.get('steering_joint_front_left', 0.0)
        steering_right = self.joint_positions.get('steering_joint_front_right', 0.0)

        # สมมติใช้ค่าเฉลี่ยของซ้าย-ขวาเป็นมุมพวงมาลัยรวม
        steer_angle = (steering_left + steering_right) / 2.0

        # 2) อ่านอัตราหมุนล้อหน้าซ้าย-ขวา (rad/s)
        v_left = self.joint_velocities.get('rotation_joint_front_left', 0.0)
        v_right = -self.joint_velocities.get('rotation_joint_front_right', 0.0)

        # 3) แปลงอัตราหมุนล้อ (rad/s) เป็นความเร็วเชิงเส้น (m/s)
        # สมมติหุ่นใช้ล้อหน้าขับเคลื่อน
        v_forward = ((v_left + v_right) / 2.0) * self.wheel_radius

        # 4) คำนวณ yaw_rate จาก Ackermann geometry อย่างง่าย
        #    yaw_rate = (v_forward / wheel_base) * tan(steer_angle)
        yaw_rate = 0.0
        if abs(steer_angle) > 1e-6:
            yaw_rate = (v_forward / self.wheel_base) * math.tan(steer_angle)

        # 5) อัปเดตตำแหน่ง (x, y) และ yaw
        self.x += v_forward * math.cos(self.yaw) * dt
        self.y += v_forward * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # ทำ yaw ให้อยู่ในช่วง [-pi, pi] (ถ้าต้องการ)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 6) สร้างข้อความ Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id   # ex: "odom"
        odom_msg.child_frame_id = self.child_frame_id   # ex: "base_link"

        # ตำแหน่ง (x, y)
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

<<<<<<< HEAD
        # Convert yaw to quaternion
=======
        # Orientation (Quaternion)
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

<<<<<<< HEAD
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom_msg)

        self.get_logger().info(
            f"Odom -> x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f} rad, v: {self.linear_velocity:.2f} m/s, ω: {self.angular_velocity:.2f} rad/s"
        )
=======
        # Twist
        odom_msg.twist.twist.linear.x = v_forward
        odom_msg.twist.twist.angular.z = yaw_rate

        # 7) Publish odometry
        self.odom_pub.publish(odom_msg)
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9


def main(args=None):
    rclpy.init(args=args)
<<<<<<< HEAD
    node = DoubleTrackOdom()
=======
    node = AckermannOdom()
>>>>>>> 42f94861bd9f8b15264d99aa16409916fab4b9f9
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#########################################################################################################################################
##double track model
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Header
# import math
# import tf_transformations

# class DoubleTrackOdom(Node):
#     def __init__(self):
#         super().__init__('double_track_odom_node')

#         # Declare parameters
#         self.declare_parameter('wheel_radius', 0.045)    # Wheel radius (m)
#         self.declare_parameter('track_width', 0.2)       # Distance between left and right wheels (m)
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)     # Hz (Publishing Rate)

#         # Load parameters
#         self.wheel_radius = self.get_parameter('wheel_radius').value
#         self.track_width = self.get_parameter('track_width').value
#         self.odom_frame_id = self.get_parameter('odom_frame_id').value
#         self.child_frame_id = self.get_parameter('child_frame_id').value
#         self.publish_rate = self.get_parameter('publish_rate').value

#         # Initialize position and yaw
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0

#         # Initialize velocities
#         self.linear_velocity = 0.0  # m/s
#         self.angular_velocity = 0.0 # rad/s

#         # Initialize wheel speeds
#         self.left_wheel_speed = 0.0   # rad/s or m/s
#         self.right_wheel_speed = 0.0  # rad/s or m/s

#         # Time tracking
#         self.last_time = self.get_clock().now()

#         # Subscribe to /joint_states
#         self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

#         # Publisher for /odom
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # Timer to update odometry
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info('DoubleTrackOdom node started!')

#     def joint_states_callback(self, msg: JointState):
#         """
#         Reads joint velocities from /joint_states.
#         Uses "rotation_joint_front_left" and "rotation_joint_front_right".
#         """
#         try:
#             idx_left = msg.name.index("rotation_joint_front_left")
#             idx_right = msg.name.index("rotation_joint_front_right")

#             # Read joint velocities (Check if they are in rad/s or m/s)
#             self.left_wheel_speed = msg.velocity[idx_left]  
#             self.right_wheel_speed = -msg.velocity[idx_right]  # **FIXED: Removed unnecessary "-" sign**

#             # Compute linear velocity (v) in **m/s**
#             self.linear_velocity = ((self.left_wheel_speed + self.right_wheel_speed) / 2.0) * self.wheel_radius

#             # Compute angular velocity (ω) in **rad/s**
#             self.angular_velocity = (self.right_wheel_speed - self.left_wheel_speed) / self.track_width

#         except ValueError as e:
#             self.get_logger().warn("Missing joint in /joint_states: " + str(e))

#     def update_odometry(self):
#         """
#         Updates odometry using integrated position (x, y, yaw).
#         Publishes to /odom.
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
#         self.last_time = current_time

#         # **Exponential Moving Average (EMA) for smoothing**
#         alpha = 0.25
#         self.linear_velocity = alpha * self.linear_velocity + (1 - alpha) * self.linear_velocity
#         self.angular_velocity = alpha * self.angular_velocity + (1 - alpha) * self.angular_velocity

#         # **Update Position & Orientation**
#         self.x += self.linear_velocity * math.cos(self.yaw) * dt
#         self.y += self.linear_velocity * math.sin(self.yaw) * dt
#         self.yaw += self.angular_velocity * dt
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Normalize yaw

#         # **Create Odometry Message**
#         odom_msg = Odometry()
#         odom_msg.header = Header()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id
#         odom_msg.child_frame_id = self.child_frame_id

#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Convert yaw to quaternion
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         odom_msg.twist.twist.linear.x = self.linear_velocity
#         odom_msg.twist.twist.angular.z = self.angular_velocity

#         self.odom_pub.publish(odom_msg)

#         self.get_logger().info(
#             f"Odom -> x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f} rad, v: {self.linear_velocity:.2f} m/s, ω: {self.angular_velocity:.2f} rad/s"
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = DoubleTrackOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
