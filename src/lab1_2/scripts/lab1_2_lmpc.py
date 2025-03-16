# #!/usr/bin/env python3
#########################################################################################################################################################
##Yaw rate
#!/usr/bin/env python3
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

#         # Declare parameters
#         self.declare_parameter('wheel_base', 0.35)      # Distance between front & rear axles
#         self.declare_parameter('track_width', 0.2)     # Distance between left & right wheels
#         self.declare_parameter('wheel_radius', 0.045)  # Wheel radius
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)   # Hz

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
        

#         # Joint state storage
#         self.joint_positions = {}
#         self.joint_velocities = {}

#         # Timestamp tracking
#         self.last_time = self.get_clock().now()

#         # Subscribe to /joint_states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         # Publish odometry
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # Timer to update odometry
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info("AckermannOdom node started!")

#     def joint_state_callback(self, msg: JointState):
#         """
#         Reads joint states from /joint_states and stores them.
#         """
#         for i, name in enumerate(msg.name):
#             position = msg.position[i] if len(msg.position) > i else 0.0
#             velocity = msg.velocity[i] if len(msg.velocity) > i else 0.0

#             self.joint_positions[name] = position
#             self.joint_velocities[name] = velocity

#     def update_odometry(self):
#         """
#         Computes (x, y, yaw) using Ackermann Model and updates odometry.
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # 1) Read front wheel steering angles
#         steering_left = self.joint_positions.get('steering_joint_front_left', 0.0)
#         steering_right = self.joint_positions.get('steering_joint_front_right', 0.0)

#         # Compute average steering angle
#         if abs(steering_left - steering_right) > 1e-3:
#             self.get_logger().warn("⚠️ Left & Right Steering Angles Differ! Averaging them.")
#         steer_angle = (steering_left + steering_right) / 2.0

#         # 2) Read wheel rotational velocities
#         v_left = self.joint_velocities.get('rotation_joint_front_left', 0.0)
#         v_right = -self.joint_velocities.get('rotation_joint_front_right', 0.0)

#         # Convert wheel angular velocity (rad/s) to linear velocity (m/s)
#         v_forward = ((v_left + v_right) / 2.0) * self.wheel_radius

#         # 3) Compute yaw rate from Ackermann steering geometry
#         yaw_rate = 0.0
#         if abs(steer_angle) > 1e-6:
#             turning_radius = self.wheel_base / math.tan(steer_angle)
#             yaw_rate = v_forward / turning_radius

#         # 4) Update (x, y, yaw) position
#         self.x += v_forward * math.cos(self.yaw) * dt
#         self.y += v_forward * math.sin(self.yaw) * dt
#         self.yaw += yaw_rate * dt

#         # Normalize yaw to [-pi, pi]
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # 5) Publish Odometry
#         self.publish_odometry(current_time, v_forward, yaw_rate)

#     def publish_odometry(self, current_time, v_forward, yaw_rate):
#         """
#         Creates and publishes the Odometry message.
#         """
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id
#         odom_msg.child_frame_id = self.child_frame_id

#         # Set position (x, y)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Set orientation (yaw -> quaternion)
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Set velocity (linear & angular)
#         odom_msg.twist.twist.linear.x = v_forward
#         odom_msg.twist.twist.angular.z = yaw_rate

#         # Publish the message
#         self.odom_pub.publish(odom_msg)

#         self.get_logger().info(
#             f"📌 Odom: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad, v={v_forward:.2f} m/s, ω={yaw_rate:.2f} rad/s"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = AckermannOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
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

#         # Declare parameters
#         self.declare_parameter('wheel_base', 0.35)      # Distance between front & rear axles
#         self.declare_parameter('track_width', 0.2)     # Distance between left & right wheels
#         self.declare_parameter('wheel_radius', 0.045)  # Wheel radius
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)   # Hz

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

#         # Joint state storage
#         self.joint_positions = {}
#         self.joint_velocities = {}

#         # Timestamp tracking
#         self.last_time = self.get_clock().now()

#         # Subscribe to /joint_states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         # Publish odometry
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # Timer to update odometry
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info("AckermannOdom node started!")

#     def joint_state_callback(self, msg: JointState):
#         """
#         Reads joint states from /joint_states and stores them.
#         """
#         for i, name in enumerate(msg.name):
#             position = msg.position[i] if len(msg.position) > i else 0.0
#             velocity = msg.velocity[i] if len(msg.velocity) > i else 0.0

#             self.joint_positions[name] = position
#             self.joint_velocities[name] = velocity

#     def update_odometry(self):
#         """
#         Computes (x, y, yaw) using Ackermann Model and updates odometry.
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # 1) Read front wheel steering angles
#         steering_left = self.joint_positions.get('steering_joint_front_left', 0.0)
#         steering_right = self.joint_positions.get('steering_joint_front_right', 0.0)

#         # Compute average steering angle and clamp within [-0.5, 0.5] rad
#         steer_angle = (steering_left + steering_right) / 2.0
#         steer_angle = max(-0.5, min(steer_angle, 0.5))  # Clamping

#         # 2) Read wheel rotational velocities
#         v_left = self.joint_velocities.get('rotation_joint_front_left', 0.0)
#         v_right = -self.joint_velocities.get('rotation_joint_front_right', 0.0)

#         # Convert wheel angular velocity (rad/s) to linear velocity (m/s)
#         v_forward = ((v_left + v_right) / 2.0) * self.wheel_radius

#         # 3) Compute yaw rate from Ackermann steering geometry
#         if abs(math.tan(steer_angle)) < 1e-6:
#             yaw_rate = 0.0  # Moving straight
#         else:
#             turning_radius = self.wheel_base / math.tan(steer_angle)
#             yaw_rate = v_forward / turning_radius

#         # 4) Update (x, y, yaw) position
#         self.x += v_forward * math.cos(self.yaw) * dt
#         self.y += v_forward * math.sin(self.yaw) * dt
#         self.yaw += yaw_rate * dt

#         # Normalize yaw to [-pi, pi]
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # 5) Publish Odometry
#         self.publish_odometry(current_time, v_forward, yaw_rate)

#     def publish_odometry(self, current_time, v_forward, yaw_rate):
#         """
#         Creates and publishes the Odometry message.
#         """
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id
#         odom_msg.child_frame_id = self.child_frame_id

#         # Set position (x, y)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Set orientation (yaw -> quaternion)
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Set velocity (linear & angular)
#         odom_msg.twist.twist.linear.x = v_forward
#         odom_msg.twist.twist.angular.z = yaw_rate

#         # Publish the message
#         self.odom_pub.publish(odom_msg)

#         self.get_logger().info(
#             f"📌 Odom: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad, v={v_forward:.2f} m/s, ω={yaw_rate:.2f} rad/s"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = AckermannOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #########################################################################################################################################
# #!/usr/bin/env python3
# ## double track
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
#         self.declare_parameter('wheel_base', 0.2)      # Distance between front & rear axles
#         self.declare_parameter('track_width', 0.14)     # Distance between left & right rear wheels
#         self.declare_parameter('wheel_radius', 0.045)  # Wheel radius
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('child_frame_id', 'base_link')
#         self.declare_parameter('publish_rate', 50.0)   # Hz

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

#         # Joint state storage
#         self.joint_positions = {}
#         self.joint_velocities = {}

#         # Timestamp tracking
#         self.last_time = self.get_clock().now()

#         # Subscribe to /joint_states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         # Publish odometry
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # Timer to update odometry
#         timer_period = 1.0 / self.publish_rate
#         self.timer = self.create_timer(timer_period, self.update_odometry)

#         self.get_logger().info("DoubleTrackOdom node started!")

#     def joint_state_callback(self, msg: JointState):
#         """
#         Reads joint states from /joint_states and stores them.
#         """
#         for i, name in enumerate(msg.name):
#             position = msg.position[i] if len(msg.position) > i else 0.0
#             velocity = msg.velocity[i] if len(msg.velocity) > i else 0.0

#             self.joint_positions[name] = position
#             self.joint_velocities[name] = velocity

#     def update_odometry(self):
#         """
#         Computes (x, y, yaw) using Double Track Model and updates odometry.
#         """
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         self.last_time = current_time

#         # 1) Read rear wheel velocities
#         v_left = self.joint_velocities.get('joint_wheel_rear_left', 0.0)
#         v_right = -self.joint_velocities.get('joint_wheel_rear_right', 0.0)

#         # Convert wheel angular velocity (rad/s) to linear velocity (m/s)
#         v_left_linear = v_left * self.wheel_radius
#         v_right_linear = v_right * self.wheel_radius

#         # 2) Compute linear velocity as average of rear wheels
#         v_forward = (v_left_linear + v_right_linear) / 2.0

#         # 3) Compute yaw rate from rear wheel speeds
#         yaw_rate = (v_right_linear - v_left_linear) / self.track_width

#         # 4) Update (x, y, yaw) position
#         self.x += v_forward * math.cos(self.yaw) * dt
#         self.y += v_forward * math.sin(self.yaw) * dt
#         self.yaw += yaw_rate * dt

#         # Normalize yaw to [-pi, pi]
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         # 5) Publish Odometry
#         self.publish_odometry(current_time, v_forward, yaw_rate)

#     def publish_odometry(self, current_time, v_forward, yaw_rate):
#         """
#         Creates and publishes the Odometry message.
#         """
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = self.odom_frame_id
#         odom_msg.child_frame_id = self.child_frame_id

#         # Set position (x, y)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Set orientation (yaw -> quaternion)
#         quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Set velocity (linear & angular)
#         odom_msg.twist.twist.linear.x = v_forward
#         odom_msg.twist.twist.angular.z = yaw_rate

#         # Publish the message
#         self.odom_pub.publish(odom_msg)

#         self.get_logger().info(
#             f"📌 Odom: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad, v={v_forward:.2f} m/s, ω={yaw_rate:.2f} rad/s"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = DoubleTrackOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
###################################################################################################################################################
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
from scipy.optimize import minimize

class AckermanMPC(Node):
    def __init__(self):
        super().__init__('ackerman_mpc')

        # Load path.yaml from package
        package_name = 'lab1_1'  # Change this to match your package name
        path_file = os.path.join(get_package_share_directory(package_name), 'config', 'path.yaml')

        if not os.path.exists(path_file):
            self.get_logger().error(f"Path file not found: {path_file}")
            return

        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        if not self.path:
            self.get_logger().error("Path file is empty or incorrectly formatted.")
            return

        self.current_pose = None
        self.target_index = 0

        # MPC Parameters
        self.wheel_base = 0.2  # Distance between front and rear wheels
        self.dt = 0.1  # Time step for prediction
        self.horizon = 5  # Prediction horizon
        self.max_speed =0.5
        self.min_speed = 0.1
        self.max_steering = 0.5  # Steering angle limit (radians)
        self.lookahead_distance = 1.0  # Lookahead distance in meters

        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """Converts Odometry quaternion to yaw angle."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_pose = np.array([x, y, yaw])

    def find_closest_point(self):
        """Finds the index of the closest point in the path to the current position."""
        if self.current_pose is None:
            return None
        distances = [np.linalg.norm([pt['x'] - self.current_pose[0], pt['y'] - self.current_pose[1]]) for pt in self.path]
        return np.argmin(distances)

    def find_lookahead_point(self):
        """Finds the target point index based on lookahead distance."""
        if self.current_pose is None:
            return None

        closest_index = self.find_closest_point()
        lookahead_index = closest_index
        total_distance = 0.0

        # Accumulate distance along the path until lookahead_distance is exceeded
        for i in range(closest_index, len(self.path) - 1):
            pt_current = np.array([self.path[i]['x'], self.path[i]['y']])
            pt_next = np.array([self.path[i+1]['x'], self.path[i+1]['y']])
            segment_distance = np.linalg.norm(pt_next - pt_current)
            total_distance += segment_distance
            if total_distance >= self.lookahead_distance:
                lookahead_index = i + 1
                break
        return lookahead_index

    def mpc_cost_function(self, u, *args):
        """Cost function for MPC optimization."""
        x0, path_segment, dt, horizon, wheel_base = args
        x = np.copy(x0)
        cost = 0.0

        for i in range(horizon):
            v, delta = u[i], u[horizon + i]
            delta = np.clip(delta, -self.max_steering, self.max_steering)

            # Kinematic update (Single Track/Bicycle Model)
            x[0] += v * np.cos(x[2]) * dt
            x[1] += v * np.sin(x[2]) * dt
            x[2] += (v / wheel_base) * np.tan(delta) * dt

            # Tracking error based on target path segment
            path_x, path_y = path_segment[i]['x'], path_segment[i]['y']
            cost += (x[0] - path_x)**2 + (x[1] - path_y)**2  # Position error
            cost += 0.1 * v**2 + 0.1 * delta**2  # Control effort penalty

        return cost

    def control_loop(self):
        """Model Predictive Control (MPC) trajectory tracking with lookahead."""
        if self.current_pose is None:
            return

        # Use lookahead to choose target point
        target_index = self.find_lookahead_point()
        if target_index is None or target_index + self.horizon >= len(self.path):
            self.get_logger().info("No more target points. Stopping.")
            return

        # Get the segment of the path starting from the lookahead target
        path_segment = self.path[target_index:target_index + self.horizon]

        # Define optimization variables: [v1, ..., vh, delta1, ..., deltah]
        u0 = np.zeros(2 * self.horizon)
        bounds = [(self.min_speed, self.max_speed)] * self.horizon + [(-self.max_steering, self.max_steering)] * self.horizon

        # Solve optimization
        res = minimize(self.mpc_cost_function, u0, args=(self.current_pose, path_segment, self.dt, self.horizon, self.wheel_base), bounds=bounds)
        if res.success:
            v_opt, delta_opt = res.x[0], res.x[self.horizon]
        else:
            v_opt, delta_opt = 0.0, 0.0

        # Publish command
        velocity = Twist()
        velocity.linear.x = v_opt
        velocity.angular.z = delta_opt
        self.cmd_pub.publish(velocity)

        self.get_logger().info(f"MPC -> v: {v_opt:.2f} m/s, δ: {delta_opt:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = AckermanMPC()
    if node.path:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
