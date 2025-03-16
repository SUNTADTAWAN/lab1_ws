#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml
import threading
import os
from ament_index_python.packages import get_package_share_directory

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')

        # **Subscribe to Ground Truth Odometry**
        self.create_subscription(Odometry, '/odometry/ground_truth', self.ground_truth_callback, 10)

        # **Load YAML Path**
        self.yaml_x_positions = []
        self.yaml_y_positions = []
        self.load_yaml_path()

        # **Lists for Ground Truth Data**
        self.gt_x_positions = []
        self.gt_y_positions = []

        # **Plot Thread**
        self.plot_thread = threading.Thread(target=self.plot_graph)
        self.plot_thread.start()

    def load_yaml_path(self):
        """Load the YAML file containing path data"""
        try:
            # Find the path of path.yaml
            package_name = "lab1_1"
            file_path = os.path.join(get_package_share_directory(package_name), "config", "path.yaml")

            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
                if data is None or not isinstance(data, list):
                    self.get_logger().error("YAML file is empty or incorrect format.")
                    return

                self.yaml_x_positions = [point['x'] for point in data]
                self.yaml_y_positions = [point['y'] for point in data]
                self.get_logger().info(f"Loaded {len(self.yaml_x_positions)} points from YAML")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")

    def ground_truth_callback(self, msg):
        """Callback function to store ground truth positions"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gt_x_positions.append(x)
        self.gt_y_positions.append(y)
        self.get_logger().info(f'Ground Truth -> X: {x:.2f}, Y: {y:.2f}')

    def plot_graph(self):
        """Plot the YAML path and Ground Truth path"""
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Path Comparison")

        while rclpy.ok():
            ax.clear()
            # Plot YAML Path
            # Plot Ground Truth Path
            ax.plot(self.gt_x_positions, self.gt_y_positions, 'r-', linewidth=2, label="Ground Truth Path")
            if self.yaml_x_positions and self.yaml_y_positions:
                ax.plot(self.yaml_x_positions, self.yaml_y_positions, 'g--', linewidth=0.5, label="Planned Path (YAML)")
            ax.legend()
            plt.pause(0.1)

            # Stop plotting when user presses 'q'
            if plt.waitforbuttonpress(timeout=0.1):
                plt.ioff()
                plt.savefig("path_comparison.png")  # Save plot as image
                self.get_logger().info("Saved plot as 'path_comparison.png'")
                plt.show()
                break

def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
