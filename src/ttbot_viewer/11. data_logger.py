#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import csv
import argparse

class PathDataLogger(Node):
    def __init__(self, prefix: str, odom_topic: str, path_topic: str):
        super().__init__('path_data_logger')
        self.prefix = prefix
        
        self.robot_data = []
        self.path_xy = []
        self.path_received = False
        
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        
        # Đưa về cấu hình số 10 mặc định, hết báo lỗi QoS
        self.sub_path = self.create_subscription(
            Path, path_topic, self.path_callback, 10
        )
        
        self.get_logger().info(f"Logging data from {path_topic} and {odom_topic}...")
        self.get_logger().info("Press Ctrl+C to stop and save CSV files.")

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_data.append((t, x, y))

    def path_callback(self, msg: Path):
        self.path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self.path_received and self.path_xy:
            self.path_received = True
            self.get_logger().info(f"Received path: {len(self.path_xy)} points")

    def save_results(self):
        traj_file = f"{self.prefix}_traj.csv"
        ref_file = f"{self.prefix}_ref.csv"
        
        with open(traj_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "x", "y"])
            writer.writerows(self.robot_data)
            
        with open(ref_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(self.path_xy)
            
        self.get_logger().info(f"Saved: {traj_file}")
        self.get_logger().info(f"Saved: {ref_file}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=str, default="mpc")
    parser.add_argument("--odom_topic", type=str, default="/mpc_state")
    parser.add_argument("--path_topic", type=str, default="/mpc_path")
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = PathDataLogger(args.prefix, args.odom_topic, args.path_topic)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()