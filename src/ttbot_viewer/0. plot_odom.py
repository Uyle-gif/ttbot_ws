#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading
import numpy as np

class MPCXYPlotter(Node):
    def __init__(self):
        super().__init__('mpc_xy_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/mpc_state', 
            self.odom_callback,
            10)
        
        self.x_data = []
        self.y_data = []
        self.lock = threading.Lock()
        self.get_logger().info("Node Plotter đã sẵn sàng, đang chờ dữ liệu từ /mpc_state...")

    def odom_callback(self, msg):
        with self.lock:
            self.x_data.append(msg.pose.pose.position.x)
            self.y_data.append(msg.pose.pose.position.y)

def main(args=None):
    rclpy.init(args=args)
    node = MPCXYPlotter()
    
    # Chạy ROS 2 spin trong luồng riêng
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Khởi tạo đồ thị ở luồng chính
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title('Kiểm tra Quỹ đạo MPC State (X-Y)')
    
    # Tạo các đối tượng đồ họa trống để cập nhật nhanh hơn
    line, = ax.plot([], [], 'r-', linewidth=1.5, label='Filtered Path')
    start_point, = ax.plot([], [], 'ko', fillstyle='none', markersize=8, label='Start')
    current_point, = ax.plot([], [], 'kx', markersize=10, markeredgewidth=2, label='Current')

    # Cấu hình ban đầu
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('X (m)')
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_aspect('equal', adjustable='box')
    ax.invert_xaxis() 
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')

    try:
        while rclpy.ok():
            with node.lock:
                if len(node.x_data) > 0:
                    # Cập nhật dữ liệu cho các đường vẽ
                    line.set_data(node.y_data, node.x_data)
                    start_point.set_data([node.y_data[0]], [node.x_data[0]])
                    current_point.set_data([node.y_data[-1]], [node.x_data[-1]])

                    # Tự động nới rộng khung hình nếu xe đi xa hơn 5x5
                    curr_max_x = max(max(node.x_data), 2.5)
                    curr_min_x = min(min(node.x_data), -2.5)
                    curr_max_y = max(max(node.y_data), 2.5)
                    curr_min_y = min(min(node.y_data), -2.5)
                    
                    ax.set_xlim([curr_max_y + 0.5, curr_min_y - 0.5]) # Giữ invert_xaxis
                    ax.set_ylim([curr_min_x - 0.5, curr_max_x + 0.5])

                    # Render lại đồ thị
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                
            plt.pause(0.05) # Quan trọng để Matplotlib xử lý sự kiện GUI
                
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show() # Giữ cửa sổ cuối cùng nếu cần
        rclpy.shutdown()

if __name__ == '__main__':
    main()