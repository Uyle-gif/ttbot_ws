import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

class PathTrackingEval(Node):
    def __init__(self):
        super().__init__('path_tracking_eval')
        
        # 1. Lấy dữ liệu VỊ TRÍ THỰC (Actual Trajectory)
        # Sử dụng Odom đã lọc để có vị trí mượt nhất
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered', 
            self.odom_callback,
            10)

        # 2. Lấy dữ liệu QUỸ ĐẠO MONG MUỐN (Planned Path)
        # Topic này chứa danh sách các điểm mà MPC tính toán
        self.sub_path = self.create_subscription(
            Path,
            '/mpc_path',
            self.path_callback,
            10)

        # Dữ liệu lưu trữ
        self.robot_x = [] # Lưu lịch sử đi của robot
        self.robot_y = []
        
        self.path_x = []  # Lưu đường MPC hiện tại
        self.path_y = []

    def odom_callback(self, msg):
        # Lưu lại vết xe đi
        self.robot_x.append(msg.pose.pose.position.x)
        self.robot_y.append(msg.pose.pose.position.y)

    def path_callback(self, msg):
        # MPC Path là một danh sách các điểm (Poses)
        # Chúng ta cần tách nó ra để vẽ
        temp_x = []
        temp_y = []
        for pose_stamped in msg.poses:
            temp_x.append(pose_stamped.pose.position.x)
            temp_y.append(pose_stamped.pose.position.y)
        
        # Cập nhật đường dẫn mới nhất từ thuật toán
        self.path_x = temp_x
        self.path_y = temp_y

def update_plot(frame, node, line_robot, line_path, title_text):
    # Vẽ lịch sử đường đi của robot (Nét liền đỏ)
    line_robot.set_data(node.robot_x, node.robot_y)
    
    # Vẽ đường MPC mong muốn (Nét đứt xanh lá)
    # Vì MPC cập nhật liên tục, đường này sẽ thay đổi theo thời gian thực
    line_path.set_data(node.path_x, node.path_y)
    
    # Tự động căn chỉnh khung hình
    if node.robot_x and node.path_x:
        all_x = node.robot_x + node.path_x
        all_y = node.robot_y + node.path_y
        plt.xlim(min(all_x)-1, max(all_x)+1)
        plt.ylim(min(all_y)-1, max(all_y)+1)
        
        # Tính khoảng cách đơn giản giữa đầu xe và điểm đầu của Path (Manhattan distance cho nhanh)
        # Để ước lượng độ lệch
        if len(node.path_x) > 0:
            dx = node.robot_x[-1] - node.path_x[0]
            dy = node.robot_y[-1] - node.path_y[0]
            err = np.sqrt(dx**2 + dy**2)
            title_text.set_text(f"Tracking Eval - Sai số dẫn đường: {err:.3f} m")

    return line_robot, line_path, title_text

def main():
    rclpy.init()
    node = PathTrackingEval()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    ax.axis('equal')

    # Setup 2 đường
    line_path, = ax.plot([], [], 'g--', linewidth=2, label='MPC Planned Path (Tham chiếu)')
    line_robot, = ax.plot([], [], 'r-', linewidth=2, label='Robot Actual Trace (Thực tế)')
    
    title_text = ax.set_title("Đang chờ dữ liệu MPC...")
    ax.legend()

    ani = FuncAnimation(fig, update_plot, fargs=(node, line_robot, line_path, title_text), interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()