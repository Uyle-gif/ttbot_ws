import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class HeadingPlotter(Node):
    def __init__(self):
        super().__init__('heading_plotter_node')
        
        self.topic_err_head = '/stanley/error/heading'
        
        self.head_data = []
        self.time_data = []
        self.start_time = None

        self.create_subscription(Float32, self.topic_err_head, self.callback, 10)
        self.get_logger().info("--> Đang vẽ đồ thị Heading (Toàn miền thời gian)...")

    def callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_t = time.time() - self.start_time
        
        self.head_data.append(msg.data)
        self.time_data.append(current_t)

def update(frame, node, line, ax):
    if len(node.time_data) > 0:
        x_data = node.time_data
        y_data = node.head_data
        
        line.set_data(x_data, y_data)
        
        # --- CẤU HÌNH TOÀN MIỀN ---
        ax.set_xlim(0, x_data[-1] + 1)
            
        current_min = min(y_data)
        current_max = max(y_data)
        # Thêm khoảng đệm 5 độ cho thoáng
        ax.set_ylim(current_min - 5, current_max + 5)

    return line,

def main():
    rclpy.init()
    node = HeadingPlotter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Trang trí
    ax.set_title("Heading Error", fontsize=14, fontweight='bold', color='purple')
    ax.set_xlabel("Thời gian (giây)", fontsize=12) # Đã đổi thành Giây
    ax.set_ylabel("Sai số (độ)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    ax.axhline(0, color='black', linewidth=1.5, linestyle='-', alpha=0.5, label="0 độ")
    
    line, = ax.plot([], [], 'm-', linewidth=2, label='Thực tế')
    ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update, fargs=(node, line, ax), interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        fig.savefig('bieu_do_Heading_seconds.png', dpi=300)
        print("\n--> Đã lưu ảnh: bieu_do_Heading_seconds.png")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()