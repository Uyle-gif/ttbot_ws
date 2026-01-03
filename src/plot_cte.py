import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class CTEPlotter(Node):
    def __init__(self):
        super().__init__('cte_plotter_node')
        
        self.topic_err_cte = '/stanley/error/cte'
        
        self.cte_data = []
        self.time_data = [] # Lưu thời gian (giây)
        self.start_time = None # Mốc thời gian bắt đầu

        self.create_subscription(Float32, self.topic_err_cte, self.callback, 10)
        self.get_logger().info("--> Đang vẽ đồ thị CTE (Toàn miền thời gian)...")

    def callback(self, msg):
        # Nếu là gói tin đầu tiên, lấy mốc thời gian hiện tại làm gốc (0s)
        if self.start_time is None:
            self.start_time = time.time()
        
        # Tính thời gian trôi qua kể từ lúc bắt đầu
        current_t = time.time() - self.start_time
        
        self.cte_data.append(msg.data)
        self.time_data.append(current_t)

def update(frame, node, line, ax):
    if len(node.time_data) > 0:
        x_data = node.time_data
        y_data = node.cte_data
        
        line.set_data(x_data, y_data)
        
        # --- CẤU HÌNH TOÀN MIỀN (Hiển thị hết từ 0 đến t hiện tại) ---
        # Trục X: Từ 0 đến thời gian hiện tại + 1 chút lề phải
        ax.set_xlim(0, x_data[-1] + 1)
            
        # Trục Y: Tự động co giãn theo dữ liệu lớn nhất/nhỏ nhất
        current_min = min(y_data)
        current_max = max(y_data)
        # Thêm khoảng đệm 0.1m cho thoáng
        ax.set_ylim(current_min - 0.2, current_max + 0.2)

    return line,

def main():
    rclpy.init()
    node = CTEPlotter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Trang trí
    ax.set_title("CTE", fontsize=14, fontweight='bold', color='blue')
    ax.set_xlabel("Thời gian (giây)", fontsize=12) # Đã đổi thành Giây
    ax.set_ylabel("Sai số (mét)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Đường tham chiếu 0
    ax.axhline(0, color='red', linewidth=1.5, linestyle='-', alpha=0.5, label="0m")
    
    line, = ax.plot([], [], 'b-', linewidth=2, label='Thực tế')
    ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update, fargs=(node, line, ax), interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        fig.savefig('bieu_do_CTE_seconds.png', dpi=300)
        print("\n--> Đã lưu ảnh: bieu_do_CTE_seconds.png")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()