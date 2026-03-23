import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class StanleyMonitor(Node):
    def __init__(self):
        super().__init__('stanley_monitor_node')
        
        self.topic_cte = '/stanley/error/cte'
        self.topic_head = '/stanley/error/heading'
        
        self.start_time = None
        self.time_data = []
        self.cte_data = []
        self.head_data = []

        self.create_subscription(Float32, self.topic_cte, self.cte_callback, 10)
        self.create_subscription(Float32, self.topic_head, self.head_callback, 10)
        self.get_logger().info("--> Monitor STANLEY started.")

    def check_time(self):
        if self.start_time is None:
            self.start_time = time.time()
        return time.time() - self.start_time

    def cte_callback(self, msg):
        t = self.check_time()
        self.cte_data.append(msg.data)
        if len(self.time_data) < len(self.cte_data):
            self.time_data.append(t)

    def head_callback(self, msg):
        self.check_time()
        self.head_data.append(msg.data)

def update(frame, node, line_cte, line_head, ax_cte, ax_head):
    # Đồng bộ độ dài dữ liệu
    n = len(node.time_data)
    limit = min(n, len(node.cte_data), len(node.head_data))
    
    if limit > 0:
        x = node.time_data[:limit]
        
        # --- Vẽ CTE (Biểu đồ trên) ---
        line_cte.set_data(x, node.cte_data[:limit])
        ax_cte.set_xlim(0, x[-1] + 1)
        current_cte = node.cte_data[:limit]
        ax_cte.set_ylim(min(current_cte)-0.2, max(current_cte)+0.2)
        
        # --- Vẽ Heading (Biểu đồ dưới) ---
        line_head.set_data(x, node.head_data[:limit])
        ax_head.set_xlim(0, x[-1] + 1)
        current_head = node.head_data[:limit]
        ax_head.set_ylim(min(current_head)-5, max(current_head)+5)

    return line_cte, line_head

def main():
    rclpy.init()
    node = StanleyMonitor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # TẠO 1 CỬA SỔ CÓ 2 BIỂU ĐỒ (2 hàng, 1 cột)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Stanley Monitor System')
    
    # --- Biểu đồ 1: CTE ---
    ax1.set_title("Stanley: Sai số Lệch làn (CTE)", fontweight='bold', color='blue')
    ax1.set_ylabel("Error (mét)")
    ax1.grid(True, linestyle='--')
    ax1.axhline(0, color='red', alpha=0.5)
    line1, = ax1.plot([], [], 'b-', lw=2, label='CTE')

    # --- Biểu đồ 2: Heading ---
    ax2.set_title("Stanley: Sai số Góc hướng (Heading)", fontweight='bold', color='purple')
    ax2.set_xlabel("Thời gian (giây)")
    ax2.set_ylabel("Error (độ)")
    ax2.grid(True, linestyle='--')
    ax2.axhline(0, color='black', alpha=0.5)
    line2, = ax2.plot([], [], 'm-', lw=2, label='Heading')

    # Chạy animation
    ani = FuncAnimation(fig, update, fargs=(node, line1, line2, ax1, ax2), interval=100)
    plt.tight_layout() # Căn chỉnh lề tự động cho đẹp

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        fig.savefig('BaoCao_Stanley_Full.png', dpi=300)
        print("\n--> Đã lưu ảnh: BaoCao_Stanley_Full.png")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()