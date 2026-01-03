import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class GlobalMonitor(Node):
    def __init__(self):
        super().__init__('global_monitor_node')
        
        # Topic
        self.topic_cte = '/stanley/error/cte'
        self.topic_head = '/stanley/error/heading'
        
        # Dữ liệu
        self.start_time = None
        self.time_data = []
        self.cte_data = []
        self.head_data = []

        # Subscribers
        self.create_subscription(Float32, self.topic_cte, self.cte_callback, 10)
        self.create_subscription(Float32, self.topic_head, self.head_callback, 10)
        self.get_logger().info("--> Đã chạy Monitor. Đang mở 2 cửa sổ đồ thị...")

    def check_time(self):
        if self.start_time is None:
            self.start_time = time.time()
        return time.time() - self.start_time

    def cte_callback(self, msg):
        t = self.check_time()
        self.cte_data.append(msg.data)
        # Đồng bộ thời gian tương đối cho đơn giản
        if len(self.time_data) < len(self.cte_data):
            self.time_data.append(t)

    def head_callback(self, msg):
        self.check_time() # Start timer if needed
        self.head_data.append(msg.data)

def update(frame, node, lines, axes):
    l_cte, l_head = lines
    ax_cte, ax_head = axes
    
    # Chỉ vẽ khi có dữ liệu thời gian
    n = len(node.time_data)
    n_cte = len(node.cte_data)
    n_head = len(node.head_data)
    
    limit = min(n, n_cte, n_head)
    
    if limit > 0:
        x = node.time_data[:limit]
        
        # --- CẬP NHẬT CỬA SỔ 1: CTE ---
        l_cte.set_data(x, node.cte_data[:limit])
        ax_cte.set_xlim(0, x[-1] + 1)
        ax_cte.set_ylim(min(node.cte_data[:limit])-0.2, max(node.cte_data[:limit])+0.2)
        
        # --- CẬP NHẬT CỬA SỔ 2: HEADING ---
        l_head.set_data(x, node.head_data[:limit])
        ax_head.set_xlim(0, x[-1] + 1)
        ax_head.set_ylim(min(node.head_data[:limit])-5, max(node.head_data[:limit])+5)

    return l_cte, l_head

def main():
    rclpy.init()
    node = GlobalMonitor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # --- TẠO CỬA SỔ 1: CTE ---
    fig1, ax1 = plt.subplots(figsize=(8, 4))
    fig1.canvas.manager.set_window_title('CTE)')
    ax1.set_title("Cross-Track Error", fontweight='bold', color='blue')
    ax1.set_xlabel("Thời gian (s)"); ax1.set_ylabel("Mét (m)")
    ax1.grid(True, linestyle='--'); ax1.axhline(0, color='red', alpha=0.5)
    line1, = ax1.plot([], [], 'b-', lw=2)

    # --- TẠO CỬA SỔ 2: HEADING ---
    fig2, ax2 = plt.subplots(figsize=(8, 4))
    fig2.canvas.manager.set_window_title('Heading Error')
    ax2.set_title("Heading Error", fontweight='bold', color='purple')
    ax2.set_xlabel("Thời gian (s)"); ax2.set_ylabel("Độ (deg)")
    ax2.grid(True, linestyle='--'); ax2.axhline(0, color='black', alpha=0.5)
    line2, = ax2.plot([], [], 'm-', lw=2)

    # Chạy Animation chung
    # Lưu ý: Matplotlib quản lý event loop chung nên 1 FuncAnimation có thể update nhiều figure
    ani = FuncAnimation(fig1, update, fargs=(node, (line1, line2), (ax1, ax2)), interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Lưu cả 2 ảnh khi tắt
        fig1.savefig('ket_qua_CTE.png', dpi=300)
        fig2.savefig('ket_qua_Heading.png', dpi=300)
        print("\n--> Đã lưu 2 ảnh: ket_qua_CTE.png và ket_qua_Heading.png")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()