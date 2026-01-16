import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import math
from collections import deque

# --- CẤU HÌNH XE (PHẢI CHỈNH CHO ĐÚNG) ---
WHEEL_BASE = 0.65      # Khoảng cách 2 cầu (mét)
TEST_SPEED = 0.5       # Vận tốc xe chạy khi test (m/s)

# [ĐÃ SỬA] Thời gian mỗi bước giảm xuống còn 5 giây
STEP_DURATION = 5.0    
# -----------------------------------------

# Mảng góc lái mong muốn (ĐỘ)
TARGET_ANGLES_DEG = [0.0, -15.0, -25.0, -15.0, 0.0, 15.0, 25.0]

class SteeringStepTest(Node):
    def __init__(self):
        super().__init__('steering_step_test')
        
        # Publisher lệnh
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_controller/cmd_vel', 10)
        
        # Subscriber phản hồi (JointState)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.start_time = time.time()
        
        # Lưu 3000 điểm
        self.t_data = deque(maxlen=3000)
        self.sp_data = deque(maxlen=3000) 
        self.fb_data = deque(maxlen=3000) 
        
        self.current_target_deg = 0.0
        self.current_feedback_deg = 0.0
        
        print(f"--> BẮT ĐẦU TEST GÓC LÁI (Step = {STEP_DURATION}s)")
        print(f"--> Wheelbase: {WHEEL_BASE}m | Speed: {TEST_SPEED}m/s")

    def joint_callback(self, msg):
        # Lấy position thứ 3 (index = 2)
        if len(msg.position) > 2:
            rad_val = msg.position[2] 
            self.current_feedback_deg = math.degrees(rad_val)

    def update_control_loop(self):
        t_now = time.time() - self.start_time
        
        # 1. Xác định bước (Step)
        total_steps = len(TARGET_ANGLES_DEG)
        step_idx = int((t_now / STEP_DURATION) % total_steps)
        
        # 2. Lấy góc lái mục tiêu
        self.current_target_deg = TARGET_ANGLES_DEG[step_idx]
        
        # 3. Tính toán Ackermann: Omega = (v * tan(delta)) / L
        target_rad = math.radians(self.current_target_deg)
        omega = (TEST_SPEED * math.tan(target_rad)) / WHEEL_BASE
        
        # 4. Gửi lệnh
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(TEST_SPEED)
        msg.twist.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        
        # 5. Lưu dữ liệu
        self.t_data.append(t_now)
        self.sp_data.append(self.current_target_deg)
        self.fb_data.append(self.current_feedback_deg)

def animate(i, node, line_sp, line_fb, ax):
    node.update_control_loop()
    
    line_sp.set_data(node.t_data, node.sp_data)
    line_fb.set_data(node.t_data, node.fb_data)
    
    if len(node.t_data) > 0:
        current_t = node.t_data[-1]
        
        # Cửa sổ hiển thị giảm xuống 40s cho gọn vì step ngắn hơn
        WINDOW = 40.0
        if current_t < WINDOW:
            ax.set_xlim(0, WINDOW)
        else:
            ax.set_xlim(current_t - WINDOW, current_t)
            
        all_y = list(node.sp_data) + list(node.fb_data)
        if all_y:
            curr_min = min(all_y)
            curr_max = max(all_y)
            ax.set_ylim(curr_min - 5, curr_max + 5)
            
    return line_sp, line_fb

def main():
    rclpy.init()
    node = SteeringStepTest()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Góc Lái (Độ)")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    line_sp, = ax.plot([], [], 'r--', label='Setpoint', linewidth=0.8)
    line_fb, = ax.plot([], [], 'b-', label='Feedback', linewidth=0.8)
    
    ax.legend(loc='upper left')

    try:
        ani = FuncAnimation(fig, animate, fargs=(node, line_sp, line_fb, ax), interval=30)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()