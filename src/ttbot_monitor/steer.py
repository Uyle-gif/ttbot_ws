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

# --- CẤU HÌNH XE ---
WHEEL_BASE = 0.65       # Khoảng cách 2 cầu (mét)
TEST_SPEED = 0.5        # Vận tốc xe chạy (m/s)

# --- CẤU HÌNH GÓC LÁI MƯỢT (SINE WAVE) ---
MAX_STEERING_ANGLE = 25.0  # Góc lái tối đa (Độ) (+/- 25 độ)
CYCLE_TIME = 10.0          # Thời gian hoàn thành 1 chu kỳ (giây) - Đảo từ 0 -> 25 -> -25 -> 0
# -----------------------------------------

class SteeringSmoothTest(Node):
    def __init__(self):
        super().__init__('steering_smooth_test')
        
        # Publisher lệnh
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_controller/cmd_vel', 10)
        
        # Subscriber phản hồi
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.start_time = time.time()
        
        # Lưu dữ liệu vẽ đồ thị
        self.t_data = deque(maxlen=3000)
        self.sp_data = deque(maxlen=3000) 
        self.fb_data = deque(maxlen=3000) 
        
        self.current_target_deg = 0.0
        self.current_feedback_deg = 0.0
        
        print(f"--> BẮT ĐẦU TEST SINE WAVE")
        print(f"--> Max Angle: {MAX_STEERING_ANGLE} deg | Chu kỳ: {CYCLE_TIME}s")

    def joint_callback(self, msg):
        # Lấy position thứ 3 (index = 2) - Tùy chỉnh theo robot thực tế
        if len(msg.position) > 2:
            rad_val = msg.position[2] 
            self.current_feedback_deg = math.degrees(rad_val)

    def update_control_loop(self):
        t_now = time.time() - self.start_time
        
        # --- TẠO GÓC LÁI TĂNG/GIẢM DẦN (SÓNG SIN) ---
        # Công thức: Angle = A * sin(2 * pi * f * t)
        # f = 1 / T
        frequency = 1.0 / CYCLE_TIME
        self.current_target_deg = MAX_STEERING_ANGLE * math.sin(2 * math.pi * frequency * t_now)
        
        # --- TÍNH TOÁN ACKERMANN ---
        # Omega = (v * tan(delta)) / L
        target_rad = math.radians(self.current_target_deg)
        omega = (TEST_SPEED * math.tan(target_rad)) / WHEEL_BASE
        
        # Gửi lệnh
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(TEST_SPEED)
        msg.twist.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        
        # Lưu dữ liệu
        self.t_data.append(t_now)
        self.sp_data.append(self.current_target_deg)
        self.fb_data.append(self.current_feedback_deg)

def animate(i, node, line_sp, line_fb, ax):
    node.update_control_loop()
    
    line_sp.set_data(node.t_data, node.sp_data)
    line_fb.set_data(node.t_data, node.fb_data)
    
    if len(node.t_data) > 0:
        current_t = node.t_data[-1]
        
        # Cửa sổ hiển thị 20s để nhìn rõ sóng
        WINDOW = 20.0
        if current_t < WINDOW:
            ax.set_xlim(0, WINDOW)
        else:
            ax.set_xlim(current_t - WINDOW, current_t)
            
        # Cố định trục Y theo Max Angle để đồ thị không bị giật
        ax.set_ylim(-MAX_STEERING_ANGLE - 5, MAX_STEERING_ANGLE + 5)
            
    return line_sp, line_fb

def main():
    rclpy.init()
    node = SteeringSmoothTest()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Góc Lái (Độ)")
    ax.set_title("Kiểm tra bám góc lái (Sine Wave)")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    line_sp, = ax.plot([], [], 'r--', label='Setpoint (Sine)', linewidth=1.5)
    line_fb, = ax.plot([], [], 'b-', label='Feedback', linewidth=1.0)
    
    ax.legend(loc='upper right')

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