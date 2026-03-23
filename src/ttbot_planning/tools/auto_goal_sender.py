import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class AutoGoalSender(Node):
    def __init__(self):
        super().__init__('auto_goal_sender')
        
        # Tạo publisher gửi tín hiệu đích đến cho Nav2
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 20 tọa độ test case của bạn
        self.goals = [
            (15.1, -8.37), (25.1, 4.53), (21.9, -7.92), (21.9, 3.28),
            (15.75, -9.22), (24.7, 1.68), (14.45, 6.28), (21.25, -7.52),
            (22.7, -6.97), (22.1, 4.18), (14.25, -10.07), (18.6, -5.27),
            (15.1, 2.28), (18.95, -0.82), (21.55, 4.53), (22.85, -9.02),
            (14.95, -10.02), (20.8, -6.47), (16.0, -1.47), (14.85, 2.88),
        ]
        
        # Đợi 2 giây để node kết nối ổn định với hệ thống ROS 2
        time.sleep(2.0)
        self.send_goals()

    def send_goals(self):
        for i, (x, y) in enumerate(self.goals):
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Gán tọa độ X, Y
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            
            # Quaternion góc xoay mặc định (không bẻ lái ở đích)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            
            self.get_logger().info(f'🚀 Đang gửi Test Case {i+1}/20: [X: {x}, Y: {y}]')
            self.publisher_.publish(msg)
            
            # Dừng 4 giây để Nav2 tính đường và file path_evaluator.py kịp lưu Excel
            time.sleep(4.0)
            
        self.get_logger().info('✅ ĐÃ BẮN XONG 20 MỤC TIÊU!')

def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalSender()
    # Chạy 1 lần rồi tự tắt
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()