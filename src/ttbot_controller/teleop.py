#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
Điều khiển ttbot - Teleop Truyền thống
--------------------------------------
Nhấn phím để thay đổi vận tốc (không cần giữ):
        w
   a    s    d
        x

w/x : Tăng/Giảm tốc độ tiến (±0.2 m/s)
a/d : Tăng/Giảm tốc độ quay (±0.1 rad/s)
s   : Dừng khẩn cấp (về 0)

CTRL-C để thoát
"""

class TeleopNormal(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(TwistStamped, '/ackermann_controller/cmd_vel', 10)
        
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link" 
        
        twist_msg.twist.linear.x = self.linear_x
        twist_msg.twist.angular.z = self.angular_z
        
        self.publisher.publish(twist_msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNormal()
    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'w':
                node.linear_x += 0.2
            elif key == 'x':
                node.linear_x -= 0.2
            elif key == 'a':
                node.angular_z += 0.1
            elif key == 'd':
                node.angular_z -= 0.1
            elif key == 's':
                node.linear_x = 0.0
                node.angular_z = 0.0
            elif key == '\x03': 
                break
            
            node.linear_x = round(min(max(node.linear_x, -3.0), 6.0), 2)
            node.angular_z = round(min(max(node.angular_z, -1.0), 1.0), 2)

            sys.stdout.write(f"\rVận tốc: {node.linear_x:.1f} m/s | Quay: {node.angular_z:.1f} rad/s   ")
            sys.stdout.flush()
            
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(f"\nLỗi: {e}")
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        stop_node = rclpy.create_node('stop_node')
        pub = stop_node.create_publisher(TwistStamped, '/ackermann_controller/cmd_vel', 10)
        pub.publish(TwistStamped())
        rclpy.shutdown()

if __name__ == '__main__':
    main()