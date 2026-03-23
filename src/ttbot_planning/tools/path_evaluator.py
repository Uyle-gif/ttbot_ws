import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
import math
import csv
import re

class PathEvaluator(Node):
    def __init__(self):
        super().__init__('path_evaluator')
        
        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 10)
            
        self.path_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
            
        self.latest_planning_time = 0.0 
        self.test_count = 0
        
        self.csv_file = open('astar_kinematic_metrics.csv', mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(['Test Case', 'Path Length (m)', 'Planning Time (ms)', 'Total Turning Angle (Deg)', 'Inflection Points'])
            
        self.get_logger().info("✅ ĐÃ SẴN SÀNG! Đang lắng nghe hệ thống...")

    def rosout_callback(self, msg):
        # 🚀 Đã sửa lại Regex: Tìm chữ "Time:" thay vì "Planning Time:"
        if "Time:" in msg.msg and "ms" in msg.msg:
            match = re.search(r'Time: ([\d\.]+) ms', msg.msg)
            if match:
                self.latest_planning_time = float(match.group(1))

    def plan_callback(self, msg):
        if len(msg.poses) < 2:
            return
            
        self.test_count += 1
        poses = msg.poses
        
        path_length = 0.0
        total_turn_angle = 0.0
        inflection_points = 0
        segment_yaws = []
        
        for i in range(len(poses) - 1):
            x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
            x2, y2 = poses[i+1].pose.position.x, poses[i+1].pose.position.y
            path_length += math.hypot(x2 - x1, y2 - y1)
            segment_yaws.append(math.atan2(y2 - y1, x2 - x1))
            
        for i in range(len(segment_yaws) - 1):
            diff = segment_yaws[i+1] - segment_yaws[i]
            diff = math.atan2(math.sin(diff), math.cos(diff))
            
            abs_diff = abs(math.degrees(diff))
            total_turn_angle += abs_diff
            
            if abs_diff > 5.0:  
                inflection_points += 1
                
        p_time = self.latest_planning_time
                
        self.get_logger().info(f"--- TEST CASE {self.test_count} ---")
        self.get_logger().info(f"Độ dài: {path_length:.2f} m | Thời gian C++: {p_time:.3f} ms | Góc quay: {total_turn_angle:.1f}° | Điểm gãy: {inflection_points}")
        
        self.csv_writer.writerow([self.test_count, round(path_length, 3), round(p_time, 3), round(total_turn_angle, 1), inflection_points])
        self.csv_file.flush()
        
        self.latest_planning_time = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PathEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()