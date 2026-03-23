import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import random
import math

# 📍 1. BÁC ĐIỀN TỌA ĐỘ LÚC XE XUẤT PHÁT VÀO ĐÂY (Mặc định đang để 0, 0)
ROBOT_START_X = 0.0
ROBOT_START_Y = 0.0

# 📏 2. KHOẢNG CÁCH TỐI THIỂU TỪ XE ĐẾN ĐÍCH (Đơn vị: Mét)
MIN_DISTANCE_FROM_ROBOT = 15.0 

class RandomGoalGenerator(Node):
    def __init__(self):
        super().__init__('random_goal_generator')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.get_logger().info(f'⏳ Đang tìm 20 điểm CÁCH XE ÍT NHẤT {MIN_DISTANCE_FROM_ROBOT}m và tránh xa vật cản...')

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data
        
        goals = []
        safe_margin = 10 # Bán kính an toàn tránh tường (10 ô)

        while len(goals) < 20:
            gx = random.randint(safe_margin, width - safe_margin - 1)
            gy = random.randint(safe_margin, height - safe_margin - 1)
            
            x = ox + (gx + 0.5) * res
            y = oy + (gy + 0.5) * res
            
            # --- ĐIỀU KIỆN 1: PHẢI CÁCH XA XE TỐI THIỂU 15 MÉT ---
            dist_to_robot = math.hypot(x - ROBOT_START_X, y - ROBOT_START_Y)
            if dist_to_robot < MIN_DISTANCE_FROM_ROBOT:
                continue # Nằm quá gần xe -> Bỏ qua ngay lập tức
            
            # --- ĐIỀU KIỆN 2: PHẢI CÁCH XA TƯỜNG ---
            is_completely_safe = True
            for dx in range(-safe_margin, safe_margin + 1):
                for dy in range(-safe_margin, safe_margin + 1):
                    idx = (gy + dy) * width + (gx + dx)
                    if data[idx] != 0:
                        is_completely_safe = False
                        break
                if not is_completely_safe:
                    break
            
            # Nếu thỏa mãn cả 2 điều kiện thì mới đưa vào danh sách
            if is_completely_safe:
                goals.append((round(x, 2), round(y, 2)))
        
        self.get_logger().info(f'\n\n✅ ĐÃ TÌM THẤY 20 ĐIỂM (Cách xe > {MIN_DISTANCE_FROM_ROBOT}m)! COPY MẢNG NÀY:\n')
        print("        self.goals = [")
        for i in range(0, 20, 4):
            row = ", ".join([f"({p[0]}, {p[1]})" for p in goals[i:i+4]])
            print(f"            {row},")
        print("        ]\n")
        
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RandomGoalGenerator()
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()