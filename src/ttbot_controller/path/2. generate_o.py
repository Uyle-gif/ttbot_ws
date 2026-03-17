import math
import csv
import matplotlib.pyplot as plt

def generate_balloon_path(filename="balloon_path.csv"):
    points = []

    # 1. Đoạn đường thẳng (dài 3 mét)
    # X chạy từ 0 đến 3, Y = 0
    num_line_points = 100
    for i in range(num_line_points):
        x = (i / (num_line_points - 1)) * 3.0
        y = 0.0
        points.append((x, y))

    # 2. Đoạn vòng tròn (Bán kính 3.5 mét, Tâm tại X=6.5, Y=0)
    # Phương trình: X = 6.5 - 3.5*cos(theta), Y = -3.5*sin(theta)
    # Quét góc 1.99 * Pi (khoảng 358 độ) để chỉ hở 1 xíu ở điểm END
    num_circle_points = 500
    max_theta = 1.97 * math.pi 

    for i in range(1, num_circle_points + 1): 
        theta = (i / num_circle_points) * max_theta
        
        x = 6.5 - 3.5 * math.cos(theta)
        y = -3.5 * math.sin(theta)
        
        points.append((x, y))

    # 3. Ghi dữ liệu ra file CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for p in points:
            writer.writerow([round(p[0], 4), round(p[1], 4)])

    return points

def plot_path(points):
    """Hàm này vẽ hình lên màn hình để bạn nghiệm thu"""
    x_vals = [p[0] for p in points]
    y_vals = [p[1] for p in points]

    plt.figure(figsize=(7, 9))
    
    # Vẽ quỹ đạo
    plt.plot(y_vals, x_vals, 'k-', linewidth=2.5, label='Path')
    
    # Đánh dấu START và END
    plt.plot(y_vals[0], x_vals[0], 'go', markersize=8, label='START (0,0)')
    plt.plot(y_vals[-1], x_vals[-1], 'rx', markersize=10, markeredgewidth=2, label='END')
    
    # Đánh dấu Tâm vòng tròn (6.5, 0)
    plt.plot(0, 6.5, 'k.', markersize=15, label='Center (6.5, 0)')

    # Định dạng đồ thị
    plt.gca().invert_xaxis() # Đảo ngược trục Y (chiều dương sang trái)
    plt.xlabel('Y (m)')
    plt.ylabel('X (m)')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.axis('equal') 
    plt.show()

if __name__ == '__main__':
    file_name = "balloon_path.csv"
    pts = generate_balloon_path(file_name)
    print(f"Đã tạo {len(pts)} điểm (bán kính 3.5m, hở rất nhỏ).")
    print(f"Lưu thành công vào file: {file_name}")
    
    plot_path(pts)