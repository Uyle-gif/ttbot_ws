import csv
import matplotlib.pyplot as plt

def generate_square_path(filename="square_path.csv"):
    points = []

    # 1. Đoạn 1: Đi thẳng từ (0,0) lên (13,0)
    # Lấy 130 điểm cho 13m
    for i in range(130):
        x = (i / 129) * 13.0
        y = 0.0
        points.append((x, y))

    # 2. Đoạn 2: Rẽ phải sang (13, -10)
    # Lấy 100 điểm cho 10m
    for i in range(1, 101):
        x = 13.0
        y = 0.0 - (i / 100) * 10.0
        points.append((x, y))

    # 3. Đoạn 3: Đi thẳng xuống (3, -10)
    # Lấy 100 điểm cho 10m
    for i in range(1, 101):
        x = 13.0 - (i / 100) * 10.0
        y = -10.0
        points.append((x, y))

    # 4. Đoạn 4: Đi ngang về END tại (3, -0.2)
    # Để hở ra một xíu ở chỗ chữ X, khoảng cách 9.8m lấy 98 điểm
    for i in range(1, 99):
        x = 3.0
        y = -10.0 + (i / 98) * 9.7
        points.append((x, y))

    # Ghi dữ liệu ra file CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for p in points:
            writer.writerow([round(p[0], 4), round(p[1], 4)])

    return points

def plot_path(points):
    """Hàm hiển thị trực quan đồ thị"""
    x_vals = [p[0] for p in points]
    y_vals = [p[1] for p in points]

    plt.figure(figsize=(7, 9))
    
    # Vẽ quỹ đạo xe chạy
    plt.plot(y_vals, x_vals, 'k-', linewidth=2.5, label='Square Path')
    
    # Đánh dấu các mốc
    plt.plot(y_vals[0], x_vals[0], 'go', markersize=8, label='START (0,0)')
    plt.plot(y_vals[-1], x_vals[-1], 'rx', markersize=10, markeredgewidth=2, label='END (3, -0.2)')
    plt.plot(0, 3, 'm+', markersize=12, markeredgewidth=2, label='Intersection (3,0)')

    # Định dạng hệ trục: Trục Y đảo ngược để chiều dương hướng sang trái
    plt.gca().invert_xaxis() 
    plt.xlabel('Y (m)')
    plt.ylabel('X (m)')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.title('Mô phỏng Quỹ đạo Hình vuông')
    plt.axis('equal') 
    plt.show()

if __name__ == '__main__':
    file_name = "square_path.csv"
    pts = generate_square_path(file_name)
    print(f"Đã tạo {len(pts)} điểm cho quỹ đạo hình vuông.")
    print(f"Dữ liệu đã được lưu thành công vào file: {file_name}")
    
    plot_path(pts)