import csv
import matplotlib.pyplot as plt

def generate_square_path(filename="square_path.csv"):
    points = []

    # 1. Đoạn 1: Đi thẳng từ (0,0) lên (15,0)
    # Lấy 150 điểm cho 15m
    for i in range(150):
        x = (i / 149) * 15.0
        y = 0.0
        points.append((x, y))

    # 2. Đoạn 2: Rẽ phải sang (15, -15)
    # Lấy 150 điểm cho 15m
    for i in range(1, 151):
        x = 15.0
        y = 0.0 - (i / 150) * 15.0
        points.append((x, y))

    # 3. Đoạn 3: Rẽ phải đi xuống (0, -15)
    # Lấy 150 điểm cho 15m
    for i in range(1, 151):
        x = 15.0 - (i / 150) * 15.0
        y = -15.0
        points.append((x, y))

    # 4. Đoạn 4: Rẽ phải đi ngang về END tại (0, -0.2)
    # Để hở ra một xíu (cách 0.2m), khoảng cách đi là 14.8m lấy 148 điểm
    for i in range(1, 149):
        x = 0.0
        y = -15.0 + (i / 148) * 14.8
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

    plt.figure(figsize=(8, 8))
    
    # Vẽ quỹ đạo xe chạy
    plt.plot(y_vals, x_vals, 'k-', linewidth=2.5, label='Square Path 15x15m')
    
    # Đánh dấu các mốc
    plt.plot(y_vals[0], x_vals[0], 'go', markersize=8, label='START (0, 0)')
    plt.plot(y_vals[-1], x_vals[-1], 'rx', markersize=10, markeredgewidth=2, label='END (0, -0.2)')
    plt.plot(0, 0, 'm+', markersize=12, markeredgewidth=2, label='Intersection (0, 0)')

    # Định dạng hệ trục: Trục Y đảo ngược để chiều dương hướng sang trái
    plt.gca().invert_xaxis() 
    plt.xlabel('Y (m)')
    plt.ylabel('X (m)')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.title('Mô phỏng Quỹ đạo Hình vuông (15m)')
    plt.axis('equal') 
    plt.show()

if __name__ == '__main__':
    file_name = "square_path.csv"
    pts = generate_square_path(file_name)
    print(f"Đã tạo {len(pts)} điểm cho quỹ đạo hình vuông 15x15m.")
    print(f"Dữ liệu đã được lưu thành công vào file: {file_name}")
    
    plot_path(pts)