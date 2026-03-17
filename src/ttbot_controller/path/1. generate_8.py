import math
import csv
import matplotlib.pyplot as plt

def generate_figure8_path(filename="figure8_path.csv"):
    points = []

    # 1. Đoạn đường thẳng (dài 3 mét)
    # X từ 0 đến 3, Y = 0
    for i in range(100):
        x = (i / 99) * 3.0
        y = 0.0
        points.append((x, y))

    # 2. Nửa phải vòng tròn dưới
    # Tâm (6.5, 0), Bán kính 3.5. Bắt đầu từ X=3, Y=0.
    # Quét góc từ Pi đến 2*Pi (đi sang phía Y âm)
    for i in range(1, 201):
        theta = math.pi + (i / 200) * math.pi
        x = 6.5 + 3.5 * math.cos(theta)
        y = 3.5 * math.sin(theta)
        points.append((x, y))

    # 3. Vòng tròn trên (Trọn 1 vòng)
    # Tâm (13.5, 0), Bán kính 3.5. Bắt đầu từ giao điểm X=10, Y=0.
    # Quét ngược từ Pi lùi về -Pi để xe đan sang nửa trái (Y dương)
    for i in range(1, 401):
        theta = math.pi - (i / 400) * (2 * math.pi)
        x = 13.5 + 3.5 * math.cos(theta)
        y = 3.5 * math.sin(theta)
        points.append((x, y))

    # 4. Nửa trái vòng tròn dưới (Hở 1 xíu ở điểm END)
    # Tâm (6.5, 0), Bắt đầu từ X=10, Y=0.
    # Quét từ 0 đến Pi. Để hở 1 xíu, mình dừng sớm ở Pi - 0.05 (cách ~2.8 độ)
    end_theta = math.pi - 0.1
    for i in range(1, 201):
        theta = (i / 200) * end_theta
        x = 6.5 + 3.5 * math.cos(theta)
        y = 3.5 * math.sin(theta)
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

    plt.figure(figsize=(7, 10))
    
    # Vẽ quỹ đạo xe chạy
    plt.plot(y_vals, x_vals, 'k-', linewidth=2.5, label='Actual Path')
    
    # Đánh dấu các mốc quan trọng
    plt.plot(y_vals[0], x_vals[0], 'go', markersize=8, label='START (0,0)')
    plt.plot(y_vals[-1], x_vals[-1], 'rx', markersize=10, markeredgewidth=2, label='END')
    plt.plot(0, 6.5, 'b.', markersize=12, label='Center 1 (6.5, 0)')
    plt.plot(0, 13.5, 'c.', markersize=12, label='Center 2 (13.5, 0)')
    plt.plot(0, 10, 'm+', markersize=12, markeredgewidth=2, label='Cross Point (10, 0)')

    # Định dạng hệ trục: Trục Y đảo ngược để chiều dương hướng sang trái
    plt.gca().invert_xaxis() 
    plt.xlabel('Y (m) [Dương sang trái]')
    plt.ylabel('X (m) [Dương lên trên]')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.title('Mô phỏng Quỹ đạo Số 8 (Figure-8)')
    plt.axis('equal') 
    plt.show()

if __name__ == '__main__':
    file_name = "figure8_path.csv"
    pts = generate_figure8_path(file_name)
    print(f"Đã tạo {len(pts)} điểm cho quỹ đạo hình số 8.")
    print(f"Dữ liệu đã được lưu thành công vào file: {file_name}")
    
    plot_path(pts)