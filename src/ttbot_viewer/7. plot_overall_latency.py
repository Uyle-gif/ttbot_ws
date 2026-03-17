import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Cấu hình đường dẫn
file_path = os.path.expanduser('~/ttbot_ws/src/FAST-LIVO2/Log/odom_time.csv')

if not os.path.exists(file_path):
    print(f"Lỗi: Không tìm thấy file tại {file_path}")
    exit()

# 2. Đọc dữ liệu
df = pd.read_csv(file_path)
df['Latency_ms'] = pd.to_numeric(df['Latency_ms'], errors='coerce')
df = df.dropna(subset=['Latency_ms'])

# GOM CHUNG TOÀN BỘ DỮ LIỆU ĐỘ TRỄ (Không phân biệt LIO/VIO)
y_lat = df['Latency_ms'].to_numpy()
x_values = np.arange(len(y_lat))

# Tính các thông số thống kê tổng thể
mean_lat = np.mean(y_lat)
max_lat = np.max(y_lat)
std_lat = np.std(y_lat)

# 3. Khởi tạo Dashboard
fig = plt.figure(figsize=(15, 10))
plt.rcParams['font.family'] = 'serif'
gs = fig.add_gridspec(2, 2)

# Chọn màu Xanh lá mạ (Green) đại diện cho Toàn hệ thống
system_color = '#2ca02c'

# --- BIỂU ĐỒ A: LINE PLOT (Tiến trình Độ trễ tổng thể) ---
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(x_values, y_lat, label='System Latency', color=system_color, alpha=0.7, lw=1.2)
ax1.axhline(y=mean_lat, color='red', linestyle='--', lw=2, label=f'Overall Mean: {mean_lat:.2f} ms')

ax1.set_title('A. Overall End-to-End System Latency', fontsize=14, fontweight='bold')
ax1.set_ylabel('Total Latency (ms)', fontsize=12)
ax1.set_xlabel('Odometry Frame Sequence', fontsize=12)
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':', alpha=0.6)

# --- BIỂU ĐỒ B: BOX PLOT (Đánh giá độ ổn định tổng thể) ---
ax2 = fig.add_subplot(gs[1, 0])
bp = ax2.boxplot([y_lat], labels=[f'Overall Latency\n(Max: {max_lat:.2f} ms)'], patch_artist=True, widths=0.4)

# Tô màu Boxplot
bp['boxes'][0].set_facecolor(system_color)
bp['boxes'][0].set_alpha(0.6)

ax2.set_title('B. Overall Stability (Jitter Analysis)', fontsize=14, fontweight='bold')
ax2.set_ylabel('Total Latency (ms)', fontsize=12)
ax2.grid(True, linestyle=':', alpha=0.3)

# --- BIỂU ĐỒ C: HISTOGRAM (Mật độ phân phối tổng thể) ---
ax3 = fig.add_subplot(gs[1, 1])
ax3.hist(y_lat, bins=50, alpha=0.6, color=system_color, label='Latency Distribution')
ax3.axvline(x=mean_lat, color='red', linestyle='--', lw=2, label=f'Mean: {mean_lat:.2f} ms')

ax3.set_title('C. System Latency Distribution', fontsize=14, fontweight='bold')
ax3.set_xlabel('Total Latency (ms)', fontsize=12)
ax3.set_ylabel('Frequency', fontsize=12)
ax3.legend()
ax3.grid(True, linestyle=':', alpha=0.3)

# Lưu file ảnh
plt.tight_layout()
output_filename = "ieee_overall_latency_report.png"
plt.savefig(output_filename, dpi=300)
print("="*50)
print("THỐNG KÊ ĐỘ TRỄ TỔNG THỂ HỆ THỐNG:")
print(f"- Trung bình (Mean) : {mean_lat:.2f} ms")
print(f"- Lớn nhất (Max)    : {max_lat:.2f} ms")
print(f"- Độ lệch chuẩn (Std): {std_lat:.2f} ms")
print("="*50)
print(f"Thành công! Biểu đồ đánh giá tổng thể đã được lưu tại: {output_filename}")
plt.show()