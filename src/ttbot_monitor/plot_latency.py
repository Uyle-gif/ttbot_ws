import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Cấu hình đường dẫn chính xác (FAST-LIVO2)
file_path = os.path.expanduser('~/ttbot_ws/src/FAST-LIVO2/Log/odom_time.csv')

if not os.path.exists(file_path):
    print(f"Lỗi: Không tìm thấy file tại {file_path}")
    exit()

# 2. Đọc dữ liệu và chỉ lấy cột Latency_ms
df = pd.read_csv(file_path)
df['Latency_ms'] = pd.to_numeric(df['Latency_ms'], errors='coerce')
df = df.dropna(subset=['Latency_ms']) # Xóa các hàng bị lỗi dữ liệu Latency

# 3. Khởi tạo Dashboard chuyên biệt cho Latency
fig = plt.figure(figsize=(15, 10))
plt.rcParams['font.family'] = 'serif'
gs = fig.add_gridspec(2, 2)

# Dùng tông màu Đỏ/Tím để phân biệt với biểu đồ Execution Time (Xanh/Cam) trước đó
colors_lat = {'LIO': '#d62728', 'VIO': '#9467bd'}

# --- BIỂU ĐỒ A: LINE PLOT (Tiến trình Độ trễ theo thời gian) ---
ax1 = fig.add_subplot(gs[0, :])

for mode in ['LIO', 'VIO']:
    mask = df['Mode'].str.contains(mode, na=False)
    data_subset = df[mask]
    
    if not data_subset.empty:
        x_values = data_subset.index.to_numpy()
        y_lat = data_subset['Latency_ms'].to_numpy()
        
        ax1.plot(x_values, y_lat, label=f'{mode} Latency', color=colors_lat[mode], alpha=0.7, lw=1.2)
        
        # Vẽ đường trung bình
        mean_lat = np.mean(y_lat)
        ax1.axhline(y=mean_lat, color=colors_lat[mode], linestyle='--', lw=2, 
                    label=f'Mean {mode} Latency: {mean_lat:.2f} ms')

ax1.set_title('A. End-to-End System Latency over Sequence', fontsize=14, fontweight='bold')
ax1.set_ylabel('Total Latency (ms)', fontsize=12)
ax1.set_xlabel('Frame Sequence', fontsize=12)
ax1.legend(loc='upper right', ncol=2)
ax1.grid(True, linestyle=':', alpha=0.6)

# --- BIỂU ĐỒ B: BOX PLOT (Đánh giá độ Jitter của Hệ thống truyền thông) ---
ax2 = fig.add_subplot(gs[1, 0])

box_data = [
    df[df['Mode'].str.contains('LIO', na=False)]['Latency_ms'].to_numpy(),
    df[df['Mode'].str.contains('VIO', na=False)]['Latency_ms'].to_numpy()
]
labels = ['LIO\nLatency', 'VIO\nLatency']

bp = ax2.boxplot(box_data, labels=labels, patch_artist=True, widths=0.4)

# Tô màu Boxplot
for patch, color in zip(bp['boxes'], [colors_lat['LIO'], colors_lat['VIO']]):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)

ax2.set_title('B. Latency Stability (Jitter Analysis)', fontsize=14, fontweight='bold')
ax2.set_ylabel('Total Latency (ms)', fontsize=12)
ax2.grid(True, linestyle=':', alpha=0.3)

# --- BIỂU ĐỒ C: HISTOGRAM (Mật độ phân phối Độ trễ) ---
ax3 = fig.add_subplot(gs[1, 1])

for mode in ['LIO', 'VIO']:
    lat_data = df[df['Mode'].str.contains(mode, na=False)]['Latency_ms'].to_numpy()
    
    if len(lat_data) > 0:
        ax3.hist(lat_data, bins=40, alpha=0.5, color=colors_lat[mode], label=f'{mode} Latency')

ax3.set_title('C. Latency Distribution', fontsize=14, fontweight='bold')
ax3.set_xlabel('Total Latency (ms)', fontsize=12)
ax3.set_ylabel('Frequency', fontsize=12)
ax3.legend()
ax3.grid(True, linestyle=':', alpha=0.3)

# Lưu file ảnh
plt.tight_layout()
output_filename = "ieee_latency_analysis_report.png"
plt.savefig(output_filename, dpi=300)
print(f"Thành công! Biểu đồ phân tích Latency đã được lưu tại: {output_filename}")
plt.show()