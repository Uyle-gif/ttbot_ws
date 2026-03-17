import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Cấu hình đường dẫn chính xác (FAST-LIVO2)
file_path = os.path.expanduser('~/ttbot_ws/src/FAST-LIVO2/Log/odom_time.csv')

if not os.path.exists(file_path):
    print(f"Lỗi: Không tìm thấy file tại {file_path}")
    exit()

# Đọc dữ liệu và xử lý lỗi định dạng
df = pd.read_csv(file_path)
df['Odom_Time_ms'] = pd.to_numeric(df['Odom_Time_ms'], errors='coerce')
df = df.dropna()
# Lọc bỏ các khung hình rỗng (thoát sớm) có thời gian xử lý < 1.0 ms
df = df[df['Odom_Time_ms'] > 1.0]

# Khởi tạo Dashboard
fig = plt.figure(figsize=(15, 10))
plt.rcParams['font.family'] = 'serif'
gs = fig.add_gridspec(2, 2)

# --- BIỂU ĐỒ A: LINE PLOT (Tiến trình thời gian) ---
ax1 = fig.add_subplot(gs[0, :])
colors = {'LIO': '#1f77b4', 'VIO': '#ff7f0e'}

for mode in ['LIO', 'VIO']:
    # Lọc dữ liệu và chuyển sang numpy array để sửa lỗi ValueError
    mask = df['Mode'].str.contains(mode, na=False)
    data_subset = df[mask]
    
    if not data_subset.empty:
        # SỬA LỖI TẠI ĐÂY: .to_numpy()
        x_values = data_subset.index.to_numpy()
        y_values = data_subset['Odom_Time_ms'].to_numpy()
        
        ax1.plot(x_values, y_values, label=f'{mode} Execution', color=colors[mode], alpha=0.5, lw=1)
        
        mean_val = np.mean(y_values)
        ax1.axhline(y=mean_val, color=colors[mode], linestyle='--', lw=2, 
                    label=f'Mean {mode}: {mean_val:.2f}ms')

ax1.set_title('A. Computational Efficiency over Sequence', fontsize=14, fontweight='bold')
ax1.set_ylabel('Execution Time (ms)', fontsize=12)
ax1.legend(loc='upper right', ncol=2)
ax1.grid(True, linestyle=':', alpha=0.6)

# --- BIỂU ĐỒ B: BOX PLOT (Độ ổn định hệ thống) ---
ax2 = fig.add_subplot(gs[1, 0])
# Chuyển dữ liệu sang list numpy để vẽ Boxplot an toàn
box_data = [
    df[df['Mode'].str.contains('LIO', na=False)]['Odom_Time_ms'].to_numpy(),
    df[df['Mode'].str.contains('VIO', na=False)]['Odom_Time_ms'].to_numpy()
]
bp = ax2.boxplot(box_data, labels=['LIO Module', 'VIO Module'], patch_artist=True, widths=0.4)

# Tô màu cho Boxplot
for patch, color in zip(bp['boxes'], [colors['LIO'], colors['VIO']]):
    patch.set_facecolor(color)
    patch.set_alpha(0.5)

ax2.set_title('B. Stability Analysis (Box Plot)', fontsize=14, fontweight='bold')
ax2.set_ylabel('Execution Time (ms)', fontsize=12)

# --- BIỂU ĐỒ C: HISTOGRAM (Mật độ phân phối) ---
ax3 = fig.add_subplot(gs[1, 1])
for mode in ['LIO', 'VIO']:
    dist_data = df[df['Mode'].str.contains(mode, na=False)]['Odom_Time_ms'].to_numpy()
    if len(dist_data) > 0:
        ax3.hist(dist_data, bins=40, alpha=0.5, label=mode, color=colors[mode], density=True)

ax3.set_title('C. Processing Time Density', fontsize=14, fontweight='bold')
ax3.set_xlabel('Execution Time (ms)', fontsize=12)
ax3.set_ylabel('Probability', fontsize=12)
ax3.legend()

plt.tight_layout()
plt.savefig("ieee_performance_report.png", dpi=300)
print("Thành công! Biểu đồ khoa học đã được lưu tại: ieee_performance_report.png")
plt.show()