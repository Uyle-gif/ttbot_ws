#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import csv
from pathlib import Path
import numpy as np

# ====== ĐƯỜNG DẪN CỐ ĐỊNH ======
BASE_PATH = Path(__file__).resolve().parent

REF_FILE  = BASE_PATH / "gmpc_ref.csv"
GMPC_FILE = BASE_PATH / "gmpc_traj.csv"

# ========================================================
# HÀM ĐỌC DỮ LIỆU
# ========================================================
def read_ref_csv(filename):
    x_vals, y_vals = [], []
    with open(str(filename), 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row: continue
            x_vals.append(float(row[0]))
            y_vals.append(float(row[1]))
    return x_vals, y_vals

def read_traj_csv(filename):
    x_vals, y_vals = [], []
    with open(str(filename), 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row: continue
            x_vals.append(float(row[1]))
            y_vals.append(float(row[2]))
    return x_vals, y_vals

def main():
    # 1. THIẾT LẬP STYLE
    plt.rcParams["font.family"] = "DejaVu Sans"
    plt.rcParams["font.size"] = 14

    print(f"Đang đọc dữ liệu GMPC...")
    ref_x, ref_y   = read_ref_csv(REF_FILE)
    gmpc_x, gmpc_y = read_traj_csv(GMPC_FILE)

    if not gmpc_x:
        print("LỖI: Không có dữ liệu.")
        return

    # 2. XỬ LÝ ĐỒNG BỘ THỜI GIAN THỰC (REAL-TIME SYNC)
    # Lấy toàn bộ dữ liệu, không bỏ qua frame nào
    step = 1  
    fps_camera = 30.0 # Giả sử camera quay ở 30 FPS, ROS bag lưu ở ~30Hz
    
    gmpc_x_arr = np.array(gmpc_x)[::step]
    gmpc_y_arr = np.array(gmpc_y)[::step]
    num_frames = len(gmpc_x_arr)

    # 3. KHỞI TẠO ĐỒ THỊ
    fig, ax = plt.subplots(figsize=(7.2, 7.2))

    # Nền: Reference path
    if ref_x and ref_y:
        ax.plot(ref_y, ref_x, 'k--', linewidth=2.0, label='Reference', alpha=0.6)
        ax.plot(ref_y[0], ref_x[0], 'ko', markerfacecolor='none', markersize=10, markeredgewidth=2.0)
        ax.plot(ref_y[-1], ref_x[-1], 'kx', markersize=12, markeredgewidth=2.5)

    # Động: GMPC
    line_gmpc, = ax.plot([], [], 'r-', linewidth=3.0, label='GMPC Trajectory')
    robot_gmpc, = ax.plot([], [], 'ro', markersize=10, markeredgecolor='black')

    # THÊM ĐỒNG HỒ ĐỒNG BỘ TRÊN ĐỒ THỊ (Góc trên bên trái)
    time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=16, 
                        fontweight='bold', color='darkred',
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))

    # Giới hạn khung hình
    ax.set_xlim(10, -10)
    ax.set_ylim(-1.5, 18.5)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel("Y (m)", fontsize=16)
    ax.set_ylabel("X (m)", fontsize=16)
    ax.legend(loc='lower left', fontsize=12, frameon=True) # Dời legend xuống để nhường chỗ cho đồng hồ
    ax.set_aspect('equal', adjustable='box')
    plt.tight_layout()

    # 4. HÀM CẬP NHẬT FRAME
    def update(frame):
        # Tính toán thời gian thực đang trôi qua
        current_time = frame * (1.0 / fps_camera)
        time_text.set_text(f'Time: {current_time:.2f} s')

        # Cập nhật đường đi và robot
        line_gmpc.set_data(gmpc_y_arr[:frame+1], gmpc_x_arr[:frame+1])
        robot_gmpc.set_data([gmpc_y_arr[frame]], [gmpc_x_arr[frame]])

        if frame % 100 == 0:
            print(f"Đang render frame {frame}/{num_frames} ({current_time:.1f}s)...")

        return line_gmpc, robot_gmpc, time_text

    # 5. TẠO VÀ LƯU VIDEO
    print("\nBắt đầu tạo Video Animation (Tốc độ 1x)...")
    # interval = 1000ms / 30fps = ~33.33ms
    interval_ms = 1000 / fps_camera 
    ani = FuncAnimation(fig, update, frames=num_frames, interval=interval_ms, blit=True)

    out_dir = BASE_PATH
    out_dir.mkdir(parents=True, exist_ok=True)
    gif_path = out_dir / "gmpc_realtime_sync.gif"

    print(f"Đang lưu file ảnh động thời gian thực: {gif_path}")
    # Chạy lưu ở đúng 30 FPS
    ani.save(str(gif_path), writer=PillowWriter(fps=fps_camera))
    print("Hoàn tất! Video hiện tại chạy với tốc độ 1:1 so với thực tế.")

if __name__ == '__main__':
    main()