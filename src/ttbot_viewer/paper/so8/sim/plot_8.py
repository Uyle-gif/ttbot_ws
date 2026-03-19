#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import os
from pathlib import Path
import numpy as np

# ====== ĐƯỜNG DẪN CỐ ĐỊNH ======
BASE_PATH = Path("/home/vinh/ttbot_ws/src/ttbot_viewer/paper/so8/sim")

REF_FILE  = BASE_PATH / "gmpc_ref.csv"
GMPC_FILE = BASE_PATH / "gmpc_traj.csv"
MPC_FILE  = BASE_PATH / "mpc_traj.csv"


def read_ref_csv(filename):
    x_vals, y_vals = [], []

    if not Path(filename).exists():
        print(f"Không tìm thấy file: {filename}")
        return x_vals, y_vals

    with open(str(filename), 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            x_vals.append(float(row[0]))
            y_vals.append(float(row[1]))

    return x_vals, y_vals


def read_traj_csv(filename):
    x_vals, y_vals = [], []

    if not Path(filename).exists():
        print(f"Không tìm thấy file: {filename}")
        return x_vals, y_vals

    with open(str(filename), 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            x_vals.append(float(row[1]))
            y_vals.append(float(row[2]))

    return x_vals, y_vals


def calculate_tracking_error(traj_x, traj_y, ref_x, ref_y):
    """
    Tính toán khoảng cách ngắn nhất từ mỗi điểm thực tế đến quỹ đạo mẫu.
    Trả về RMSE, Mean Error, và Max Error.
    """
    if not traj_x or not ref_x:
        return 0.0, 0.0, 0.0
    
    traj_pts = np.array(list(zip(traj_x, traj_y)))
    ref_pts = np.array(list(zip(ref_x, ref_y)))
    
    errors = []
    for pt in traj_pts:
        dists = np.linalg.norm(ref_pts - pt, axis=1)
        errors.append(np.min(dists))
        
    errors = np.array(errors)
    rmse = np.sqrt(np.mean(errors**2))
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    
    return rmse, mean_error, max_error


def save_figure(fig, out_name="mpc_vs_gmpc_tracking_result"):
    out_dir = BASE_PATH
    out_dir.mkdir(parents=True, exist_ok=True)

    pdf_path = out_dir / f"{out_name}.pdf"
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.02)

    print(f"Đã lưu PDF vector: {pdf_path}")


def main():
    # Thiết lập PDF đẹp cho paper
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["ps.fonttype"] = 42
    plt.rcParams["svg.fonttype"] = "none"
    plt.rcParams["font.family"] = "DejaVu Sans"
    plt.rcParams["font.size"] = 14

    # Debug path để kiểm tra
    print(f"Ref file : {REF_FILE}")
    print(f"GMPC file: {GMPC_FILE}")
    print(f"MPC file : {MPC_FILE}")

    ref_x, ref_y   = read_ref_csv(REF_FILE)
    gmpc_x, gmpc_y = read_traj_csv(GMPC_FILE)
    mpc_x, mpc_y   = read_traj_csv(MPC_FILE)

    # ========================================================
    # TÍNH TOÁN VÀ IN SAI SỐ (Dùng để đưa vào Table)
    # ========================================================
    print("\n" + "="*50)
    print(" BẢNG TÍNH TOÁN SAI SỐ BÁM QUỸ ĐẠO")
    print("="*50)
    
    gmpc_rmse, gmpc_mean, gmpc_max = calculate_tracking_error(gmpc_x, gmpc_y, ref_x, ref_y)
    mpc_rmse, mpc_mean, mpc_max = calculate_tracking_error(mpc_x, mpc_y, ref_x, ref_y)
    
    if gmpc_x:
        print(f"[GMPC] RMSE: {gmpc_rmse:.4f} m | Mean: {gmpc_mean:.4f} m | Max: {gmpc_max:.4f} m")
    if mpc_x:
        print(f"[MPC]  RMSE: {mpc_rmse:.4f} m | Mean: {mpc_mean:.4f} m | Max: {mpc_max:.4f} m")
    print("="*50 + "\n")

    # ========================================================
    # VẼ ĐỒ THỊ
    # ========================================================
    fig, ax = plt.subplots(figsize=(7.2, 7.2))

    # Reference path
    if ref_x and ref_y:
        ax.plot(ref_y, ref_x, 'k--', linewidth=2.0, label='Reference Path')

        ax.plot(
            ref_y[0], ref_x[0],
            'ko',
            markerfacecolor='none',
            markersize=10,
            markeredgewidth=2.0,
            label='Path Start'
        )

        ax.plot(
            ref_y[-1], ref_x[-1],
            'kx',
            markersize=12,
            markeredgewidth=2.5,
            label='Path End'
        )

    # GMPC
    if gmpc_x and gmpc_y:
        ax.plot(gmpc_y, gmpc_x, 'r-', linewidth=3.0, label='GMPC')

    # MPC
    if mpc_x and mpc_y:
        ax.plot(mpc_y, mpc_x, 'b-', linewidth=3.0, label='MPC')

    # Đảo trục ngang như code cũ
    ax.set_xlim(10, -10)
    ax.set_ylim(-1.5, 18.5)

    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel("Y (m)", fontsize=20)
    ax.set_ylabel("X (m)", fontsize=20)
    
    # Legend hiển thị gọn gàng, không có text dư thừa
    ax.legend(loc='upper right', fontsize=11, frameon=True)
    ax.set_aspect('equal', adjustable='box')
    ax.tick_params(labelsize=13)

    plt.tight_layout()

    save_figure(fig, out_name="mpc_vs_gmpc_tracking_result")
    plt.show()

if __name__ == '__main__':
    main()