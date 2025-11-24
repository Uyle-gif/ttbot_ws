#!/usr/bin/env python3
import numpy as np
import csv

# ==== THAM SỐ ĐƯỜNG U-TURN ====
L1 = 4.0      # chiều dài đoạn thẳng trước cua (m)
R  = 3.0      # bán kính cua (m) - rất êm, phù hợp giới hạn 30 độ
ds = 0.1      # bước lấy mẫu (m), càng nhỏ đường càng mượt

xs = []
ys = []

# ==== ĐOẠN 1: ĐI THẲNG THEO TRỤC X ====
s_vals = np.arange(0.0, L1, ds)
for s in s_vals:
    xs.append(s)
    ys.append(0.0)

# ==== ĐOẠN 2: CUNG TRÒN 180 ĐỘ ====
# Tâm cung tròn
xc, yc = L1, R

# Quay từ -90° đến +90° (đơn vị rad)
theta_vals = np.arange(-np.pi/2, np.pi/2 + np.deg2rad(2), np.deg2rad(2))
for th in theta_vals:
    x = xc + R * np.cos(th)
    y = yc + R * np.sin(th)
    xs.append(x)
    ys.append(y)

# ==== ĐOẠN 3: ĐI THẲNG SAU KHI QUAY ĐẦU ====
s_vals2 = np.arange(L1, -0.01, -ds)   # từ x = L1 → 0
for s in s_vals2:
    xs.append(s)
    ys.append(2*R)  # y = 2R = 6 m

# ==== GHI RA FILE path.csv ====
with open("path.csv", "w", newline="") as f:
    writer = csv.writer(f)
    for x, y in zip(xs, ys):
        writer.writerow([x, y])

print("Đã tạo path.csv với", len(xs), "điểm")
