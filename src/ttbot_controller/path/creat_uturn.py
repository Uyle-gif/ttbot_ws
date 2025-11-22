import numpy as np
import csv

# Tham số đường
L1 = 5.0      # m, đoạn thẳng trước U-turn
R = 2.0       # m, bán kính quay đầu
ds = 0.1      # bước lấy mẫu, càng nhỏ đường càng mượt

xs = []
ys = []

# ---- ĐOẠN 1: đi thẳng ----
s_vals = np.arange(0.0, L1, ds)
for s in s_vals:
    xs.append(s)
    ys.append(0.0)

# ---- ĐOẠN 2: cung tròn 180 độ ----
xc, yc = L1, R
theta_vals = np.arange(-np.pi/2, np.pi/2 + 0.01, np.deg2rad(2))  # mỗi 2°
for th in theta_vals:
    x = xc + R * np.cos(th)
    y = yc + R * np.sin(th)
    xs.append(x)
    ys.append(y)

# ---- ĐOẠN 3: đi thẳng sau khi quay đầu ----
s_vals2 = np.arange(L1, -0.01, -ds)   # đi lùi về x=0
for s in s_vals2:
    xs.append(s)
    ys.append(2*R)

# ---- Ghi ra CSV ----
with open("path_uturn.csv", "w", newline="") as f:
    writer = csv.writer(f)
    for x, y in zip(xs, ys):
        writer.writerow([x, y])

print("Đã tạo path_uturn.csv với", len(xs), "điểm")
