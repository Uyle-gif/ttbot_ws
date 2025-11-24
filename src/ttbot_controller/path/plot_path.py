#!/usr/bin/env python3
import sys
import csv
import matplotlib.pyplot as plt

if len(sys.argv) > 1:
    path_file = sys.argv[1]
else:
    path_file = "path_right.csv"

xs = []
ys = []

with open(path_file) as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        xs.append(float(row[0]))
        ys.append(float(row[1]))

# ==== ĐỔI TRỤC (X <-> Y) ====
plt.plot(ys, xs, "-o")

# ==== Nếu muốn lật Y lại cho cùng chiều ROS, bật dòng dưới ====
# plt.gca().invert_yaxis()

plt.xlabel("Y (m)")
plt.ylabel("X (m)")
plt.axis("equal")
plt.grid(True)
plt.title(path_file)
plt.show()