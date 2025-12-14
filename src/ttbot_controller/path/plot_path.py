#!/usr/bin/env python3
import sys
import csv
import matplotlib.pyplot as plt
import numpy as np

# --- Load Data ---
if len(sys.argv) > 1:
    path_file = sys.argv[1]
else:
    path_file = "gps_path.csv"

xs_list = []
ys_list = []

try:
    with open(path_file) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row: continue
            xs_list.append(float(row[0]))
            ys_list.append(float(row[1]))
except FileNotFoundError:
    print("File not found. Using dummy data.")
    # Dummy figure-8 data
    t = np.linspace(0, 2*np.pi, 100)
    xs_list = 4 * np.sin(t)
    ys_list = 2 * np.sin(2*t)

xs_np = np.array(xs_list)
ys_np = np.array(ys_list)

# --- Plotting Setup ---
fig, ax = plt.subplots(figsize=(8, 8))

plot_h = ys_np # Y axis data (horizontal)
plot_v = xs_np # X axis data (vertical)

# 1. Draw path with specific name
ax.plot(plot_h, plot_v, "-b", linewidth=1.5, alpha=0.5, label="path")

# 2. Draw NEAT & CLEAR arrows
if len(plot_h) > 2:
    # Calculate direction vectors
    u_raw = np.diff(plot_h)
    v_raw = np.diff(plot_v)
    pos_h = plot_h[:-1]
    pos_v = plot_v[:-1]

    # Normalize vectors (make all arrows same length)
    norm = np.sqrt(u_raw**2 + v_raw**2)
    norm[norm == 0] = 1
    u_norm = u_raw / norm
    v_norm = v_raw / norm

    # Sample points (adjust '12' to change density)
    step = max(1, len(pos_h) // 12)

    # --- ARROW STYLING ---
    ax.quiver(pos_h[::step], pos_v[::step], u_norm[::step], v_norm[::step],
              angles='xy', scale_units='width',
              scale=35,            # Higher number = Shorter/Smaller arrows (Try 30-40)
              width=0.006,         # Thinner shaft for a "neater" look
              headwidth=4.5,       # Width of the arrow head
              headlength=5,        # Length of the arrow head
              color='blue', pivot='mid', zorder=5)

# 3. Markers
ax.plot(plot_h[0], plot_v[0], 'go', markersize=10, label='Start', zorder=6, markeredgecolor='black')
ax.plot(plot_h[-1], plot_v[-1], 'rX', markersize=10, label='End', zorder=6, markeredgecolor='black')

# --- Axis & Grid ---
ax.set_ylabel("x", rotation=0, loc='top', fontsize=12, fontweight='bold')
ax.set_xlabel("y", loc='left', fontsize=12, fontweight='bold')

# Invert horizontal axis (Y)
ax.invert_xaxis()

# Center spines
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')

ax.grid(True, linestyle='--', alpha=0.5)
ax.legend(loc='upper right')
ax.set_title("path", y=1.02) # Set Title explicitly
ax.axis("equal")

plt.show()