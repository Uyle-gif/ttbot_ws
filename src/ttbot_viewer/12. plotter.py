#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import argparse
import os

def read_csv(filename, has_time=False):
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return [], []
    
    x_vals, y_vals = [], []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row: continue
            if has_time:
                x_vals.append(float(row[1]))
                y_vals.append(float(row[2]))
            else:
                x_vals.append(float(row[0]))
                y_vals.append(float(row[1]))
    return x_vals, y_vals

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=str, default="mpc")
    args = parser.parse_args()

    traj_file = f"{args.prefix}_traj.csv"
    ref_file = f"{args.prefix}_ref.csv"

    robot_x, robot_y = read_csv(traj_file, has_time=True)
    path_x, path_y = read_csv(ref_file, has_time=False)

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    
    ax.set_xlabel("Y (m)")
    ax.set_ylabel("X (m)")

    if path_x and path_y:
        ax.plot(path_y, path_x, 'k--', linewidth=1.0, label='Reference Path')
        ax.plot(path_y[0], path_x[0], 'ko', markerfacecolor='none', markersize=8, markeredgewidth=1.5, label='Path Start')
        ax.plot(path_y[-1], path_x[-1], 'kx', markersize=10, markeredgewidth=2, label='Path End')
    
    if robot_x and robot_y:
        ax.plot(robot_y, robot_x, 'r-', linewidth=1.2, label='Actual Robot Path')

    all_x = robot_x + path_x
    all_y = robot_y + path_y

    if all_x and all_y:
        padding = 2.0
        ax.set_xlim(max(all_y) + padding, min(all_y) - padding)
        ax.set_ylim(min(all_x) - padding, max(all_x) + padding)

    ax.legend(loc='upper right')
    
    fig_file = f"{args.prefix}_overlay.png"
    fig.savefig(fig_file, dpi=200, bbox_inches="tight")
    print(f"Saved plot to: {fig_file}")

    plt.show()

if __name__ == '__main__':
    main()