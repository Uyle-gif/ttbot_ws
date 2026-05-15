#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as mtransforms
from matplotlib.animation import FuncAnimation
import threading
import csv
import argparse
import os
import math

class PathTrackingEvalAndRecord(Node):
    def __init__(self, prefix: str, odom_topic: str, path_topic: str, save_dir: str):
        super().__init__('path_tracking_eval_and_record')

        self.prefix = prefix
        self.odom_topic = odom_topic
        self.path_topic = path_topic
        self.save_dir = save_dir  

        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.sub_path = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )

        self.robot_data = []
        self.path_xy = []

        self.lock = threading.Lock()
        self.path_received = False
        self.saved = False

        self.get_logger().info(
            f"Waiting for data from {self.path_topic} and {self.odom_topic}..."
        )
        self.get_logger().info(f"Dữ liệu sẽ được lưu tại thư mục: {self.save_dir}")

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        with self.lock:
            self.robot_data.append((
                t, 
                pos.x, pos.y, pos.z, 
                ori.x, ori.y, ori.z, ori.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z
            ))

    def path_callback(self, msg: Path):
        with self.lock:
            self.path_xy = [
                (p.pose.position.x, p.pose.position.y)
                for p in msg.poses
            ]

            if not self.path_received and self.path_xy:
                self.path_received = True
                self.get_logger().info(f"Received path: {len(self.path_xy)} points")

    def save_results(self, fig=None):
        if self.saved:
            return

        with self.lock:
            traj_file = os.path.join(self.save_dir, "traj.csv")
            ref_file = os.path.join(self.save_dir, "ref.csv")
            fig_file = os.path.join(self.save_dir, "overlay.png")

            with open(traj_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "t", "x", "y", "z", 
                    "qx", "qy", "qz", "qw", 
                    "vx", "vy", "vz", 
                    "wx", "wy", "wz"
                ])
                writer.writerows(self.robot_data)

            with open(ref_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(self.path_xy)

            if fig is not None:
                fig.savefig(fig_file, dpi=200, bbox_inches="tight")

        self.saved = True
        self.get_logger().info(f"Saved: {traj_file}")
        self.get_logger().info(f"Saved: {ref_file}")
        if fig is not None:
            self.get_logger().info(f"Saved: {fig_file}")


def update_plot(frame, node, line_robot, line_path, ax, car_patch, v_text):
    with node.lock:
        robot_x = [p[1] for p in node.robot_data]
        robot_y = [p[2] for p in node.robot_data]
        path_x = [p[0] for p in node.path_xy]
        path_y = [p[1] for p in node.path_xy]

        current_v = 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        if node.robot_data:
            current_vx = node.robot_data[-1][8]
            current_vy = node.robot_data[-1][9]
            current_v = math.sqrt(current_vx**2 + current_vy**2)
            
            qx = node.robot_data[-1][4]
            qy = node.robot_data[-1][5]
            qz = node.robot_data[-1][6]
            qw = node.robot_data[-1][7]

    # KHÓA CHẶT: Nếu chưa nhận được MPC Path, đồ thị đứng im
    if not path_x:
        return line_robot, line_path, car_patch, v_text

    line_robot.set_data(robot_y, robot_x)
    line_path.set_data(path_y, path_x)

    if robot_x and robot_y:
        # Bật hiển thị xe và Text khi ĐÃ CÓ DATA ODOM
        car_patch.set_visible(True)
        v_text.set_visible(True)
        v_text.set_text(f"Velocity: {current_v:.2f} m/s")

        # Tính góc Yaw và xoay xe
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        angle_deg = math.degrees(yaw)

        car_width, car_length = 0.6, 1.2
        tr = mtransforms.Affine2D().rotate_deg_around(0, 0, -angle_deg)
        tr += mtransforms.Affine2D().translate(robot_y[-1], robot_x[-1])
        tr += ax.transData
        
        car_patch.set_transform(tr)
        car_patch.set_xy((-car_width/2, -car_length/2))

    # ==========================================
    # THUẬT TOÁN TỰ ĐỘNG CÂN CHỈNH 16:9 THEO TÂM QUỸ ĐẠO
    # ==========================================
    all_x = path_x + robot_x
    all_y = path_y + robot_y
    
    if all_x and all_y:
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        range_x = max_x - min_x
        range_y = max_y - min_y
        
        center_x = (max_x + min_x) / 2
        center_y = (max_y + min_y) / 2
        
        padding = 1.35 
        
        if range_y / max(range_x, 0.001) > 16/9:
            width = range_y * padding
            height = width * 9 / 16
        else:
            height = range_x * padding
            width = height * 16 / 9

        # Căn giữa và tự động đảo chiều trục Y (từ lớn -> nhỏ)
        ax.set_xlim(center_y + width/2, center_y - width/2) 
        ax.set_ylim(center_x - height/2, center_x + height/2)

    return line_robot, line_path, car_patch, v_text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=str, default="data") 
    parser.add_argument("--odom_topic", type=str, default="/mpc_state")
    parser.add_argument("--path_topic", type=str, default="/mpc_path")
    parser.add_argument("--base_dir", type=str, default=os.path.expanduser("~/ttbot_ws/src/ttbot_viewer/paper"))
    args, unknown = parser.parse_known_args()

    os.makedirs(args.base_dir, exist_ok=True)

    counter = 1
    save_dir = os.path.join(args.base_dir, f"{args.prefix}_{counter}")
    while os.path.exists(save_dir):
        counter += 1
        save_dir = os.path.join(args.base_dir, f"{args.prefix}_{counter}")
    
    os.makedirs(save_dir)

    rclpy.init(args=unknown)
    node = PathTrackingEvalAndRecord(
        prefix=args.prefix,
        odom_topic=args.odom_topic,
        path_topic=args.path_topic,
        save_dir=save_dir
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # THIẾT LẬP TOÀN CỤC CHUẨN VIDEO
    plt.rcParams["font.family"] = "DejaVu Sans"
    plt.rcParams["font.size"] = 14
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_aspect('equal')
    
    # Lưới mờ nhẹ y hệt ảnh
    ax.grid(True, linestyle=':', alpha=0.4)
    
    ax.set_xlabel("Y (m)", fontsize=18)
    ax.set_ylabel("X (m)", fontsize=18)

    # 1. NÉT VẼ ĐƯỜNG: Đen nét đứt (Ref) và Đỏ nét liền (GMPC) - Nét siêu dày 3.5
    line_path, = ax.plot([], [], 'k--', linewidth=3.5, label='Reference Path', zorder=1)
    line_robot, = ax.plot([], [], 'b-', linewidth=3.5, label='MPC Trajectory', zorder=2)
    
    # 2. Ô TÔ
    car_width, car_length = 0.6, 1.2
    car_patch = patches.Rectangle((0, 0), car_width, car_length, color='red', ec='black', lw=1.5, zorder=5)
    car_patch.set_visible(False) 
    ax.add_patch(car_patch)

    # 3. TEXT VẬN TỐC
    v_text = ax.text(0.02, 0.94, 'Velocity: 0.00 m/s', transform=ax.transAxes, fontsize=22, 
                     fontweight='bold', color='yellow',
                     bbox=dict(facecolor='black', alpha=0.6, edgecolor='none', boxstyle='round,pad=0.5'))
    v_text.set_visible(False)

    # Khởi tạo mặc định góc nhìn an toàn trước khi nhận dữ liệu
    ax.set_xlim(10, -10)
    ax.set_ylim(-10, 10)

    ax.legend(loc='upper right')
    plt.tight_layout()

    ani = FuncAnimation(
        fig,
        update_plot,
        fargs=(node, line_robot, line_path, ax, car_patch, v_text), 
        interval=100, # Update nhanh hơn cho mượt
        blit=False,
        cache_frame_data=False
    )

    def on_close(event):
        node.save_results(fig)

    fig.canvas.mpl_connect("close_event", on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results(fig)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()