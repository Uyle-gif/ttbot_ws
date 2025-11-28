import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # =========================================
    # 1. KHAI BÁO THAM SỐ HỆ THỐNG
    # =========================================
    
    # --- QUAN TRỌNG: ĐỒNG BỘ THỜI GIAN ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Sử dụng thời gian mô phỏng từ Gazebo"
    )

    # =========================================
    # 2. KHAI BÁO THAM SỐ FILE PATH
    # =========================================
    
    # Lấy đường dẫn tuyệt đối đến thư mục share của package
    # Giả sử file csv nằm trong folder: ttbot_controller/path/
    pkg_share = get_package_share_directory('ttbot_controller')
    default_csv_path = os.path.join(pkg_share, 'path', 'path_l.csv')

    path_file_arg = DeclareLaunchArgument(
        "path_file",
        default_value=default_csv_path,
        description="Đường dẫn tuyệt đối tới file path CSV"
    )

    # =========================================
    # 3. CÁC THAM SỐ MPC & ROBOT
    # =========================================
    desired_speed_arg = DeclareLaunchArgument("desired_speed", default_value="1.5", description="Vận tốc xe (m/s)")
    wheel_base_arg = DeclareLaunchArgument("wheel_base", default_value="0.8", description="Chiều dài trục bánh xe")
    max_steer_deg_arg = DeclareLaunchArgument("max_steer_deg", default_value="30.0", description="Giới hạn góc lái (độ)")

    # Tham số Controller
    Np_arg = DeclareLaunchArgument("N_p", default_value="10", description="Prediction Horizon")
    dt_mpc_arg = DeclareLaunchArgument("dt_mpc", default_value="0.1", description="Bước thời gian MPC (s)")
    
    # Weights
    Q_ey_arg = DeclareLaunchArgument("Q_ey", default_value="10.0", description="Trọng số lỗi ngang e_y")
    Q_epsi_arg = DeclareLaunchArgument("Q_epsi", default_value="5.0", description="Trọng số lỗi góc e_psi")
    R_delta_arg = DeclareLaunchArgument("R_delta", default_value="1.0", description="Trọng số độ lớn góc lái delta")

    # =========================================
    # 4. CẤU HÌNH NODE MPC
    # =========================================
    mpc_node = Node(
        package="ttbot_controller",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[{
            "use_sim_time":   LaunchConfiguration("use_sim_time"), # <--- FIX LỖI TIME
            
            "path_file":      LaunchConfiguration("path_file"),
            "desired_speed":  LaunchConfiguration("desired_speed"),
            "wheel_base":     LaunchConfiguration("wheel_base"),
            "max_steer_deg":  LaunchConfiguration("max_steer_deg"),

            "N_p":            LaunchConfiguration("N_p"),
            "dt_mpc":         LaunchConfiguration("dt_mpc"),
            "Q_ey":           LaunchConfiguration("Q_ey"),
            "Q_epsi":         LaunchConfiguration("Q_epsi"),
            "R_delta":        LaunchConfiguration("R_delta"),
        }],
        # remappings=[
        #     ('/cmd_vel', '/diff_cont/cmd_vel_unstamped'), # Ví dụ nếu robot dùng topic khác
        #     ('/odom', '/odometry/filtered')               # Nên lắng nghe odom từ EKF
        # ]
    )

    # =========================================
    # 5. RETURN
    # =========================================
    return LaunchDescription([
        use_sim_time_arg,
        path_file_arg,
        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        Np_arg,
        dt_mpc_arg,
        Q_ey_arg,
        Q_epsi_arg,
        R_delta_arg,
        mpc_node
    ])