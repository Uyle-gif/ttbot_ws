import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # =========================================
    # 1. KHAI BÁO THAM SỐ (Arguments)
    # =========================================
    
    # --- Đường dẫn file path ---
    path_file_arg = DeclareLaunchArgument(
        "path_file",
        default_value="path_uturn.csv",
        description="Tên file CSV trong thư mục share/ttbot_controller/path/"
    )

    # --- Thông số vật lý xe & Vận tốc ---
    desired_speed_arg = DeclareLaunchArgument(
        "desired_speed",
        default_value="1.5",
        description="Vận tốc mong muốn (m/s)"
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base",
        default_value="0.8",
        description="Khoảng cách trục bánh xe (m)"
    )

    max_steer_deg_arg = DeclareLaunchArgument(
        "max_steer_deg",
        default_value="60.0",
        description="Góc lái tối đa (độ)"
    )

    # --- Tham số MPC (Quan trọng để Tune) ---
    # Horizon & Time step
    Np_arg = DeclareLaunchArgument(
        "N_p",
        default_value="20",
        description="Prediction Horizon (Số bước dự đoán)"
    )
    
    dt_mpc_arg = DeclareLaunchArgument(
        "dt_mpc",
        default_value="0.1",
        description="Bước thời gian rời rạc hóa (s)"
    )

    # Trọng số Cost Function
    Q_x_arg = DeclareLaunchArgument(
        "Q_x",
        default_value="10.0",
        description="Trọng số phạt sai số vị trí (x, y)"
    )

    Q_psi_arg = DeclareLaunchArgument(
        "Q_psi",
        default_value="50.0",
        description="Trọng số phạt sai số góc hướng (yaw)"
    )

    R_omega_arg = DeclareLaunchArgument(
        "R_omega",
        default_value="1.0",
        description="Trọng số phạt tín hiệu điều khiển (omega) - Giúp xe đi mượt hơn"
    )

    # =========================================
    # 2. CẤU HÌNH NODE
    # =========================================
    mpc_node = Node(
        package="ttbot_controller",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[{
            # Map từ LaunchConfiguration vào ROS Parameters
            "path_file": LaunchConfiguration("path_file"),
            "desired_speed": LaunchConfiguration("desired_speed"),
            "wheel_base": LaunchConfiguration("wheel_base"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            
            "N_p": LaunchConfiguration("N_p"),
            "dt_mpc": LaunchConfiguration("dt_mpc"),
            "Q_x": LaunchConfiguration("Q_x"),
            "Q_psi": LaunchConfiguration("Q_psi"),
            "R_omega": LaunchConfiguration("R_omega"),
        }]
    )

    return LaunchDescription([
        # Load các arguments
        path_file_arg,
        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        Np_arg,
        dt_mpc_arg,
        Q_x_arg,
        Q_psi_arg,
        R_omega_arg,
        
        # Chạy node
        mpc_node
    ])