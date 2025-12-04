from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # <--- Thêm cái này
from launch.substitutions import LaunchConfiguration # <--- Thêm cái này
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # 1. KHAI BÁO NHẬN THAM SỐ TỪ BÊN NGOÀI
    # Nếu không ai truyền vào thì mặc định là False (an toàn cho robot thật)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 2. SỬA LẠI BIẾN NÀY ĐỂ DÙNG THAM SỐ ĐỘNG
    # Thay vì True/False cứng, ta đưa biến use_sim_time vào
    common_params = [{"use_sim_time": use_sim_time}]
    
    config_file = os.path.join(
        get_package_share_directory("ttbot_localization"), 
        "config", 
        "dual_ekf.yaml"
    )
    # --- TF: IMU -> BASE_LINK ---
    static_transform_publisher_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu_link"],
        parameters=common_params 
    )

    # --- TF: GPS -> BASE_LINK (Bổ sung thêm) ---
    # Cần thiết để navsat_transform biết GPS nằm đâu
    # Ở đây mình để tạm là cao hơn tâm xe 0.2m, bạn chỉnh lại --x --y --z cho đúng thực tế
    static_transform_publisher_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.2",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "gps_link"], 
        parameters=common_params 
    )

    # --- NODE 1: LOCAL EKF (Odom -> Base_link) ---
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("ttbot_localization"), "config", "ekf.yaml"),
            common_params[0] # Merge dict use_sim_time vào
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/local') # Đổi tên output local
        ]
    )
    # --- NODE 2: GLOBAL EKF (Map -> Odom) ---
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global", # Tên này khớp với key thứ 2 trong dual_ekf.yaml
        output="screen",
        parameters=[
            config_file,
            common_params[0]
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/global') # Đổi tên output global
        ]
    )
    # --- NODE 3: NAVSAT TRANSFORM (GPS -> Odom Message) ---
    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[
            config_file,
            common_params[0]
        ],
        remappings=[
            # Input (Kiểm tra kỹ tên topic GPS và IMU của bạn)
            ('/gps/fix', '/gps/fix'),       
            ('/imu/data', '/imu/data_filtered'), # Lưu ý: Node này cần IMU gốc hoặc filtered đều được, miễn có hướng
            
            # Input Odom: Cần lấy odom global để khởi tạo vị trí ban đầu
            ('/odometry/filtered', '/odometry/global'), 
            
            # Output: Topic này sẽ được Global EKF (odom1) hứng lấy
            ('/odometry/gps', '/odometry/gps') 
        ]
    )





    imu_republisher_cpp = Node(
        package="ttbot_localization",
        executable="imu_republisher",
        parameters=common_params
    )

    return LaunchDescription([
        # 3. QUAN TRỌNG: PHẢI KHAI BÁO ARGUMENT Ở ĐÂY
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        static_transform_publisher_imu,
        static_transform_publisher_gps, 
        ekf_local,       
        ekf_global,
        navsat_transform,
        imu_republisher_cpp,   
    ])