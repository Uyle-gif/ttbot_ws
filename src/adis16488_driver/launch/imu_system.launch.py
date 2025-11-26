import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. DRIVER NODE (ADIS16488)
        # Nhiệm vụ: Đọc dữ liệu từ cổng USB và gửi lên topic
        # ---------------------------------------------------------
        Node(
            package='adis16488_driver',
            executable='adis16488_node',
            name='adis16488_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB1'},  # Cổng kết nối
                {'baudrate': 460800},      # Baudrate (phải khớp với code C++)
                {'frame_id': 'imu_link'}
            ]
            # Output mặc định của Driver (theo code C++):
            # - /imu/data (Gia tốc + Gyro + Góc thô)
            # - /imu/mag  (Từ trường)
        ),

        # ---------------------------------------------------------
        # 2. FILTER NODE (Madgwick)
        # Nhiệm vụ: Hợp nhất dữ liệu để tính Orientation chuẩn hơn
        # ---------------------------------------------------------
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[
                {'use_mag': True},       # Dùng từ trường để sửa hướng Yaw
                {'publish_tf': False},   # Tắt publish TF (để robot_localization lo)
                {'world_frame': 'enu'},  # Chuẩn toạ độ ROS
                {'fixed_frame': 'odom'},
                {'gain': 0.1},           # Hệ số tin tưởng Gyro
            ],
            remappings=[
                # [QUAN TRỌNG] Nối dây:
                # Input của bộ lọc (imu/data_raw) <---lấy từ--- Output của Driver (/imu/data)
                ('imu/data_raw', '/imu/data_raw'),
                
                # Input từ trường của bộ lọc (imu/mag) <---lấy từ--- Output từ trường Driver (/imu/mag)
                ('imu/mag', '/imu/mag'),
                
                # Output cuối cùng của bộ lọc (Dữ liệu sạch)
                ('imu/data', '/imu/data_filtered') 
            ]
        )
    ])