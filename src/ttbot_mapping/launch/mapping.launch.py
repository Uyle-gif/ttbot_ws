import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Đường dẫn đến file config nếu bạn muốn tách riêng sau này
    # config_file = os.path.join(get_package_share_directory('ttbot_mapping'), 'config', 'grid_params.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            name='pointcloud_to_grid_node',
            output='screen',
            parameters=[{
                # 1. Topic đầu vào: Khớp với topic Velodyne trong mô phỏng/thực tế của ttbot
                'input_topic': '/velodyne_points/points', 
                
                # 2. Khung tọa độ mục tiêu: 
                # Nếu chạy mô phỏng không có FAST_LIO, hãy dùng 'base_link' hoặc 'odom'
                'target_frame': 'odom', 
                
                # 3. Cấu hình bản đồ đầu ra
                'grid_ros_topic_name': '/grid',      # Topic OccupancyGrid cho Nav2
                'resolution': 0.1,                  # Độ phân giải: 0.1m (10cm) mỗi ô lưới
                
                # 4. Lọc độ cao (Z-axis): Giúp loại bỏ mặt đất và trần nhà
                'min_z': -0.1,                      # Loại bỏ các điểm thấp hơn LiDAR 10cm (mặt đất)
                'max_z': 0.5,                       # Chỉ lấy các vật cản cao tối đa 50cm so với LiDAR
                
                # 5. Tính năng nâng cao
                'use_intensity': True,              # Sử dụng cường độ phản xạ của Velodyne
                'filter_intensity': False,          # Có lọc nhiễu theo cường độ hay không
                
                # 6. Cấu hình QoS cho GridMap (nếu dùng)
                'mapi_gridmap_topic_name': 'intensity_gridmap',
                'maph_gridmap_topic_name': 'height_gridmap',
            }]
        )
    ])