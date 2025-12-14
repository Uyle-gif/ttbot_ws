import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Tên file csv sẽ được lưu/đọc trong thư mục 'path' của package
    path_file_arg = DeclareLaunchArgument(
        "path_file", default_value="path_gps.csv"
    )

    gps_path_node = Node(
        package="ttbot_controller",
        executable="gps_path_publisher",
        name="gps_path_publisher",
        output="screen",
        parameters=[{
            "path_file": LaunchConfiguration("path_file"),
            "frame_id": "map",
            # Tọa độ gốc phải khớp với file World GPS
            "origin_lat": 10.7769, 
            "origin_lon": 106.7009
        }]
    )

    return LaunchDescription([
        path_file_arg,
        gps_path_node
    ])