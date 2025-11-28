import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Tên file mặc định
    path_file_arg = DeclareLaunchArgument(
        "path_file", default_value="path_8.csv",
        description="Name of the CSV file inside 'path' folder"
    )

    path_node = Node(
        package="ttbot_controller",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[{
            "path_file": LaunchConfiguration("path_file"),
            "frame_id": "map",       # Hoặc 'odom' tùy hệ thống của bạn
            "publish_rate": 1.0      # 1 Hz là đủ cho đường tĩnh
        }]
    )

    return LaunchDescription([
        path_file_arg,
        path_node
    ])