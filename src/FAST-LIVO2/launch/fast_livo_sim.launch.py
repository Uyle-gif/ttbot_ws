#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    # Giữ nguyên rviz config chuẩn của tác giả
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2_sim.rviz")

    # Load parameters (Đổi tên trỏ về 2 file yaml của bạn)
    ttbot_config_cmd = os.path.join(config_file_dir, "ttbot_sim.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_sim.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True", # Đổi thành True để mặc định mở RViz xem cho tiện
        description="Whether to launch Rviz2",
    )

    ttbot_config_arg = DeclareLaunchArgument(
        'ttbot_params_file',
        default_value=ttbot_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for camera intrinsics',
    )

    # https://github.com/ros-navigation/navigation2/blob/1c68c212db01f9f75fcb8263a0fbb5dfa711bdea/nav2_bringup/launch/navigation_launch.py#L40
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    ttbot_params_file = LaunchConfiguration('ttbot_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        ttbot_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # [GHI CHÚ MÔ PHỎNG]: Tác giả dùng node 'republish' dưới đây để giải nén ảnh (từ compressed sang raw) khi chạy rosbag.
        # Vì Gazebo của bạn đã xuất thẳng ảnh thô (raw) ra topic /camera/rgbd/image, 
        # nên tôi đã comment đoạn này lại để tránh hệ thống bị treo do chờ đợi topic compressed không tồn tại.
        #
        # Node(
        #     package="image_transport",
        #     executable="republish",
        #     name="republish",
        #     arguments=[ 
        #         'compressed', 
        #         'raw',
        #     ],
        #     remappings=[
        #         ("in",  "/camera/rgbd/image"), 
        #         ("out", "/camera/rgbd/image")
        #     ],
        #     output="screen",
        #     respawn=use_respawn,
        # ),
        
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                ttbot_params_file,
                camera_params_file,
                {'use_sim_time': True} # Bắt buộc phải thêm dòng này để đồng bộ thời gian với Gazebo
            ],
            prefix=[
                # ("gdb -ex run --args"),
                # ("valgrind --log-file=./valgrind_report.log --tool=memcheck --leak-check=full --show-leak-kinds=all -s --track-origins=yes --show-reachable=yes --undef-value-errors=yes --track-fds=yes")
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[
                {'use_sim_time': True} # Đồng bộ thời gian cho RViz để TF không bị lỗi
            ],
            output="screen"
        ),
    ])