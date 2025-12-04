import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Trỏ đúng vào file config trong package ttbot_localization
    config_file = os.path.join(
        get_package_share_directory('ttbot_localization'),
        'config',
        'neo_m8p.yaml'
    )

    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('fix', '/gps/fix') # Remap để Navsat Transform nhận được
            ]
        )
    ])