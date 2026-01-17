import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    imu_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttbot_imu',
        description='Serial port for IMU sensor'
    )
    
    imu_driver_node = Node(
        package='ttbot_sensors',
        executable='adis16488_node',
        name='adis16488_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 460800,
            'frame_id': 'imu_link',
            'publish_rate': 50.0  # Giới hạn 50Hz
        }]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': True,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'odom',
            'gain': 0.1,
        }],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/mag', '/imu/mag'),
            ('imu/data', '/imu/data_filtered') 
        ]
    )

    return LaunchDescription([
        imu_port_arg,
        imu_driver_node,
        imu_filter_node
    ])