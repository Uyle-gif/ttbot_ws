import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_port = DeclareLaunchArgument(
        'port', 
        default_value='/dev/ttbot_gps',
        description='Serial port for GPS'
    )
    
    arg_baud = DeclareLaunchArgument(
        'baud', 
        default_value='19200', 
        description='Baudrate for GPS'
    )
    
    arg_frame_id = DeclareLaunchArgument(
        'frame_id', 
        default_value='gps_link', 
        description='TF Frame ID for GPS'
    )

    # nmea_navsat_driver
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='ublox_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
            'time_ref_source': 'gps', 
            'useRMC': False           
        }],
        remappings=[
            ('fix', '/gps/fix'),           
            # ('heading', '/gps/heading'),   
            # ('vel', '/gps/vel')            
        ]
    )

    return LaunchDescription([
        arg_port,
        arg_baud,
        arg_frame_id,
        gps_node
    ])