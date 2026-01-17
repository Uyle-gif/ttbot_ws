import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    gps_port_arg = DeclareLaunchArgument(
        'port', 
        default_value='/dev/ttbot_gps',
        description='Serial port for GPS'
    )
    
    gps_baud_arg = DeclareLaunchArgument(
        'baud', 
        default_value='19200', 
        description='Baudrate for GPS'
    )
    
    gps_frame_arg = DeclareLaunchArgument(
        'frame_id', 
        default_value='gps_link', 
        description='TF Frame ID for GPS'
    )
    
    gps_driver_node = Node(
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
        gps_port_arg,
        gps_baud_arg,
        gps_frame_arg,
        gps_driver_node
    ])