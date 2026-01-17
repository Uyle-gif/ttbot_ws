import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_sensors = get_package_share_directory('ttbot_sensors')

    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttbot_imu',
        description='Port for IMU'
    )

    gps_port_arg = DeclareLaunchArgument(
        'gps_port', 
        default_value='/dev/ttbot_gps',
        description='Port for GPS'
    )
    
    gps_baud_arg = DeclareLaunchArgument(
        'gps_baud', 
        default_value='19200', 
        description='Baudrate for GPS'
    )


    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'imu.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('imu_port') 
        }.items()
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'gps.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('gps_port'),
            'baud': LaunchConfiguration('gps_baud'),
            'frame_id': 'gps_link'
        }.items()
    )

    return LaunchDescription([
        imu_port_arg,
        gps_port_arg,
        gps_baud_arg,
        
        imu_launch,
        gps_launch
    ])