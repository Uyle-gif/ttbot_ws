import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Lấy đường dẫn package
    pkg_share = get_package_share_directory('ttbot_localization')
    
    # 2. Argument: use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 3. Include LOCAL Localization (Odom + IMU)
    # Tận dụng lại file bạn đã có để publish TF: odom -> base_link
    local_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'local_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Config cho Global EKF & Navsat
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    # 5. Node: Navsat Transform (GPS Lat/Lon -> Odom X/Y)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', '/imu/data_filtered'),          # IMU đã qua lọc Madgwick [cite: 17]
            ('gps/fix', '/gps/fix'),                     # Topic GPS từ Gazebo/Sensor thật
            ('odometry/gps', '/odometry/gps'),           # Output: Odom từ GPS
            ('odometry/filtered', '/odometry/global')    # Input feedback: Vị trí global hiện tại
        ]
    )

    # 6. Node: Global EKF (Fuse Odom + IMU + GPS_Odom)
    # Nhiệm vụ: Publish TF map -> odom
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/global')    # Output topic của global filter
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        local_localization_launch,
        navsat_transform_node,
        ekf_global_node
    ])