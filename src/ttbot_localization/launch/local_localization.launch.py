from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    common_params = [{"use_sim_time": use_sim_time}]

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("ttbot_localization"), "config", "ekf_local_enc.yaml"),
            common_params[0] 
        ],
    )

    imu_republisher_cpp = Node(
        package="ttbot_localization",
        executable="imu_republisher",
        parameters=common_params
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        robot_localization,
        imu_republisher_cpp,   
    ])