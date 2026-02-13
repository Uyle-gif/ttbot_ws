import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ttbot_mapping = get_package_share_directory('ttbot_mapping')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ttbot_mapping, 'launch', 'pointcloud_to_laserscan.launch.py')
        )
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': '/home/uylegia/ttbot_ws/src/ttbot_mapping/config/mapper_params.yaml',
            'use_sim_time': 'True'
        }.items()
    )

    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam_toolbox_launch]
    )

    return LaunchDescription([
        pointcloud_to_laserscan_launch,
        delayed_slam
    ])