import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fast_livo_pkg = get_package_share_directory('fast_livo')
    realsense_pkg = get_package_share_directory('realsense2_camera')
    velodyne_pkg = get_package_share_directory('velodyne')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')

    config_file_dir = os.path.join(fast_livo_pkg, "config")
    ttbot_config_real = os.path.join(config_file_dir, "ttbot_real.yaml") 
    camera_config_real = os.path.join(config_file_dir, "camera_real.yaml") 
    rviz_config_file = os.path.join(fast_livo_pkg, "rviz_cfg", "fast_livo2.rviz")

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')])
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(realsense_pkg, 'launch', 'rs_launch.py')]),
        launch_arguments={'pointcloud.enable': 'false'}.items()
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(xsens_pkg, 'launch', 'xsens_mti_node.launch.py')])
    )

    fastlivo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[ttbot_config_real, camera_config_real, {'use_sim_time': False}],
        output="screen"
    )

    tf_camera_init_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # Nâng camera_init (bản đồ) lên 1.38m so với odom
        # Lưu ý: odom là parent, camera_init là child
        arguments=['0', '0', '1.38', '0', '0', '0', 'odom', 'camera_init']
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        velodyne_launch,
        realsense_launch,
        xsens_launch,
        fastlivo_node,
        rviz_node,
        tf_camera_init_to_odom
    ])