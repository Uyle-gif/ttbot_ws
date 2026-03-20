import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    description_dir = get_package_share_directory('ttbot_description')
    localization_dir = get_package_share_directory('ttbot_localization')
    realsense_pkg = get_package_share_directory('realsense2_camera')
    velodyne_pkg = get_package_share_directory('velodyne')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')

    urdf_file = os.path.join(description_dir, 'urdf', 'ttbot.urdf.xacro')    
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_dir, 'launch', 'global_localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

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

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,  
        joint_state_publisher_node,
        localization_launch,
        velodyne_launch,
        realsense_launch,
        xsens_launch
    ])