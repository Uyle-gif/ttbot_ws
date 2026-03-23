import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = get_package_share_directory('ttbot_navigation')
    pkg_mapping = get_package_share_directory('ttbot_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(
        pkg_mapping, 'maps', 'bk_map.yaml'))

    lifecycle_nodes = [
        'map_server', 
        'planner_server', 
        'smoother_server', 
        'bt_navigator'
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[os.path.join(pkg_navigation, "config", "planner_server.yaml"),
                    {"use_sim_time": use_sim_time}],
        remappings=[('/plan', '/mpc_path')] 
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        parameters=[os.path.join(pkg_navigation, "config", "smoother_server.yaml"),
                    {"use_sim_time": use_sim_time}]
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[os.path.join(pkg_navigation, "config", "bt_navigator.yaml"),
                    {"use_sim_time": use_sim_time}],
        remappings=[('/plan', '/mpc_path')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'autostart': True, 
            'node_names': lifecycle_nodes,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_yaml_path),
        
        map_server_node,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        lifecycle_manager
    ])