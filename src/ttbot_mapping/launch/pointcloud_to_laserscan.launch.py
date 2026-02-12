from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/velodyne_points/points'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link', 
                'transform_tolerance': 0.01,
                
                'min_height': 0.05, 
                'max_height': 0.3,  
                
                'angle_min': -3.1415, 
                'angle_max': 3.1415,
                'angle_increment': 0.0087, 
                'scan_time': 0.1,
                'range_min': 0.5, 
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
        )
    ])