import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            name='pointcloud_to_grid_node',
            output='screen',
            parameters=[{
                'input_topic': '/velodyne_points/points', 
                'target_frame': 'odom', 
                
                'grid_ros_topic_name': '/grid',     
                'resolution': 0.1,                 
                
                'min_z': -0.1,                    
                'max_z': 0.5,                      
                
                'use_intensity': True,              
                'filter_intensity': False,        
                
                'mapi_gridmap_topic_name': 'intensity_gridmap',
                'maph_gridmap_topic_name': 'height_gridmap',
            }]
        )
    ])