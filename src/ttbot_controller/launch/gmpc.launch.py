import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time (true for Gazebo, false for Real Robot)"
    )
    pkg_share = get_package_share_directory('ttbot_controller')
    config_file = os.path.join(pkg_share, 'config', 'gmpc_params.yaml')
    gmpc_node = Node(
        package="ttbot_controller",
        executable="gmpc_controller",
        name="gmpc_controller",
        output="screen",
        parameters=[
            config_file,  
            {
                "use_sim_time": LaunchConfiguration("use_sim_time") 
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gmpc_node
    ])