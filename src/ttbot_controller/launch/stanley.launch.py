from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    path_file_arg = DeclareLaunchArgument(
        "path_file",
        default_value="path_right.csv",  # mặc định path u
        description="CSV path file to load"
    )

    stanley_node = Node(
        package="ttbot_controller",
        executable="stanley_controller",
        name="stanley_controller",
        output="screen",
        parameters=[{
            "path_file": LaunchConfiguration("path_file")
        }]
    )

    return LaunchDescription([
        path_file_arg,
        stanley_node
    ])
