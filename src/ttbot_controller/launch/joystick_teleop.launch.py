import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True",
        description="Use simulated time"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            # os.path.join(get_package_share_directory("ttbot_controller"), "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            
            "publish_stamped_twist": True, 

            "require_enable_button": True,
            "enable_button": 5, 
            "axis_linear.x": 1,
            "scale_linear.x": 1.0,
            "axis_angular.yaw": 3,
            "scale_angular.yaw": 1.33,
        }],
        remappings=[
            ('/cmd_vel', '/joy_cmd_vel') 
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        teleop_node
    ])