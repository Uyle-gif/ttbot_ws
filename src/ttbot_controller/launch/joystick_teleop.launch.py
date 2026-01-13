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

    # 1. JOY NODE (Driver đọc phần cứng)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            # Có thể giữ file config joy_config.yaml nếu nó chỉ chứa cấu hình deadzone
            # Nếu không có file này thì xóa dòng dưới đi cũng được, joy_node tự có default
            # os.path.join(get_package_share_directory("ttbot_controller"), "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # 2. TELEOP NODE (ĐÃ SỬA LẠI CẤU HÌNH CHO ĐÚNG GÓI)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            
            # --- [QUAN TRỌNG] BẬT TRUE ---
            "publish_stamped_twist": True, 
            # -----------------------------

            "require_enable_button": True,
            "enable_button": 5, 
            "axis_linear.x": 1,
            "scale_linear.x": 1.5,
            "axis_angular.yaw": 3,
            "scale_angular.yaw": -1.33,
        }],
        remappings=[
            ('/cmd_vel', '/joy_cmd_vel') # Output topic này giờ là TwistStamped
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        teleop_node
    ])