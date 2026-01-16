import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time"
    )

    pkg_share = get_package_share_directory('ttbot_controller')
    default_csv_path = os.path.join(pkg_share, 'path', 'path_l.csv')

    path_file_arg = DeclareLaunchArgument(
        "path_file", default_value=default_csv_path,
        description="Path to CSV file"
    )

    # ==== Stanley Params Tuning ====
    # desired_speed: Tốc độ tối đa khi đi thẳng
    desired_speed_arg = DeclareLaunchArgument("desired_speed", default_value="1.5") 
    
    # min_speed: Tốc độ tối thiểu khi vào cua gắt (quan trọng để không bị vọt)
    min_speed_arg = DeclareLaunchArgument("min_speed", default_value="0.3")

    # slow_down_factor: Càng lớn thì giảm tốc càng gắt khi đánh lái
    slow_down_factor_arg = DeclareLaunchArgument("slow_down_factor", default_value="4.0")

    wheel_base_arg    = DeclareLaunchArgument("wheel_base", default_value="0.65")
    max_steer_deg_arg = DeclareLaunchArgument("max_steer_deg", default_value="30.0")
    
    # K_gain tăng lên 2.5 để phản ứng mạnh hơn với sai số
    k_gain_arg = DeclareLaunchArgument("k_gain", default_value="2.5")
    k_soft_arg = DeclareLaunchArgument("k_soft", default_value="1.0")

    stanley_node = Node(
        package="ttbot_controller",
        executable="stanley_controller",
        name="stanley_controller",
        output="screen",
        parameters=[{
            "use_sim_time":     LaunchConfiguration("use_sim_time"),
            "path_file":        LaunchConfiguration("path_file"),
            "desired_speed":    LaunchConfiguration("desired_speed"),
            "min_speed":        LaunchConfiguration("min_speed"),
            "slow_down_factor": LaunchConfiguration("slow_down_factor"),
            "wheel_base":       LaunchConfiguration("wheel_base"),
            "max_steer_deg":    LaunchConfiguration("max_steer_deg"),
            "k_gain":           LaunchConfiguration("k_gain"),
            "k_soft":           LaunchConfiguration("k_soft"),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        path_file_arg,
        desired_speed_arg,
        min_speed_arg,
        slow_down_factor_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        k_gain_arg,
        k_soft_arg,
        stanley_node
    ])