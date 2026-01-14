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
    # Giữ nguyên speed nếu muốn test performance, hoặc giảm về 1.0 nếu thấy lắc
    desired_speed_arg = DeclareLaunchArgument("desired_speed", default_value="1.5") 
    
    wheel_base_arg    = DeclareLaunchArgument("wheel_base", default_value="0.65")
    max_steer_deg_arg = DeclareLaunchArgument("max_steer_deg", default_value="30.0")
    
    k_gain_arg = DeclareLaunchArgument("k_gain", default_value="0.65", description="Cross track error gain")
    
    # Tăng Soft lên một chút để mượt hơn
    k_soft_arg = DeclareLaunchArgument("k_soft", default_value="1.0", description="Softening gain")

    stanley_node = Node(
        package="ttbot_controller",
        executable="stanley_controller",
        name="stanley_controller",
        output="screen",
        parameters=[{
            "use_sim_time":   LaunchConfiguration("use_sim_time"),
            "path_file":      LaunchConfiguration("path_file"),
            "desired_speed":  LaunchConfiguration("desired_speed"),
            "wheel_base":     LaunchConfiguration("wheel_base"),
            "max_steer_deg":  LaunchConfiguration("max_steer_deg"),
            "k_gain":         LaunchConfiguration("k_gain"),
            "k_soft":         LaunchConfiguration("k_soft"),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        path_file_arg,
        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        k_gain_arg,
        k_soft_arg,
        stanley_node
    ])