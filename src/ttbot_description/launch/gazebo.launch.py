import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ttbot_description = get_package_share_directory("ttbot_description")
    world_path = os.path.join(ttbot_description, "worlds", "gps_world.sdf")

    # 1. Lấy biến môi trường đúng cách
    ros_distro = os.environ.get("ROS_DISTRO")
    is_ignition = "True" if ros_distro == "humble" else "False"

    # 2. Khai báo Argument
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(ttbot_description, "urdf", "ttbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # 3. Config Resource cho Gazebo
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(ttbot_description).parent.resolve())]
    )
    
    # 4. Xử lý file Xacro thành URDF
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
        ]),
        value_type=str
    )
    
    # 5. Robot State Publisher (QUAN TRỌNG: use_sim_time=True)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True 
        }]
    )

    # 6. Khởi chạy Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            # Chú ý: "-r -v 4 " (có dấu cách ở cuối chuỗi) nối với world_path
            ("gz_args", ["-r -v 4 ", world_path]) 
        ]
    )
    

    # 7. Spawn Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ttbot",
            "-z", "0.1" # Nâng robot lên 1 chút để không bị kẹt dưới sàn khi spawn
        ],
    )

    # 8. Bridge (Cầu nối ROS 2 <-> Gazebo)
    # Lưu ý: Topic joint_state phụ thuộc vào tên World trong file SDF (ở đây giả định là 'empty')
# 8. Bridge (Cầu nối ROS 2 <-> Gazebo)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'use_sim_time': True}],
        arguments=[
            # 1. Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            
            # 2. GPS (Tên ngắn từ lệnh ign topic -l)
            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            
            # 3. IMU (Tên ngắn từ lệnh ign topic -l)
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            
            # 4. Joint States
            # Lưu ý: Joint state thường vẫn bị dài, ta dùng remap để xử lý
            "/world/gps_world/model/ttbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            # Remap GPS (Nguồn /gps/fix -> Đích /gps/fix)
            ('/gps/fix', '/gps/fix'),
            
            # Remap IMU (Nguồn /imu -> Đích /imu/out)
            ('/imu', '/imu/out'),
            
            # Remap Joint State
            ('/world/gps_world/model/ttbot/joint_state', '/joint_states')
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])