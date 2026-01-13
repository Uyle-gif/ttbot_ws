import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap # <--- Thêm SetRemap
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. PATH DEFINITIONS
    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
    pkg_imu_driver = get_package_share_directory('adis16488_driver')
    pkg_gps_driver = get_package_share_directory('ublox_driver') 
    
    # Path tới file config mux (Bạn nhớ tạo file này như hướng dẫn trước)
    mux_config = os.path.join(pkg_controller, 'config', 'mux_config.yaml')

    # 2. ARGUMENTS
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false') 

    run_qgc = LaunchConfiguration('run_qgc')
    arg_run_qgc = DeclareLaunchArgument('run_qgc', default_value='true')

    imu_port = LaunchConfiguration('imu_port')
    arg_imu = DeclareLaunchArgument('imu_port', default_value='/dev/ttbot_imu')
    
    stm32_port = LaunchConfiguration('stm32_port')
    arg_stm32 = DeclareLaunchArgument('stm32_port', default_value='/dev/ttbot_stm32')

    gps_port = LaunchConfiguration('gps_port')
    arg_gps = DeclareLaunchArgument('gps_port', default_value='/dev/ttbot_gps')

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument('run_joy', default_value='false')

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument('controller_type', default_value='mpc')

    run_path = LaunchConfiguration('run_path')
    arg_run_path = DeclareLaunchArgument('run_path', default_value='false')
    
    path_file = LaunchConfiguration('path_file')
    arg_path = DeclareLaunchArgument('path_file', default_value='path_l.csv')
   
    # 3. HARDWARE & DRIVERS LAYER

    xacro_file = os.path.join(pkg_description, 'urdf', 'ttbot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' is_ignition:=false']), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_imu_driver, 'launch', 'imu_system.launch.py')),
        launch_arguments={'imu_port': imu_port}.items()
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gps_driver, 'launch', 'gps.launch.py')),
        launch_arguments={
            'port': gps_port,   
            'baud': '19200',    
            'frame_id': 'gps_link'
        }.items()
    )

    # --- NODE ĐIỀU KHIỂN CẤP THẤP ---
    # Node này sẽ nhận lệnh cuối cùng từ Mux
    ackermann_node = Node(
        package='ttbot_controller',
        executable='ackermann_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Đảm bảo node này nghe đúng topic đầu ra của Mux
            ('/cmd_vel', '/ackermann_controller/cmd_vel') 
        ]
    )

    # --- QGC BRIDGE ---
    qgc_bridge_node = TimerAction(
        period=16.0, 
        actions=[
            Node(
                package='qgc_bridge_cpp',      
                executable='qgc_bridge_node',  
                name='qgc_bridge_node',
                output='screen',
                condition=IfCondition(run_qgc),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'heading_offset_deg': -90.0     
                }]
            )
        ]
    )

    # --- JOYSTICK (ĐÃ SỬA ĐỂ REMAP) ---
    # Dùng GroupAction để ép toàn bộ topic trong joy_launch đổi tên
    joy_launch_group = GroupAction(
        condition=IfCondition(run_joy),
        actions=[
            SetRemap(src='/cmd_vel', dst='/joy_cmd_vel'), # <--- MAGIC HERE
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'joystick_teleop.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # --- TWIST MUX (NGƯỜI PHÁN XỬ) ---
    stamped_mux_node = Node(
        package='ttbot_controller',
        executable='stamped_twist_mux',  # Tên executable khai báo trong CMakeLists
        name='stamped_twist_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'joy_timeout': 0.5  # Thời gian chờ (giây) trước khi trả quyền cho MPC
        }],
        remappings=[
            # Input (Khớp với topic trong code C++)
            ('joy_cmd_vel', '/joy_cmd_vel'),
            ('mpc_cmd_vel', '/mpc_cmd_vel'),
            
            # Output (Bắn vào Controller)
            ('cmd_cmd_out', '/ackermann_controller/cmd_vel')
        ]
    )

    # 4. ALGORITHMS LAYER
    
    localization_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_localization, 'launch', 'global_localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    path_pub_launch = GroupAction(
        condition=IfCondition(run_path),
        actions=[
            TimerAction(
                period=20.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'path_publisher.launch.py')),
                        launch_arguments={'use_sim_time': use_sim_time, 'path_file': path_file}.items()
                    )
                ]
            )
        ]
    )

    # --- MPC & STANLEY (ĐÃ SỬA ĐỂ REMAP) ---
    # Ép output của thuật toán sang /mpc_cmd_vel để đi vào Mux
    
    mpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mpc'"])),
        actions=[
            SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'), # <--- MAGIC HERE
            SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'), # Remap luôn cho chắc
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    stanley_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        actions=[
            SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'), # <--- MAGIC HERE
            SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'stanley.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    high_level_control = TimerAction(
        period=8.0,
        actions=[stanley_group, mpc_group]
    )

    return LaunchDescription([

        arg_run_qgc,
        arg_sim_time, arg_controller, arg_run_path, arg_path, 
        arg_imu, arg_stm32, arg_gps, arg_run_joy, 

        # Hardware & Drivers
        robot_state_publisher,
        imu_launch,
        gps_launch,
        ackermann_node,   # Nhận lệnh cuối cùng
        qgc_bridge_node,
        
        # Inputs control
        joy_launch_group, # Output: /joy_cmd_vel
        stamped_mux_node,   # Input: /joy_cmd_vel & /mpc_cmd_vel -> Output: /ackermann_controller/cmd_vel

        # Algorithms layer
        localization_launch,
        path_pub_launch,
        high_level_control, # Output: /mpc_cmd_vel
    ])