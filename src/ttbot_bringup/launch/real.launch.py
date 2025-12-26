import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
    pkg_imu_driver = get_package_share_directory('adis16488_driver')
    pkg_gps_driver = get_package_share_directory('ublox_driver') 

    run_qgc = LaunchConfiguration('run_qgc')
    arg_run_qgc = DeclareLaunchArgument(
        'run_qgc', 
        default_value='true', 
        description='Enable QGroundControl Bridge'
    )

    qgc_bridge_node = TimerAction(
        period=16.0, 
        actions=[
            Node(
                package='qgc_bridge',
                executable='bridge_node', 
                name='qgc_bridge_node',
                output='screen',
                condition=IfCondition(run_qgc) 
            )
        ]
    )


    imu_port = LaunchConfiguration('imu_port')
    arg_imu = DeclareLaunchArgument('imu_port', default_value='/dev/ttbot_imu')
    
    stm32_port = LaunchConfiguration('stm32_port')
    arg_stm32 = DeclareLaunchArgument('stm32_port', default_value='/dev/ttbot_stm32')

    gps_port = LaunchConfiguration('gps_port')
    arg_gps = DeclareLaunchArgument('gps_port', default_value='/dev/ttbot_gps')

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument(
        'run_joy', default_value='false',
        description='Set to true to enable joystick teleop'
    )

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument(
        'controller_type', default_value='mpc',
        description='Choose controller: "stanley" or "mpc"'
    )

    run_path = LaunchConfiguration('run_path')
    arg_run_path = DeclareLaunchArgument('run_path', default_value='false')
    
    path_file = LaunchConfiguration('path_file')
    arg_path = DeclareLaunchArgument('path_file', default_value='path_l.csv')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

   
    #  HARDWARE & DRIVERS LAYER

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

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen'
    )



    ackermann_node = Node(
        package='ttbot_controller',
        executable='ackermann_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'joystick_teleop.launch.py')),
        condition=IfCondition(run_joy),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ALGORITHMS LAYER
    
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
                period=6.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'path_publisher.launch.py')),
                        launch_arguments={'use_sim_time': use_sim_time, 'path_file': path_file}.items()
                    )
                ]
            )
        ]
    )

    mpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mpc'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    stanley_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        actions=[
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
      # micro_ros_agent,
        ackermann_node,
        qgc_bridge_node,
        joy_launch,

        # Algorithms layer
        localization_launch,
        path_pub_launch,
        high_level_control,
        micro_ros_agent
    ])