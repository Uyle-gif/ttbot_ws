import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')


    run_qgc = LaunchConfiguration('run_qgc')
    arg_run_qgc = DeclareLaunchArgument(
        'run_qgc', 
        default_value='true', 
        description='Enable QGroundControl Bridge'
    )


    qgc_bridge_node = TimerAction(
        period=15.0, 
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
    

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument(
        'run_joy', 
        default_value='false',
        description='Set to true to run joystick teleop'
    )

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument(
        'controller_type', 
        default_value='mpc',
        description='Choose controller: "stanley" or "mpc"'
    )

    run_path = LaunchConfiguration('run_path')
    arg_run_path = DeclareLaunchArgument(
        'run_path',
        default_value='false',
        description='Set to true to publish path from CSV'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    run_rviz = LaunchConfiguration('run_rviz')
    arg_rviz = DeclareLaunchArgument('run_rviz', default_value='false')

    path_file = LaunchConfiguration('path_file')
    arg_path = DeclareLaunchArgument('path_file', default_value='path_l.csv')


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    low_level_control_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'controller.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    localization_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_localization, 'launch', 'global_localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'joystick_teleop.launch.py')),
        condition=IfCondition(run_joy),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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

    
    mpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mpc'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 'desired_speed': '1.5'}.items()
            )
        ]
    )

    stanley_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'stanley.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 'desired_speed': '1.5'}.items()
            )
        ]
    )

    high_level_control_delayed = TimerAction(
        period=12.0,
        actions=[stanley_group, mpc_group]
    )

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        arg_sim_time,
        arg_run_qgc,
        arg_controller,
        arg_rviz,
        arg_path,
        arg_run_path,
        arg_run_joy,

        gazebo_launch,
        low_level_control_launch,
        localization_launch,
        path_pub_launch,
        joy_launch,
        high_level_control_delayed,
        qgc_bridge_node,
        rviz_node
    ])