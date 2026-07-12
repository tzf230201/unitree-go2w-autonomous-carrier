from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py"
        ])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )
    moveit = TimerAction(period=3.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"),
            "launch", "om6dof_moveit.launch.py",
        ])),
        launch_arguments={
            "start_rviz": LaunchConfiguration("start_rviz"),
            "use_sim": LaunchConfiguration("use_sim"),
        }.items(),
    )])

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        hardware,
        moveit,
    ])
