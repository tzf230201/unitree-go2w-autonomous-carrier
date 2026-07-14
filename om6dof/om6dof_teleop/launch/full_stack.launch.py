"""Canonical runtime: hardware owner -> command converter -> input adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py",
        ])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_controller"), "launch", "controller.launch.py",
        ])),
        launch_arguments={
            "remote_enabled_on_start": LaunchConfiguration(
                "remote_enabled_on_start"
            ),
        }.items(),
    )
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_teleop"), "launch", "teleop.launch.py",
        ])),
        launch_arguments={
            "joint_velocity": LaunchConfiguration("joint_velocity"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("joint_velocity", default_value="0.5"),
        DeclareLaunchArgument("remote_enabled_on_start", default_value="false"),
        hardware,
        controller,
        teleop,
    ])
