from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params",
        default_value=PathJoinSubstitution([
            FindPackageShare("open_manipulator_x_ctrl"),
            "config",
            "params.yaml",
        ]),
        description="Path to YAML params file for the controller node.",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="/dev/ttyUSB0",
        description="Serial device for U2D2.",
    )
    return LaunchDescription([
        params_arg,
        device_arg,
        Node(
            package="open_manipulator_x_ctrl",
            executable="controller_node",
            name="open_manipulator_x_ctrl",
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                {"device": LaunchConfiguration("device")},
            ],
            emulate_tty=True,
        ),
    ])
