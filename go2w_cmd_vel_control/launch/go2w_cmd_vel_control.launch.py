from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    request_topic = LaunchConfiguration("request_topic")
    use_teleop = LaunchConfiguration("use_teleop")

    control_node = Node(
        package="go2w_cmd_vel_control",
        executable="go2w_cmd_vel_control_node",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": cmd_vel_topic,
                "request_topic": request_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Twist topic consumed by go2w_cmd_vel_control",
            ),
            DeclareLaunchArgument(
                "request_topic",
                default_value="/api/sport/request",
                description="Unitree sport API request topic",
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="true",
                description="Whether to launch teleop_twist_keyboard in xterm",
            ),
            control_node,
            ExecuteProcess(
                cmd=[
                    "xterm",
                    "-title",
                    "go2w teleop",
                    "-e",
                    "ros2",
                    "run",
                    "teleop_twist_keyboard",
                    "teleop_twist_keyboard",
                    "--ros-args",
                    "-r",
                    ["cmd_vel:=", cmd_vel_topic],
                ],
                output="screen",
                condition=IfCondition(use_teleop),
            ),
        ]
    )
