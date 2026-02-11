from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    cmd_vel = LaunchConfiguration("cmd_vel").perform(context)
    use_xterm = _as_bool(LaunchConfiguration("use_xterm").perform(context))

    base_cmd = [
        "ros2",
        "run",
        "teleop_twist_keyboard",
        "teleop_twist_keyboard",
        "--ros-args",
        "-r",
        f"cmd_vel:={cmd_vel}",
    ]

    if use_xterm:
        cmd = [
            "xterm",
            "-title",
            "go2w teleop",
            "-e",
            *base_cmd,
        ]
    else:
        cmd = base_cmd

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel",
                default_value="/cmd_vel",
                description="ROS 2 cmd_vel topic to publish Twist messages to.",
            ),
            DeclareLaunchArgument(
                "use_xterm",
                default_value="true",
                description="Launch teleop in xterm for interactive keyboard input.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
