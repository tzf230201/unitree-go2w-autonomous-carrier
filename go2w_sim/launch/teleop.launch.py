import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def _setup(context, *args, **kwargs):
        cmd_vel = LaunchConfiguration("cmd_vel").perform(context)
        use_xterm = LaunchConfiguration("use_xterm").perform(context).strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

        if not use_xterm:
            raise RuntimeError(
                "teleop_twist_keyboard needs a real TTY. Use use_xterm:=true, "
                "or run directly: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"
            )

        if shutil.which("xterm") is None:
            raise RuntimeError(
                "xterm not found. Install it (sudo apt install xterm) or run teleop directly in your terminal."
            )

        teleop_cmd = (
            "source /opt/ros/humble/setup.bash; "
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
            f"--ros-args -r __node:=teleop_twist_keyboard -r /cmd_vel:={cmd_vel} -r cmd_vel:={cmd_vel}"
        )

        return [
            ExecuteProcess(
                cmd=[
                    "xterm",
                    "-fa",
                    "Monospace",
                    "-fs",
                    "12",
                    "-e",
                    "bash",
                    "-lc",
                    teleop_cmd,
                ],
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel",
                default_value="/cmd_vel",
                description="ROS 2 cmd_vel topic to publish.",
            ),
            DeclareLaunchArgument(
                "use_xterm",
                default_value="true",
                description="Launch teleop in xterm so it has a real TTY (recommended under ros2 launch).",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
