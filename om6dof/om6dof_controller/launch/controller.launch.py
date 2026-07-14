"""Start the high-level OM6DOF command converter for existing hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller = Node(
        package="om6dof_controller",
        executable="controller_node",
        name="om6dof_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("om6dof_controller"),
                "config",
                "controller.yaml",
            ]),
            {
                "remote_enabled_on_start": ParameterValue(
                    LaunchConfiguration("remote_enabled_on_start"),
                    value_type=bool,
                )
            },
        ],
    )

    # This launch is included by the systemd-owned full stack. If the command
    # converter exits, keeping ros2_control alive would leave a deceptively
    # healthy service that no longer accepts /om6dof commands. Shut down the
    # containing launch so Restart=always rebuilds every layer together.
    stop_launch_if_controller_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(
                reason="om6dof_controller exited",
            ))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "remote_enabled_on_start",
            default_value="false",
            description="Take remote ownership and enter READY after startup.",
        ),
        stop_launch_if_controller_exits,
        controller,
    ])
