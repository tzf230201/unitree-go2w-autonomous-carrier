from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="go2w_remote_monitor",
                executable="remote_monitor",
                name="remote_monitor",
                output="screen",
                parameters=[
                    {
                        "lowstate_topic": "/lowstate",
                        "log_stick_on_every_message": False,
                    }
                ],
            )
        ]
    )
