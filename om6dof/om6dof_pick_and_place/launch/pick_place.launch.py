"""Launch the pick-and-place state machine.

The MoveIt stack must already be running. Either:

  Terminal 1:   ros2 launch om6dof_bringup real.launch.py
  Terminal 2:   ros2 launch om6dof_pick_and_place pick_place.launch.py

…or use the all-in-one launch file inside om6dof_pick_and_place if you prefer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoints_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "waypoints.yaml",
        ]),
        description="Path to the waypoints YAML.",
    )
    auto_run_arg = DeclareLaunchArgument(
        "auto_run", default_value="true",
        description="If true, run the sequence automatically once at startup.",
    )

    node = Node(
        package="om6dof_pick_and_place",
        executable="pick_place_node",
        name="om6dof_pick_and_place",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "auto_run": LaunchConfiguration("auto_run"),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([waypoints_arg, auto_run_arg, node])
