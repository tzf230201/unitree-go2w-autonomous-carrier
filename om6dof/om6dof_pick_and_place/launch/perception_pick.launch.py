"""MoveIt pickup backend driven by om6dof_perception target points.

The perception service must already be running. This launch never moves the
arm until ``/run_perception_pick`` is explicitly called.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "tag_pick.yaml",
        ]),
    )
    moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")
    topic_arg = DeclareLaunchArgument(
        "perception_topic",
        default_value="/om6dof_perception/target_point",
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"),
            "launch", "om6dof_moveit.launch.py",
        ])),
        launch_arguments={"start_rviz": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )
    picker = Node(
        package="om6dof_pick_and_place",
        executable="direct_pick_node",
        name="direct_pick",
        output="screen",
        parameters=[LaunchConfiguration("config_file"), {
            "object_source": "perception",
            "perception_topic": LaunchConfiguration("perception_topic"),
        }],
        emulate_tty=True,
    )
    return LaunchDescription([
        config_arg, moveit_arg, topic_arg, moveit, picker,
    ])
