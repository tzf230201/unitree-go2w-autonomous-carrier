"""Fake-hardware three-mode remote/controller-switch demo with RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_teleop"), "launch", "full_stack.launch.py",
        ])),
        launch_arguments={
            "use_fake_hardware": "true",
            "joint_velocity": LaunchConfiguration("joint_velocity"),
            "remote_enabled_on_start": LaunchConfiguration("remote_enabled_on_start"),
        }.items(),
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("om6dof_description"), "urdf", "om6dof.urdf.xacro",
                ]),
            ]),
            value_type=str,
        )
    }
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("om6dof_teleop"), "launch", "sim.rviz",
        ])],
        parameters=[robot_description],
    )
    return LaunchDescription([
        DeclareLaunchArgument("joint_velocity", default_value="0.5"),
        DeclareLaunchArgument("remote_enabled_on_start", default_value="false"),
        stack,
        rviz,
    ])
