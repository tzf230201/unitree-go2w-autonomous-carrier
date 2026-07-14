"""AprilTag direct pick using planned MoveGroup targets.

Starts MoveGroup, the AprilTag detector, the direct pick sequencer, and the live
calibration web GUI. Arm targets execute through ``arm_controller``; F3 remote
mode must be OFF.

The permanent hardware owner must already be running:

    sudo systemctl restart om6dof-hardware.service

Then, one command:

    ros2 launch om6dof_pick_and_place direct_pick.launch.py

Open http://<robot-ip>:8081 — calibrate the world object with the steppers
(EE pose comes from TF/self-FK), 💾 SIMPAN, then
➡ APPROACH (direct) / ▶ RUN PICK (direct).

Set ``start_moveit:=false`` if another launch already provides MoveGroup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "tag_pick.yaml",
        ]),
        description="Shared params for detector + direct_pick + calib_gui.",
    )
    show_cv_arg = DeclareLaunchArgument(
        "show_cv_window", default_value="false",
        description="Local OpenCV debug window (the GUI streams it anyway).",
    )
    moveit_arg = DeclareLaunchArgument(
        "start_moveit", default_value="true",
        description="Launch MoveGroup for planned arm execution.",
    )
    config = LaunchConfiguration("config_file")

    detector = Node(
        package="om6dof_pick_and_place",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        parameters=[config, {
            "show_window": ParameterValue(
                LaunchConfiguration("show_cv_window"), value_type=bool),
        }],
        emulate_tty=True,
    )
    picker = Node(
        package="om6dof_pick_and_place",
        executable="direct_pick_node",
        name="direct_pick",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    gui = Node(
        package="om6dof_pick_and_place",
        executable="calib_gui",
        name="calib_gui",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"),
            "launch", "om6dof_moveit.launch.py",
        ])),
        launch_arguments={"start_rviz": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )

    return LaunchDescription([
        config_arg, show_cv_arg, moveit_arg,
        moveit, detector, picker, gui,
    ])
