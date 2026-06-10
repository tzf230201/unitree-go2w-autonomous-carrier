"""One-shot demo launch:

  1. om_chain_bringup hardware.launch.py   (ros2_control + Dynamixels)
  2. om_chain_moveit_config move_group     (planning)
  3. om_chain_moveit_config moveit_rviz    (visualization + MotionPlanning plugin)

Only one process can hold /dev/ttyUSB0. Stop go2w-arm-launcher.service first:

    sudo systemctl stop go2w-arm-launcher.service
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om_chain_bringup"), "launch", "hardware.launch.py",
        ])),
    )

    move_group = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("om_chain_moveit_config"),
                "launch", "move_group.launch.py",
            ])),
        )],
    )

    rviz = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("om_chain_moveit_config"),
                "launch", "moveit_rviz.launch.py",
            ])),
        )],
    )

    return LaunchDescription([hardware, move_group, rviz])
