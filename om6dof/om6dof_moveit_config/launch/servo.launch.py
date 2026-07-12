from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = Path(get_package_share_directory("om6dof_moveit_config"))
    with (share / "config" / "moveit_servo.yaml").open() as stream:
        servo_parameters = yaml.safe_load(stream) or {}

    moveit_config = (
        MoveItConfigsBuilder("om6dof", package_name="om6dof_moveit_config")
        .robot_description(
            file_path=str(
                Path(get_package_share_directory("om6dof_description"))
                / "urdf" / "om6dof.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/om6dof.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="screen",
            parameters=[
                {"moveit_servo": servo_parameters},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    ])
