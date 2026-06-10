"""Launch RViz with the MoveIt MotionPlanning plugin pre-configured for om_chain."""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(pkg_share: str, rel: str) -> dict:
    with open(Path(pkg_share) / rel) as f:
        return yaml.safe_load(f) or {}


def _read_text(pkg_share: str, rel: str) -> str:
    with open(Path(pkg_share) / rel) as f:
        return f.read()


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    moveit_share = get_package_share_directory("om_chain_moveit_config")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("om_chain_bringup"), "urdf", "om_chain.urdf.xacro",
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }
    robot_description_semantic = {
        "robot_description_semantic": _read_text(moveit_share, "srdf/om_chain.srdf"),
    }
    kinematics = {"robot_description_kinematics": _load_yaml(moveit_share, "config/kinematics.yaml")}
    joint_limits = {"robot_description_planning": _load_yaml(moveit_share, "config/joint_limits.yaml")}

    rviz_config = str(Path(moveit_share) / "launch" / "moveit.rviz")
    rviz_args = ["-d", rviz_config] if Path(rviz_config).exists() else []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=rviz_args,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])
