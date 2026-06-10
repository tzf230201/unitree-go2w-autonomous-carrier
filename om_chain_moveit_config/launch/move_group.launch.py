"""Launch move_group with the om_chain MoveIt config.

Run om_chain_bringup's hardware.launch.py FIRST so the controllers + joint
state broadcaster are alive, then launch this. Or use demo.launch.py which
does it for you.
"""
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

    # robot_description from om_chain_bringup's xacro
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("om_chain_bringup"),
            "urdf", "om_chain.urdf.xacro",
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    # robot_description_semantic from our SRDF
    robot_description_semantic = {
        "robot_description_semantic": _read_text(moveit_share, "srdf/om_chain.srdf"),
    }

    # kinematics, joint limits, controllers, planning
    kinematics_yaml = _load_yaml(moveit_share, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = _load_yaml(moveit_share, "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    controllers_yaml = _load_yaml(moveit_share, "config/moveit_controllers.yaml")

    ompl_yaml = _load_yaml(moveit_share, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_yaml,
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            controllers_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor,
        ],
    )

    return LaunchDescription([move_group_node])
