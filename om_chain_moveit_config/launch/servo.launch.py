"""Launch MoveIt Servo (realtime collision-aware Cartesian jog).

Requires the controllers + move_group context. Easiest is to use the
go2w_remote_arm demo_servo.launch.py which wires up everything. Standalone:

    ros2 launch om_chain_bringup hardware.launch.py use_fake_hardware:=true
    ros2 launch om_chain_moveit_config servo.launch.py

Then publish geometry_msgs/TwistStamped on /servo_node/delta_twist_cmds
(or use the go2w_remote_arm remote_servo_bridge).
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(share, rel):
    with open(Path(share) / rel) as f:
        return yaml.safe_load(f) or {}


def _read(share, rel):
    with open(Path(share) / rel) as f:
        return f.read()


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    moveit_share = get_package_share_directory("om_chain_moveit_config")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("om_chain_bringup"),
                    "urdf", "om_chain.urdf.xacro",
                ]),
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _read(moveit_share, "srdf/om_chain.srdf"),
    }
    kinematics = {"robot_description_kinematics": _load_yaml(moveit_share, "config/kinematics.yaml")}

    servo_yaml = _load_yaml(moveit_share, "config/moveit_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics,
        ],
    )

    return LaunchDescription([servo_node])
