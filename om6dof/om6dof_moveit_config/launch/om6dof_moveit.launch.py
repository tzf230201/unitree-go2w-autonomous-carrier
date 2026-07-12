#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "start_rviz", default_value="true",
            description="Whether to start RViz with the MoveIt plugin",
        ),
        DeclareLaunchArgument(
            "use_sim", default_value="false",
            description="Whether MoveIt and RViz should use simulation time",
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic", default_value="true",
            description="Whether move_group publishes robot_description_semantic",
        ),
    ]

    start_rviz = LaunchConfiguration("start_rviz")
    use_sim = LaunchConfiguration("use_sim")
    publish_semantic = LaunchConfiguration("publish_robot_description_semantic")
    description_share = get_package_share_directory("om6dof_description")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="om6dof", package_name="om6dof_moveit_config"
        )
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "om6dof.urdf.xacro")
        )
        .robot_description_semantic(str(Path("config") / "om6dof.srdf"))
        .joint_limits(str(Path("config") / "joint_limits.yaml"))
        .trajectory_execution(str(Path("config") / "moveit_controllers.yaml"))
        .robot_description_kinematics(str(Path("config") / "kinematics.yaml"))
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim,
                "publish_robot_description_semantic": publish_semantic,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        condition=IfCondition(start_rviz),
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"), "config", "moveit.rviz"
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
