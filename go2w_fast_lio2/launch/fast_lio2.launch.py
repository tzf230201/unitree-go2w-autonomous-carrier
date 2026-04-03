import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────────
    pkg_share = get_package_share_directory("go2w_fast_lio2")
    desc_share = get_package_share_directory("go2w_description_modified")

    hesai_config_path = os.path.join(pkg_share, "config", "hesai_config.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "fast_lio2.rviz")

    # ── Robot description (URDF from xacro) ────────────────────────────
    xacro_path = os.path.join(desc_share, "xacro", "go2w.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    # ── Launch arguments ───────────────────────────────────────────────
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2",
    )
    declare_rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_path,
        description="Path to RViz config file",
    )

    # ── Nodes ──────────────────────────────────────────────────────────

    # Hesai LiDAR driver – pass config path via ROS parameter
    hesai_driver_node = Node(
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        output="log",
        parameters=[{"config_path": hesai_config_path}],
    )

    # Robot description publisher (custom node from go2w_description_modified)
    robot_description_publisher_node = Node(
        package="go2w_description_modified",
        executable="robot_description_publisher.py",
        name="robot_description_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Robot state publisher (transforms from joint states + URDF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Joint states and IMU publisher for Go2W
    joints_imu_node = Node(
        package="go2w_joints_state_and_imu_publisher",
        executable="go2w_joints_state_and_imu_publisher_node",
        name="go2w_joints_state_and_imu_publisher",
        output="screen",
    )

    # RViz2 (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_rviz_cfg_arg,
        hesai_driver_node,
        robot_description_publisher_node,
        robot_state_publisher_node,
        joints_imu_node,
        rviz_node,
    ])
