from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    desc_share = get_package_share_directory("go2w_description_modified")

    xacro_path = os.path.join(desc_share, "xacro", "go2w.xacro")
    rviz_config_path = os.path.join(desc_share, "launch", "display.rviz")

    robot_description = xacro.process_file(xacro_path).toxml()
    robot_description = robot_description.replace(
        "package://go2w_description/", "package://go2w_description_modified/"
    )

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        Node(
            package="go2w_description_modified",
            executable="robot_description_publisher.py",
            name="robot_description_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="go2w_joints_state_and_imu_publisher",
            executable="go2w_joints_state_and_imu_publisher_node",
            name="go2w_joints_state_and_imu_publisher",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]

    return LaunchDescription(actions)
