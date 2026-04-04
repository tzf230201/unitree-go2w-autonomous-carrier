from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    desc_share = get_package_share_directory("go2w_description")

    xacro_path = os.path.join(desc_share, "xacro", "go2w.xacro")
    rviz_config_path = os.path.join(desc_share, "launch", "check_joint.rviz")

    robot_description = xacro.process_file(xacro_path).toxml()

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "jsp_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for interactive joints.",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=UnlessCondition(LaunchConfiguration("jsp_gui")),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
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
