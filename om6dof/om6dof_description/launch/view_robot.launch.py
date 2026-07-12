from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("om6dof_description"), "urdf", "om6dof.urdf.xacro"
            ]),
        ]),
        value_type=str,
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])
