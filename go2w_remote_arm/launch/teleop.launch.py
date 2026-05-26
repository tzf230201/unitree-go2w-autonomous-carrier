"""All-in-one launch:
   - open_manipulator_x_bringup hardware.launch.py    (ros2_control + Dynamixel)
   - go2w_remote_arm teleop_node                       (remote -> direct joint trajectory)

Remote joint buttons:
   Left/Right arrows -> joint1, Down/Up arrows -> joint2,
   A/Y -> joint3, X/B -> joint4, L1/L2 -> gripper.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_velocity_arg = DeclareLaunchArgument(
        "joint_velocity",
        default_value="0.5",
        description="Joint velocity in rad/s while a remote button is held.",
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("open_manipulator_x_bringup"), "launch", "hardware.launch.py",
        ])),
    )

    remote_arm = TimerAction(
        period=3.0,
        actions=[Node(
            package="go2w_remote_arm",
            executable="teleop_node",
            name="go2w_remote_arm",
            output="screen",
            parameters=[{
                "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                "joint_velocity": ParameterValue(
                    LaunchConfiguration("joint_velocity"),
                    value_type=float,
                ),
                "button_joint_signs": [1.0, 1.0, 1.0, 1.0],
            }],
            emulate_tty=True,
        )],
    )

    return LaunchDescription([
        joint_velocity_arg,
        hardware,
        remote_arm,
    ])
