"""RViz simulation of the remote teleop — NO hardware.

Runs go2w_remote_arm's teleop_node in `sim_mode`, so it subscribes to the
real `/wirelesscontroller` and runs the exact same JOINT/IK control logic,
but instead of driving Dynamixels it just publishes `/joint_states`. A
robot_state_publisher (om_chain URDF) + RViz then visualize the motion.

Use this to verify the control feel against the URDF before touching the
real arm:

    ros2 launch go2w_remote_arm sim.launch.py

Then on the Go2W remote: tap Select to enter IK mode, jog with the d-pad /
A-Y-X-B / sticks / R1-R2 / L1-L2 and watch the arm in RViz.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_velocity_arg = DeclareLaunchArgument(
        "joint_velocity", default_value="0.5",
        description="JOINT-mode jog speed (rad/s while a button is held).",
    )

    pkg_bringup = FindPackageShare("om_chain_bringup")

    # robot_description (use_fake_hardware so no ros2_control plugin is needed)
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([pkg_bringup, "urdf", "om_chain.urdf.xacro"]),
                " use_fake_hardware:=true",
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    teleop = Node(
        package="go2w_remote_arm",
        executable="teleop_node",
        name="go2w_remote_arm",
        output="screen",
        parameters=[{
            "sim_mode": True,
            "joint_names": [
                "joint1", "joint2", "joint3", "joint4",
                "joint5", "joint6", "gripper",
            ],
            "motor_ids": [31, 32, 33, 24, 35, 26, 37],
            "joint_velocity": ParameterValue(
                LaunchConfiguration("joint_velocity"), value_type=float),
            "joint_signs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "joint_axes": [-1, -1, -1, -1, 3, -1],
            "button_pairs": [
                15, 13,   # j1: Left / Right
                14, 12,   # j2: Down / Up
                 8, 11,   # j3: A / Y
                10,  9,   # j4: X / B
                -1, -1,   # j5: (analog ry)
                 0,  4,   # j6: R1 / R2
            ],
            "gripper_btn_open": 1,
            "gripper_btn_close": 5,
            "gripper_open_target": 0.019,   # prismatic metres (URDF range)
            "gripper_close_target": -0.010,
            "ik_enabled": True,
            "ik_base_link": "world",
            "ik_tip_link": "end_effector_link",
            "ik_urdf_pkg": "om_chain_bringup",
        }],
        emulate_tty=True,
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2w_remote_arm"), "launch", "sim.rviz",
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
    )

    return LaunchDescription([
        joint_velocity_arg,
        robot_state_publisher,
        teleop,
        rviz,
    ])
