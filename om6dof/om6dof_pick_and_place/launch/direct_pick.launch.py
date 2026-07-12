"""All-in-one launch for the DIRECT (no-MoveIt) AprilTag pick-and-place.

Starts the AprilTag detector, the direct pick sequencer, and the live
calibration web GUI — all reading the shared config/tag_pick.yaml. The arm
itself is driven by the om6dof_teleop teleop node acting as the arm
server (it does the PyKDL IK + smooth Time-based-Profile streaming).

The teleop arm server normally runs as the systemd service
`go2w-arm-launcher`. It MUST have been (re)started after the Cartesian-goal
servo was added:

    sudo systemctl restart go2w-arm-launcher.service

Then, one command:

    ros2 launch om6dof_pick_and_place direct_pick.launch.py

Open http://<robot-ip>:8081 — calibrate the world object with the steppers
(EE pose comes from self-FK, no MoveIt needed), 💾 SIMPAN, then
➡ APPROACH (direct) / ▶ RUN PICK (direct).

Set start_arm_server:=true to also launch the teleop arm server here
(only when the systemd service is stopped — otherwise the serial port
conflicts).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "tag_pick.yaml",
        ]),
        description="Shared params for detector + direct_pick + calib_gui.",
    )
    show_cv_arg = DeclareLaunchArgument(
        "show_cv_window", default_value="false",
        description="Local OpenCV debug window (the GUI streams it anyway).",
    )
    arm_server_arg = DeclareLaunchArgument(
        "start_arm_server", default_value="false",
        description="Also launch the teleop arm server here. Only when the "
                    "go2w-arm-launcher systemd service is stopped.",
    )
    port_arg = DeclareLaunchArgument(
        "port_name", default_value="/dev/serial/by-id/"
        "usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        description="U2D2 serial device (only used if start_arm_server).",
    )

    config = LaunchConfiguration("config_file")

    detector = Node(
        package="om6dof_pick_and_place",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        parameters=[config, {
            "show_window": ParameterValue(
                LaunchConfiguration("show_cv_window"), value_type=bool),
        }],
        emulate_tty=True,
    )
    picker = Node(
        package="om6dof_pick_and_place",
        executable="direct_pick_node",
        name="direct_pick",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    gui = Node(
        package="om6dof_pick_and_place",
        executable="calib_gui",
        name="calib_gui",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    arm_server = Node(
        package="om6dof_teleop",
        executable="teleop_node",
        name="om6dof_teleop",
        output="screen",
        parameters=[{
            "port_name": LaunchConfiguration("port_name"),
        }],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_arm_server")),
    )

    return LaunchDescription([
        config_arg, show_cv_arg, arm_server_arg, port_arg,
        arm_server, detector, picker, gui,
    ])
