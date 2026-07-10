"""Launch the QR/AprilTag follower.

Starts the real MoveIt stack, the AprilTag detector, and a follower node that
keeps the end effector 10 cm in front of the detected tag.

Typical usage:

  ros2 launch om_chain_pick_place qr_follower.launch.py auto_start:=false
  ros2 service call /qr_follow/start std_srvs/srv/Trigger
  ros2 service call /qr_follow/stop std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_moveit_arg = DeclareLaunchArgument(
        "start_moveit",
        default_value="true",
        description="Start hardware + move_group via open_manipulator_6dof_moveit/real.launch.py.",
    )
    port_arg = DeclareLaunchArgument(
        "port_name",
        default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        description="U2D2 serial device for the Dynamixel chain.",
    )
    baud_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="1000000",
        description="Dynamixel baud rate.",
    )
    fake_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use fake ros2_control hardware instead of real Dynamixels.",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om_chain_pick_place"),
            "config",
            "qr_follower.yaml",
        ]),
        description="Parameters for apriltag_detector + qr_follower.",
    )
    detector_arg = DeclareLaunchArgument(
        "start_detector",
        default_value="true",
        description="Start the RealSense AprilTag detector.",
    )
    follower_arg = DeclareLaunchArgument(
        "start_follower",
        default_value="true",
        description="Start the QR follower node.",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Automatically start following after launch.",
    )
    show_cv_window_arg = DeclareLaunchArgument(
        "show_cv_window",
        default_value="false",
        description="Show the detector OpenCV debug window.",
    )
    detector_log_period_arg = DeclareLaunchArgument(
        "detector_terminal_log_period",
        default_value="0.0",
        description="Seconds between detector terminal logs. 0 keeps the detector quiet.",
    )

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("open_manipulator_6dof_moveit"),
            "launch",
            "real.launch.py",
        ])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "baudrate": LaunchConfiguration("baudrate"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )

    detector = Node(
        package="om_chain_pick_place",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "show_window": ParameterValue(
                    LaunchConfiguration("show_cv_window"), value_type=bool,
                ),
                "terminal_log_period": ParameterValue(
                    LaunchConfiguration("detector_terminal_log_period"),
                    value_type=float,
                ),
            },
        ],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_detector")),
    )

    follower = Node(
        package="om_chain_pick_place",
        executable="qr_follower_node",
        name="qr_follower",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "auto_start": ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool,
                ),
            },
        ],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_follower")),
    )

    return LaunchDescription([
        start_moveit_arg,
        port_arg,
        baud_arg,
        fake_arg,
        config_arg,
        detector_arg,
        follower_arg,
        auto_start_arg,
        show_cv_window_arg,
        detector_log_period_arg,
        moveit_stack,
        detector,
        follower,
    ])
