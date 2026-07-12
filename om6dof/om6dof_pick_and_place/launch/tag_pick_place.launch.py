"""All-in-one AprilTag pick-and-place demo launch.

Starts the real MoveIt stack, AprilTag detector, tag picker, RViz markers, and
an image viewer for /apriltag/debug_image.

Before launching, release devices that may already own the arm/camera:

  sudo systemctl stop go2w-arm-launcher.service
  sudo systemctl stop go2w-web-monitor.service

Then:

  ros2 launch om6dof_pick_and_place tag_pick_place.launch.py
  ros2 service call /tag_world std_srvs/srv/Trigger
  ros2 service call /run_tag_pick std_srvs/srv/Trigger
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_moveit_arg = DeclareLaunchArgument(
        "start_moveit", default_value="true",
        description="Start hardware + move_group + RViz via om6dof_bringup/real.launch.py.",
    )
    port_arg = DeclareLaunchArgument(
        "port_name", default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        description="U2D2 serial device for the Dynamixel chain.",
    )
    baud_arg = DeclareLaunchArgument(
        "baudrate", default_value="1000000",
        description="Dynamixel baud rate.",
    )
    fake_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="false",
        description="Use fake ros2_control hardware instead of real Dynamixels.",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "tag_pick.yaml",
        ]),
        description="Parameters for apriltag_detector + tag_pick_place.",
    )
    waypoints_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config", "waypoints.yaml",
        ]),
        description="Waypoints YAML (start_pose / place_above / place).",
    )
    image_viewer_arg = DeclareLaunchArgument(
        "start_image_viewer", default_value="false",
        description="Open rqt_image_view on /apriltag/debug_image.",
    )
    show_cv_window_arg = DeclareLaunchArgument(
        "show_cv_window", default_value="false",
        description="Show the detector's OpenCV debug window. Usually not "
                    "needed anymore: the calib GUI (port 8081) streams the "
                    "same debug image to the browser.",
    )
    detector_log_period_arg = DeclareLaunchArgument(
        "detector_terminal_log_period",
        default_value="0.0",
        description="Seconds between detector terminal logs. 0 keeps the detector quiet.",
    )
    auto_run_arg = DeclareLaunchArgument(
        "auto_run", default_value="false",
        description="Automatically start the sequence after launch.",
    )
    auto_run_delay_arg = DeclareLaunchArgument(
        "auto_run_delay", default_value="8.0",
        description="Delay before auto_run starts, in seconds.",
    )
    picker_arg = DeclareLaunchArgument(
        "start_picker",
        default_value="true",
        description="Start the pick-place state machine. Set false for read-only manual coordinate debug.",
    )
    coord_debug_arg = DeclareLaunchArgument(
        "start_coord_debug",
        default_value="true",
        description="Start coordinate debugger node and /coord_debug_markers.",
    )
    coord_robot_debug_arg = DeclareLaunchArgument(
        "coord_robot_debug",
        default_value="true",
        description="Read robot /joint_states and TF in coord_debug. Set false for AprilTag-only debug.",
    )
    coord_log_period_arg = DeclareLaunchArgument(
        "coord_log_period",
        default_value="0.5",
        description="Seconds between coord_debug terminal reports. 0 disables periodic logs.",
    )
    coord_live_console_arg = DeclareLaunchArgument(
        "coord_live_console",
        default_value="true",
        description="Draw coord_debug as a fixed live terminal dashboard instead of scrolling logs.",
    )
    calib_gui_arg = DeclareLaunchArgument(
        "start_calib_gui",
        default_value="true",
        description="Start the live camera-calibration web GUI on port 8081.",
    )

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"),
            "launch",
            "real.launch.py",
        ])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "baud_rate": LaunchConfiguration("baudrate"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )

    detector = Node(
        package="om6dof_pick_and_place",
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
                    LaunchConfiguration("detector_terminal_log_period"), value_type=float,
                ),
            },
        ],
        emulate_tty=True,
    )
    picker = Node(
        package="om6dof_pick_and_place",
        executable="tag_pick_place_node",
        name="tag_pick_place",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "auto_run": ParameterValue(
                    LaunchConfiguration("auto_run"), value_type=bool,
                ),
                "auto_run_delay": ParameterValue(
                    LaunchConfiguration("auto_run_delay"), value_type=float,
                ),
            },
        ],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_picker")),
    )
    coord_debug = Node(
        package="om6dof_pick_and_place",
        executable="coordinate_debug_node",
        name="coord_debug",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "enable_robot_debug": ParameterValue(
                    LaunchConfiguration("coord_robot_debug"), value_type=bool,
                ),
                "log_period": ParameterValue(
                    LaunchConfiguration("coord_log_period"), value_type=float,
                ),
                "live_console": ParameterValue(
                    LaunchConfiguration("coord_live_console"), value_type=bool,
                ),
            },
        ],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_coord_debug")),
    )

    calib_gui = Node(
        package="om6dof_pick_and_place",
        executable="calib_gui",
        name="calib_gui",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_calib_gui")),
    )

    image_viewer = TimerAction(
        period=3.0,
        actions=[Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="apriltag_debug_view",
            output="screen",
            arguments=["/apriltag/debug_image"],
        )],
        condition=IfCondition(LaunchConfiguration("start_image_viewer")),
    )

    return LaunchDescription([
        start_moveit_arg,
        port_arg,
        baud_arg,
        fake_arg,
        config_arg,
        waypoints_arg,
        image_viewer_arg,
        show_cv_window_arg,
        detector_log_period_arg,
        auto_run_arg,
        auto_run_delay_arg,
        picker_arg,
        coord_debug_arg,
        coord_robot_debug_arg,
        coord_log_period_arg,
        coord_live_console_arg,
        calib_gui_arg,
        moveit_stack,
        detector,
        picker,
        coord_debug,
        calib_gui,
        image_viewer,
    ])
