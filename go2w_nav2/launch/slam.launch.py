import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def package_available(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────────
    pkg_share = get_package_share_directory("go2w_nav2")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    has_fast_lio_launch = package_available("go2w_fast_lio2")
    if has_fast_lio_launch:
        fast_lio_share = get_package_share_directory("go2w_fast_lio2")

    nav2_params_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    slam_params_path = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "nav2.rviz")
    has_twist_mux = package_available("twist_mux")
    has_pointcloud_to_laserscan = package_available("pointcloud_to_laserscan")
    has_odom_projector = package_available("go2w_odom_projector")
    has_cmd_vel_control = package_available("go2w_cmd_vel_control")
    if has_twist_mux:
        twist_mux_params_path = os.path.join(pkg_share, "config", "twist_mux_params.yaml")

    # ── Launch arguments ───────────────────────────────────────────────
    declare_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_params_path,
        description="Full path to the Nav2 parameters file",
    )
    declare_slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_path,
        description="Full path to the SLAM Toolbox parameters file",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    declare_rviz_arg = DeclareLaunchArgument(
        "nav2_rviz",
        default_value="true",
        description="Launch Nav2 RViz2",
    )
    declare_launch_fast_lio_arg = DeclareLaunchArgument(
        "launch_fast_lio",
        default_value="true",
        description="Launch go2w_fast_lio2 together with SLAM/Nav2 and keep FAST-LIO RViz disabled",
    )
    declare_launch_cmd_vel_bridge_arg = DeclareLaunchArgument(
        "launch_cmd_vel_bridge",
        default_value="true",
        description="Launch go2w_cmd_vel_control to bridge /cmd_vel into /api/sport/request",
    )

    if has_fast_lio_launch:
        fast_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_share, "launch", "fast_lio2.launch.py")
            ),
            launch_arguments={
                "rviz": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_fast_lio")),
        )

    # ── SLAM Toolbox (online async) ───────────────────────────────────
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ── Nav2 navigation (without map server / AMCL) ───────────────────
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    delayed_nav_stack = TimerAction(
        period=3.0,
        actions=[
            slam_toolbox_launch,
            nav2_navigation_launch,
        ],
        condition=IfCondition(LaunchConfiguration("launch_fast_lio")),
    )

    # Convert the body-frame FAST-LIO cloud into a 2D LaserScan for SLAM Toolbox
    if has_pointcloud_to_laserscan:
        pointcloud_to_laserscan_node = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", "/cloud_registered_body"),
                ("scan", "/scan"),
            ],
            parameters=[{
                "target_frame": "base_footprint",
                "transform_tolerance": 0.20,
                "min_height": -0.15,
                "max_height": 0.30,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.05,
                "range_min": 0.20,
                "range_max": 30.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "use_cloud_stamp": False,
            }],
        )

    if has_odom_projector:
        odom_projector_node = Node(
            package="go2w_odom_projector",
            executable="odom_projector",
            name="go2w_odom_projector",
            output="screen",
            parameters=[{
                "input_topic": "/Odometry",
                "output_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_tf": True,
                "zero_initial_pose": True,
                "use_input_stamp": False,
            }],
        )

    if has_cmd_vel_control:
        cmd_vel_control_node = Node(
            package="go2w_cmd_vel_control",
            executable="go2w_cmd_vel_control_node",
            name="go2w_cmd_vel_control",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "request_topic": "/api/sport/request",
            }],
            condition=IfCondition(LaunchConfiguration("launch_cmd_vel_bridge")),
        )

    # ── Twist mux ─────────────────────────────────────────────────────
    launch_actions = [
        declare_params_arg,
        declare_slam_params_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        declare_launch_fast_lio_arg,
        declare_launch_cmd_vel_bridge_arg,
    ]

    if has_fast_lio_launch:
        launch_actions.extend([
            fast_lio_launch,
            delayed_nav_stack,
        ])
    else:
        launch_actions.extend([
            slam_toolbox_launch,
            nav2_navigation_launch,
        ])

    if not has_fast_lio_launch:
        launch_actions.append(
            LogInfo(
                msg="go2w_fast_lio2 package not found; /Odometry and point cloud topics will need external publishers."
            )
        )

    if has_fast_lio_launch:
        launch_actions.append(
            LogInfo(
                msg="launch_fast_lio defaults to true; set launch_fast_lio:=false to use an external FAST-LIO instance."
            )
        )

    if has_pointcloud_to_laserscan:
        launch_actions.insert(4, pointcloud_to_laserscan_node)
    else:
        launch_actions.append(
            LogInfo(
                msg="pointcloud_to_laserscan package not found; /scan will need an external publisher."
            )
        )

    if has_odom_projector:
        launch_actions.insert(4, odom_projector_node)
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_odom_projector package not found; /odom will need an external planar odometry publisher."
            )
        )

    if has_cmd_vel_control:
        launch_actions.append(cmd_vel_control_node)
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_cmd_vel_control package not found; /cmd_vel will need an external bridge to drive the robot."
            )
        )

    if has_twist_mux:
        launch_actions.append(
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[twist_mux_params_path],
                remappings=[
                    ("cmd_vel_out", "cmd_vel"),
                ],
            )
        )
    else:
        launch_actions.append(
            LogInfo(
                msg="twist_mux package not found; launching SLAM/Nav2 without twist multiplexing."
            )
        )

    # ── RViz2 ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("nav2_rviz")),
    )

    launch_actions.append(rviz_node)

    return LaunchDescription(launch_actions)
