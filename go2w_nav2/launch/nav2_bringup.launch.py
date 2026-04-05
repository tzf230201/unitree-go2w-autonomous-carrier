import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
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

    nav2_params_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "nav2.rviz")
    has_twist_mux = package_available("twist_mux")
    has_odom_projector = package_available("go2w_odom_projector")
    if has_twist_mux:
        twist_mux_params_path = os.path.join(pkg_share, "config", "twist_mux_params.yaml")

    # ── Launch arguments ───────────────────────────────────────────────
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to the map yaml file to load",
    )
    declare_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_params_path,
        description="Full path to the Nav2 parameters file",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2",
    )

    # ── Nav2 bringup ──────────────────────────────────────────────────
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
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
            }],
        )

    # ── Twist mux ─────────────────────────────────────────────────────
    # Merges cmd_vel from Nav2 and teleop with priority
    launch_actions = [
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        nav2_bringup_launch,
    ]

    if has_odom_projector:
        launch_actions.insert(4, odom_projector_node)
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_odom_projector package not found; /odom will need an external planar odometry publisher."
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
                msg="twist_mux package not found; launching Nav2 without twist multiplexing."
            )
        )

    # ── RViz2 ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    launch_actions.append(rviz_node)

    return LaunchDescription(launch_actions)
