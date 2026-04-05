import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────────
    pkg_share = get_package_share_directory("go2w_nav2")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    nav2_params_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    slam_params_path = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    twist_mux_params_path = os.path.join(pkg_share, "config", "twist_mux_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "nav2.rviz")

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
        "rviz",
        default_value="true",
        description="Launch RViz2",
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

    # ── Twist mux ─────────────────────────────────────────────────────
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params_path],
        remappings=[
            ("cmd_vel_out", "cmd_vel"),
        ],
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

    return LaunchDescription([
        declare_params_arg,
        declare_slam_params_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        slam_toolbox_launch,
        nav2_navigation_launch,
        twist_mux_node,
        rviz_node,
    ])
