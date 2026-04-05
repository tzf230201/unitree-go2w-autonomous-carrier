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

    nav2_params_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    twist_mux_params_path = os.path.join(pkg_share, "config", "twist_mux_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "nav2.rviz")

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

    # ── Twist mux ─────────────────────────────────────────────────────
    # Merges cmd_vel from Nav2 and teleop with priority
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
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        nav2_bringup_launch,
        twist_mux_node,
        rviz_node,
    ])
