import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def package_available(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    pkg_share = get_package_share_directory("go2w_nav2")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    nav2_params_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    slam_params_path = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "nav2.rviz")

    has_point_lio = package_available("point_lio_unilidar")
    has_transform_sensor = package_available("go2w_transform_sensor")
    has_odom_projector = package_available("go2w_odom_projector")
    has_cmd_vel_control = package_available("go2w_cmd_vel_control")

    mode = LaunchConfiguration("mode")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("point_lio_odom_topic")

    declare_mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        description="Nav2 mode: slam or localization. localization requires map:=/path/to/map.yaml",
    )
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to a map yaml file when mode:=localization",
    )
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
    declare_launch_point_lio_arg = DeclareLaunchArgument(
        "launch_point_lio",
        default_value="true",
        description="Launch point_lio_unilidar go2w_utlidar_transformed.launch.py",
    )
    declare_launch_cmd_vel_bridge_arg = DeclareLaunchArgument(
        "launch_cmd_vel_bridge",
        default_value="true",
        description="Launch go2w_cmd_vel_control to bridge /cmd_vel into /api/sport/request",
    )
    declare_cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/utlidar/transformed_cloud",
        description="Point cloud topic converted to /scan for SLAM Toolbox, AMCL, and costmaps",
    )
    declare_odom_topic_arg = DeclareLaunchArgument(
        "point_lio_odom_topic",
        default_value="/Odometry",
        description="Point-LIO odometry topic projected to planar /odom",
    )

    launch_actions = [
        declare_mode_arg,
        declare_map_arg,
        declare_params_arg,
        declare_slam_params_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        declare_launch_point_lio_arg,
        declare_launch_cmd_vel_bridge_arg,
        declare_cloud_topic_arg,
        declare_odom_topic_arg,
    ]

    if has_point_lio:
        point_lio_share = get_package_share_directory("point_lio_unilidar")
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        point_lio_share,
                        "launch",
                        "go2w_utlidar_transformed.launch.py",
                    )
                ),
                launch_arguments={
                    "rviz": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_point_lio")),
            )
        )
    else:
        launch_actions.append(
            LogInfo(
                msg="point_lio_unilidar package not found; expecting external Point-LIO, /Odometry, /utlidar/transformed_cloud, and /go2w/imu."
            )
        )

    if has_odom_projector:
        launch_actions.append(
            Node(
                package="go2w_odom_projector",
                executable="odom_projector",
                name="go2w_odom_projector",
                output="screen",
                parameters=[{
                    "input_topic": odom_topic,
                    "output_topic": "/odom",
                    "odom_frame": "odom",
                    "base_frame": "base_footprint",
                    "publish_tf": True,
                    "zero_initial_pose": True,
                    "use_input_stamp": False,
                }],
            )
        )
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_odom_projector package not found; /odom must be provided externally."
            )
        )

    if has_transform_sensor:
        launch_actions.append(
            Node(
                package="go2w_transform_sensor",
                executable="utlidar_ground_scan",
                name="utlidar_ground_scan",
                output="screen",
                parameters=[{
                    "input_cloud_topic": cloud_topic,
                    "output_scan_topic": "/scan",
                    "output_frame": "base_footprint",
                    "sensor_x": 0.28945,
                    "sensor_y": 0.0,
                    "sensor_z": 0.353175,
                    "min_obstacle_height": 0.10,
                    "max_obstacle_height": 0.90,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.00872664626,
                    "scan_time": 0.066,
                    "range_min": 0.35,
                    "range_max": 12.0,
                    "use_cloud_stamp": False,
                    "min_points_per_bin": 1,
                }],
            )
        )
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_transform_sensor package not found; /scan must be provided externally."
            )
        )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=LaunchConfigurationEquals("mode", "slam"),
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=LaunchConfigurationEquals("mode", "slam"),
    )

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "slam": "False",
        }.items(),
        condition=LaunchConfigurationEquals("mode", "localization"),
    )

    launch_actions.append(
        TimerAction(
            period=5.0,
            actions=[
                slam_toolbox_launch,
                nav2_navigation_launch,
                nav2_localization_launch,
            ],
        )
    )

    if has_cmd_vel_control:
        launch_actions.append(
            Node(
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
        )
    else:
        launch_actions.append(
            LogInfo(
                msg="go2w_cmd_vel_control package not found; /cmd_vel will not be bridged to the robot."
            )
        )

    launch_actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(LaunchConfiguration("nav2_rviz")),
        )
    )

    return LaunchDescription(launch_actions)
