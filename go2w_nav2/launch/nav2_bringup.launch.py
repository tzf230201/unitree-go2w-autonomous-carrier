import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_nav2 = get_package_share_directory("go2w_nav2")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_cmd_vel = get_package_share_directory("go2w_cmd_vel_control")
    pkg_lio_sam = get_package_share_directory("lio_sam")
    pkg_odom_to_tf = get_package_share_directory("odom_to_tf_ros2")
    pkg_hesai = get_package_share_directory("hesai_lidar")
    pkg_description = get_package_share_directory("go2w_description")

    default_map = os.path.join(pkg_nav2, "maps", "blank_map.yaml")
    default_params = os.path.join(pkg_nav2, "config", "nav2_params.yaml")
    default_odom_to_tf_params = os.path.join(pkg_odom_to_tf, "config", "odom_to_tf.yaml")
    default_rviz_config = os.path.join(pkg_nav2, "rviz", "nav2.rviz")
    urdf_path = os.path.join(pkg_description, "urdf", "go2w_description.urdf")
    cmd_vel_control_launch = os.path.join(pkg_cmd_vel, "launch", "go2w_cmd_vel_control.launch.py")
    lio_sam_launch = os.path.join(pkg_lio_sam, "launch", "run.launch.py")
    hesai_launch = os.path.join(pkg_hesai, "launch", "hesai_lidar_launch.py")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    map_arg = LaunchConfiguration("map")
    params_arg = LaunchConfiguration("params_file")
    slam_arg = LaunchConfiguration("slam")
    use_sim_time_arg = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time_arg, value_type=bool)
    autostart_arg = LaunchConfiguration("autostart")
    odom_to_tf_params_arg = LaunchConfiguration("odom_to_tf_params_file")
    odom_yaw_offset_arg = LaunchConfiguration("odom_yaw_offset")
    cmd_vel_topic_arg = LaunchConfiguration("cmd_vel_topic")
    request_topic_arg = LaunchConfiguration("request_topic")
    start_hesai_arg = LaunchConfiguration("start_hesai")
    start_imu_arg = LaunchConfiguration("start_imu")
    start_robot_state_publisher_arg = LaunchConfiguration("start_robot_state_publisher")
    scan_target_frame_arg = LaunchConfiguration("scan_target_frame")
    scan_min_height_arg = ParameterValue(LaunchConfiguration("scan_min_height"), value_type=float)
    scan_max_height_arg = ParameterValue(LaunchConfiguration("scan_max_height"), value_type=float)
    publish_tf_rate_hz_arg = ParameterValue(
        LaunchConfiguration("publish_tf_rate_hz"), value_type=float
    )
    start_rviz_arg = LaunchConfiguration("start_rviz")
    rviz_config_arg = LaunchConfiguration("rviz_config")
    slam_nav2_arg = PythonExpression(
        ["'True' if '", slam_arg, "'.lower() in ['1','true','yes','on'] else 'False'"]
    )

    cmd_vel_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cmd_vel_control_launch),
        launch_arguments={
            "cmd_vel_topic": cmd_vel_topic_arg,
            "request_topic": request_topic_arg,
            "use_teleop": "false",
        }.items(),
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_sam_launch),
    )

    hesai_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hesai_launch),
        condition=IfCondition(start_hesai_arg),
    )

    imu_publisher = Node(
        package="go2w_imu_publisher",
        executable="go2w_imu_publisher_node",
        name="go2w_imu_publisher",
        output="screen",
        parameters=[
            {"input_lowstate_topic": "/lowstate"},
            {"output_imu_topic": "/dog_imu_raw"},
            {"frame_id": "imu_link"},
            {"use_sim_time": use_sim_time_param},
        ],
        condition=IfCondition(start_imu_arg),
    )

    joints_state_publisher = Node(
        package="go2w_joints_state_publisher",
        executable="go2w_joints_state_publisher_node",
        name="go2w_joints_state_publisher",
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        condition=IfCondition(start_robot_state_publisher_arg),
        # robot_state_publisher can take a few seconds to teardown cleanly.
        sigterm_timeout="15",
    )

    odom_to_tf = Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        # YAML + override (override wins)
        parameters=[
            odom_to_tf_params_arg,
            {
                # LaunchConfiguration is a string; force float so rclcpp accepts it as double.
                "odom_yaw_offset": ParameterValue(odom_yaw_offset_arg, value_type=float),
                # Keep TF "fresh" even if /lio_sam/mapping/odometry is low-rate/irregular.
                "publish_tf_rate_hz": publish_tf_rate_hz_arg,
                "use_sim_time": use_sim_time_param,
            },
        ],
        # rclcpp teardown can take a few seconds (DDS discovery, etc).
        # Avoid spurious "failed to terminate after 5s" warnings on Ctrl+C.
        sigterm_timeout="15",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_hesai",
        output="screen",
        remappings=[
            ("cloud_in", "/lidar_points"),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                # Publish /scan in a TF frame that exists in the URDF (default: hesai_lidar).
                "target_frame": scan_target_frame_arg,
                "transform_tolerance": 0.01,
                # Filter ground / self returns. Tweak if your environment differs.
                "min_height": scan_min_height_arg,
                "max_height": scan_max_height_arg,
                "angle_min": -3.142,
                "angle_max": 3.142,
                "angle_increment": 0.003141593,
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 12.0,
                # If false, empty rays become range_max and can look like a fake wall/arc.
                "use_inf": True,
                "inf_epsilon": 1.0,
                "use_sim_time": use_sim_time_param,
            }
        ],
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": slam_nav2_arg,
            "map": map_arg,
            "use_sim_time": use_sim_time_arg,
            "params_file": params_arg,
            "autostart": autostart_arg,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_arg],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(start_rviz_arg),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Path to map yaml file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Run SLAM (true) instead of static map localization",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "odom_to_tf_params_file",
                default_value=default_odom_to_tf_params,
                description="Path to odom_to_tf_ros2 parameter file",
            ),
            DeclareLaunchArgument(
                "odom_yaw_offset",
                default_value="1.57079632679",
                description=(
                    "Yaw rotation (rad) applied to incoming odometry before publishing TF. "
                    "If moving forward looks like moving sideways in RViz, try -1.5708 or +1.5708."
                ),
            ),
            DeclareLaunchArgument(
                "publish_tf_rate_hz",
                default_value="50.0",
                description="Republish latest odom->base_footprint TF at this rate (Hz).",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="cmd_vel topic for go2w_cmd_vel_control",
            ),
            DeclareLaunchArgument(
                "request_topic",
                default_value="/api/sport/request",
                description="request topic for go2w_cmd_vel_control",
            ),
            DeclareLaunchArgument(
                "start_hesai",
                default_value="true",
                description="Start Hesai lidar driver inside this launch",
            ),
            DeclareLaunchArgument(
                "start_imu",
                default_value="true",
                description="Start go2w_imu_publisher for /dog_imu_raw",
            ),
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
                description="Start robot_state_publisher for TF chain (including hesai_lidar frame)",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz for Nav2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for Nav2.",
            ),
            DeclareLaunchArgument(
                "scan_target_frame",
                default_value="hesai_lidar",
                description="pointcloud_to_laserscan target_frame (sets /scan frame_id).",
            ),
            DeclareLaunchArgument(
                "scan_min_height",
                default_value="-0.15",
                description="pointcloud_to_laserscan min_height in target_frame (meters).",
            ),
            DeclareLaunchArgument(
                "scan_max_height",
                default_value="1.5",
                description="pointcloud_to_laserscan max_height in target_frame (meters).",
            ),
            cmd_vel_control,
            hesai_lidar,
            imu_publisher,
            joints_state_publisher,
            robot_state_publisher,
            TimerAction(period=2.0, actions=[lio_sam]),
            TimerAction(period=4.0, actions=[odom_to_tf]),
            TimerAction(period=5.0, actions=[pointcloud_to_scan]),
            TimerAction(period=6.0, actions=[bringup]),
            TimerAction(period=7.0, actions=[rviz]),
        ]
    )
