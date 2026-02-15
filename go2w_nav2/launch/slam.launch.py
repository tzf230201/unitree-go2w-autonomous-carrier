import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_nav2 = get_package_share_directory("go2w_nav2")
    pkg_description = get_package_share_directory("go2w_description")
    pkg_lio_sam = get_package_share_directory("lio_sam")
    pkg_hesai = get_package_share_directory("hesai_lidar")

    urdf_path = os.path.join(pkg_description, "urdf", "go2w_description.urdf")
    rviz_config_path = os.path.join(pkg_nav2, "rviz", "slam.rviz")
    default_lio_sam_params = os.path.join(pkg_lio_sam, "config", "params_xt16.yaml")
    default_slam_toolbox_params = os.path.join(pkg_nav2, "config", "slam_toolbox_params.yaml")

    lio_sam_launch = os.path.join(pkg_lio_sam, "launch", "run.launch.py")
    hesai_launch = os.path.join(pkg_hesai, "launch", "hesai_lidar_launch.py")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration("use_sim_time")

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "lio_sam_params_file",
            default_value=default_lio_sam_params,
            description="Path to LIO-SAM ROS2 params YAML.",
        ),
        DeclareLaunchArgument(
            "slam_toolbox_params_file",
            default_value=default_slam_toolbox_params,
            description="Path to SLAM Toolbox params YAML.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Start robot_state_publisher with go2w URDF.",
        ),
        DeclareLaunchArgument(
            "start_joint_state_publisher",
            default_value="true",
            description="Start go2w_joints_state_publisher.",
        ),
        DeclareLaunchArgument(
            "start_hesai",
            default_value="true",
            description="Start Hesai lidar driver (hesai_lidar_launch.py).",
        ),
        DeclareLaunchArgument(
            "start_imu",
            default_value="true",
            description="Start go2w_imu_publisher.",
        ),
        DeclareLaunchArgument(
            "start_lio_sam",
            default_value="true",
            description="Start LIO-SAM nodes by including lio_sam/launch/run.launch.py.",
        ),
        DeclareLaunchArgument(
            "start_odom_to_tf",
            default_value="true",
            description="Start odom_to_tf_ros2 (publishes odom -> base_footprint TF from LIO-SAM odometry topic).",
        ),
        DeclareLaunchArgument(
            "odom_yaw_offset",
            default_value="1.57079632679",
            description="Yaw rotation (rad) applied to incoming odometry before publishing TF.",
        ),
        DeclareLaunchArgument(
            "start_pointcloud_to_laserscan",
            default_value="true",
            description="Start pointcloud_to_laserscan (/lidar_points -> /scan). Required for slam_toolbox.",
        ),
        DeclareLaunchArgument(
            "start_slam_toolbox",
            default_value="true",
            description="Start slam_toolbox async node (publishes /map and TF map->odom).",
        ),
        DeclareLaunchArgument(
            "publish_map_to_odom_static",
            default_value="false",
            description="Publish a static TF map->odom. Keep this false when using SLAM Toolbox.",
        ),
        Node(
            package="go2w_description",
            executable="robot_description_publisher.py",
            name="robot_description_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
            sigterm_timeout="15",
        ),
        Node(
            package="go2w_joints_state_publisher",
            executable="go2w_joints_state_publisher_node",
            name="go2w_joints_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_joint_state_publisher")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hesai_launch),
            condition=IfCondition(LaunchConfiguration("start_hesai")),
        ),
        Node(
            package="go2w_imu_publisher",
            executable="go2w_imu_publisher_node",
            name="go2w_imu_publisher",
            output="screen",
            parameters=[
                {"input_lowstate_topic": "/lowstate"},
                {"output_imu_topic": "/dog_imu_raw"},
                {"frame_id": "imu_link"},
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("start_imu")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_map_to_odom_static")),
        ),
        Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_to_tf",
            output="screen",
            parameters=[
                {
                    "frame_id": "odom",
                    "child_frame_id": "base_footprint",
                    "odom_topic_1": "/lio_sam/mapping/odometry",
                    "odom_topic_2": "",
                    "pose_topic": "",
                    "sport_mode_topic": "",
                    "odom_yaw_offset": ParameterValue(
                        LaunchConfiguration("odom_yaw_offset"), value_type=float
                    ),
                    "use_sim_time": use_sim_time,
                }
            ],
            condition=IfCondition(LaunchConfiguration("start_odom_to_tf")),
            sigterm_timeout="15",
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_hesai",
            remappings=[
                ("cloud_in", "/lidar_points"),
                ("scan", "/scan"),
            ],
            parameters=[
                {
                    "target_frame": "base_footprint",
                    "transform_tolerance": 0.01,
                    "min_height": -0.3,
                    "max_height": 3.0,
                    "angle_min": -3.142,
                    "angle_max": 3.142,
                    "angle_increment": 0.003141593,
                    "scan_time": 0.1,
                    "range_min": 0.1,
                    "range_max": 12.0,
                    "use_inf": False,
                    "inf_epsilon": 1.0,
                }
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_pointcloud_to_laserscan")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_sam_launch),
            launch_arguments={
                "params_file": LaunchConfiguration("lio_sam_params_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_lio_sam")),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration("slam_toolbox_params_file"),
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("start_slam_toolbox")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ]

    return LaunchDescription(actions)

