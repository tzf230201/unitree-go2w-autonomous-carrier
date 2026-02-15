from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("go2w_description")
    lio_sam_share = get_package_share_directory("lio_sam")
    urdf_path = os.path.join(desc_share, "urdf", "go2w_description.urdf")
    rviz_config_path = os.path.join(desc_share, "launch", "display_lio_sam.rviz")
    default_lio_sam_params = os.path.join(lio_sam_share, "config", "params_xt16.yaml")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "lio_sam_params_file",
            default_value=default_lio_sam_params,
            description="Path to LIO-SAM ROS2 params YAML.",
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
            description=(
                "Yaw rotation (rad) applied to incoming odometry before publishing TF. "
                "If moving forward looks like moving sideways in RViz, try -1.5708 or +1.5708."
            ),
        ),
        DeclareLaunchArgument(
            "start_pointcloud_to_laserscan",
            default_value="false",
            description="Start pointcloud_to_laserscan (/lidar_points -> /scan). Useful for Nav2, not required for LIO-SAM.",
        ),
        DeclareLaunchArgument(
            "publish_map_to_odom_static",
            default_value="false",
            description="Publish a static TF map->odom. Keep this false when using SLAM Toolbox or AMCL (they publish map->odom).",
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
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
            # robot_state_publisher can take a few seconds to teardown cleanly.
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
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("hesai_lidar"),
                    "launch",
                    "hesai_lidar_launch.py",
                )
            ),
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
                    # LaunchConfiguration is a string; force float so rclcpp accepts it as double.
                    "odom_yaw_offset": ParameterValue(
                        LaunchConfiguration("odom_yaw_offset"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("start_odom_to_tf")),
            # rclcpp teardown can take a few seconds (DDS discovery, etc).
            # Avoid spurious "failed to terminate after 5s" warnings on Ctrl+C.
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
                    # If false, empty rays become range_max and can look like a fake wall/arc.
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                }
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_pointcloud_to_laserscan")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lio_sam_share, "launch", "run.launch.py")
            ),
            launch_arguments={
                "params_file": LaunchConfiguration("lio_sam_params_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_lio_sam")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ]

    return LaunchDescription(actions)
