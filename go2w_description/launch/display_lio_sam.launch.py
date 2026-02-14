from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("go2w_description")
    hesai_share = get_package_share_directory("hesai_lidar")
    lio_sam_share = get_package_share_directory("lio_sam")
    odom_to_tf_share = get_package_share_directory("odom_to_tf_ros2")
    urdf_path = os.path.join(desc_share, "urdf", "go2w_description.urdf")
    rviz_config_path = os.path.join(desc_share, "launch", "display_lio_sam.rviz")
    hesai_correction = os.path.join(hesai_share, "config", "PandarXT-16.csv")
    odom_to_tf_params_path = os.path.join(odom_to_tf_share, "config", "odom_to_tf.yaml")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "odom_to_tf_params",
            default_value=odom_to_tf_params_path,
            description="Path to odom_to_tf params YAML.",
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
        ),
        Node(
            package="go2w_joints_state_publisher",
            executable="go2w_joints_state_publisher_node",
            name="go2w_joints_state_publisher",
            output="screen",
        ),
        Node(
            package="hesai_lidar",
            executable="hesai_lidar_node",
            name="hesai_node",
            output="screen",
            parameters=[
                {"pcap_file": ""},
                {"server_ip": "192.168.123.20"},
                {"lidar_recv_port": 2368},
                {"gps_port": 10110},
                {"start_angle": 0.0},
                {"lidar_type": "PandarXT-16"},
                {"frame_id": "hesai_lidar"},
                {"pcldata_type": 0},
                {"publish_type": "both"},
                {"timestamp_type": "realtime"},
                {"data_type": ""},
                {"lidar_correction_file": hesai_correction},
                {"multicast_ip": ""},
                {"coordinate_correction_flag": False},
                {"fixed_frame": ""},
                {"target_frame": ""},
            ],
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
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_to_tf",
            output="screen",
            parameters=[LaunchConfiguration("odom_to_tf_params")],
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
                    "range_max": 8.0,
                    "use_inf": False,
                    "inf_epsilon": 1.0,
                }
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lio_sam_share, "launch", "run.launch.py")
            ),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]

    return LaunchDescription(actions)
