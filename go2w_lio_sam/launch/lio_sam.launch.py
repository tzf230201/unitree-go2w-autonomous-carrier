from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lio_sam_share = get_package_share_directory("go2w_lio_sam")
    rviz_config_path = os.path.join(lio_sam_share, "launch", "lio_sam.rviz")
    hesai_config_path = os.path.join(lio_sam_share, "config", "hesai_config.yaml")
    lio_sam_params_path = os.path.join(lio_sam_share, "config", "lio_sam_params.yaml")

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "hesai_config_path",
            default_value=hesai_config_path,
            description="Path to Hesai config.yaml.",
        ),
        DeclareLaunchArgument(
            "hesai_namespace",
            default_value="hesai_ros_driver",
            description="Namespace for Hesai driver node.",
        ),
        DeclareLaunchArgument(
            "lio_sam_params_file",
            default_value=lio_sam_params_path,
            description="Path to LIO-SAM params.yaml.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to launch RViz.",
        ),
        Node(
            package="go2w_lio_sam",
            executable="lowstate_to_lio_imu",
            name="lowstate_to_lio_imu",
            output="screen",
            parameters=[
                {"input_lowstate_topic": "/lowstate"},
                {"output_imu_topic": "/lio_imu_raw"},
                {"frame_id": "imu"},
                {"use_rpy_orientation": False},
                {"calibration_samples": 1000},
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_broadcaster",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.4",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_broadcaster",
            arguments=[
                "--x", "-0.02557",
                "--y", "0.0",
                "--z", "0.04232",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "imu",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_hesai_broadcaster",
            arguments=[
                "--x", "0.145",
                "--y", "0.0",
                "--z", "0.11",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "1.57079632679",
                "--frame-id", "base_link",
                "--child-frame-id", "hesai_lidar",
            ],
            output="screen",
        ),
        Node(
            package="hesai_ros_driver",
            executable="hesai_ros_driver_node",
            namespace=LaunchConfiguration("hesai_namespace"),
            name="hesai_ros_driver_node",
            output="screen",
            parameters=[{"config_path": LaunchConfiguration("hesai_config_path")}],
        ),
        Node(
            package="go2w_lio_sam",
            executable="hesai_to_liosam_converter",
            name="hesai_to_liosam_converter",
            output="screen",
            parameters=[{"output_frame": "hesai_lidar"}],
            remappings=[
                ("input_cloud",  "/lidar_points"),
                ("output_cloud", "/points"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_broadcaster",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
            output="screen",
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="lio_sam",
                    executable="lio_sam_imuPreintegration",
                    name="lio_sam_imuPreintegration",
                    parameters=[LaunchConfiguration("lio_sam_params_file")],
                    output="screen",
                ),
                Node(
                    package="lio_sam",
                    executable="lio_sam_imageProjection",
                    name="lio_sam_imageProjection",
                    parameters=[LaunchConfiguration("lio_sam_params_file")],
                    output="screen",
                ),
                Node(
                    package="lio_sam",
                    executable="lio_sam_featureExtraction",
                    name="lio_sam_featureExtraction",
                    parameters=[LaunchConfiguration("lio_sam_params_file")],
                    output="screen",
                ),
                Node(
                    package="lio_sam",
                    executable="lio_sam_mapOptimization",
                    name="lio_sam_mapOptimization",
                    parameters=[LaunchConfiguration("lio_sam_params_file")],
                    output="screen",
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]

    return LaunchDescription(actions)
