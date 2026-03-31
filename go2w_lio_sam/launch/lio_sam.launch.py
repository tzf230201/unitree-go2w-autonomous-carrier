from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("go2w_description")
    lio_sam_share = get_package_share_directory("go2w_lio_sam")
    urdf_path = os.path.join(desc_share, "urdf", "go2w_description.urdf")
    rviz_config_path = os.path.join(lio_sam_share, "launch", "lio_sam.rviz")
    hesai_config_path = os.path.join(lio_sam_share, "config", "hesai_config.yaml")
    lio_sam_params_path = os.path.join(lio_sam_share, "config", "lio_sam_params.yaml")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

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
            package="go2w_joints_state_and_imu_publisher",
            executable="go2w_joints_state_and_imu_publisher_node",
            name="go2w_joints_state_and_imu_publisher",
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
            remappings=[
                ("input_cloud",  "/lidar_points"),
                ("output_cloud", "/points"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_broadcaster",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
            parameters=[LaunchConfiguration("lio_sam_params_file")],
            output="screen",
        ),
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
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]

    return LaunchDescription(actions)
