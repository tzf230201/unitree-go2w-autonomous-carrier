import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hesai_share = get_package_share_directory("hesai_lidar")
    default_correction = os.path.join(hesai_share, "config", "PandarXT-16.csv")

    server_ip = LaunchConfiguration("server_ip")
    lidar_recv_port = LaunchConfiguration("lidar_recv_port")
    gps_port = LaunchConfiguration("gps_port")
    lidar_type = LaunchConfiguration("lidar_type")
    frame_id = LaunchConfiguration("frame_id")
    correction_file = LaunchConfiguration("lidar_correction_file")
    timestamp_type = LaunchConfiguration("timestamp_type")
    publish_type = LaunchConfiguration("publish_type")
    pcap_file = LaunchConfiguration("pcap_file")
    multicast_ip = LaunchConfiguration("multicast_ip")
    data_type = LaunchConfiguration("data_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument("server_ip", default_value="192.168.123.20"),
            DeclareLaunchArgument("lidar_recv_port", default_value="2368"),
            DeclareLaunchArgument("gps_port", default_value="10110"),
            DeclareLaunchArgument("lidar_type", default_value="PandarXT-16"),
            DeclareLaunchArgument("frame_id", default_value="hesai_lidar"),
            DeclareLaunchArgument("lidar_correction_file", default_value=default_correction),
            DeclareLaunchArgument("timestamp_type", default_value="realtime"),
            DeclareLaunchArgument("publish_type", default_value="both"),
            DeclareLaunchArgument("pcap_file", default_value=""),
            DeclareLaunchArgument("multicast_ip", default_value=""),
            DeclareLaunchArgument("data_type", default_value=""),
            Node(
                package="hesai_lidar",
                namespace="",
                executable="hesai_lidar_node",
                name="hesai_node",
                output="screen",
                parameters=[
                    {"pcap_file": pcap_file},
                    {"server_ip": server_ip},
                    {"lidar_recv_port": lidar_recv_port},
                    {"gps_port": gps_port},
                    {"start_angle": 0.0},
                    {"lidar_type": lidar_type},
                    {"frame_id": frame_id},
                    {"pcldata_type": 0},
                    {"publish_type": publish_type},
                    {"timestamp_type": timestamp_type},
                    {"data_type": data_type},
                    {"lidar_correction_file": correction_file},
                    {"multicast_ip": multicast_ip},
                    {"coordinate_correction_flag": False},
                    {"fixed_frame": ""},
                    {"target_frame": ""},
                ],
            ),
        ]
    )


