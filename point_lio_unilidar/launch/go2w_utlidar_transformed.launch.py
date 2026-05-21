"""Point-LIO mapping for Go2W + Unitree L1 using the transformed cloud
pipeline (go2w_transform_sensor) and body IMU (/go2w/imu).

Topology:
  /utlidar/cloud --> [transform_cloud] --> /utlidar/transformed_cloud
                                                |
                                                v
                                        [pointlio_mapping]
                                                |
                                                v
                                  /aft_mapped_to_init (odom)
                                  /cloud_registered  (map cloud)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("point_lio_unilidar")
    desc_share = get_package_share_directory("go2w_description")

    config_path = os.path.join(pkg_share, "config", "go2w_utlidar_transformed.yaml")
    rviz_config_path = os.path.join(pkg_share, "rviz_cfg", "loam_livox2.rviz")

    xacro_path = os.path.join(desc_share, "xacro", "go2w.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch RViz2"
    )
    declare_rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_path,
        description="Path to RViz config file"
    )

    transform_cloud_node = Node(
        package="go2w_transform_sensor",
        executable="transform_cloud",
        name="transform_cloud",
        output="screen",
    )

    point_lio_node = Node(
        package="point_lio_unilidar",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            config_path,
            {
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 4,
                "space_down_sample": True,
                "filter_size_surf": 0.5,
                "filter_size_map": 0.5,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("/cloud_registered", "/registered_scan"),
            ("/aft_mapped_to_init", "/Odometry"),
        ],
    )

    robot_description_publisher_node = Node(
        package="go2w_description",
        executable="robot_description_publisher.py",
        name="robot_description_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joints_imu_node = Node(
        package="go2w_joints_state_and_imu_publisher",
        executable="go2w_joints_state_and_imu_publisher_node",
        name="go2w_joints_state_and_imu_publisher",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_rviz_cfg_arg,
        transform_cloud_node,
        robot_description_publisher_node,
        robot_state_publisher_node,
        joints_imu_node,
        point_lio_node,
        rviz_node,
    ])
