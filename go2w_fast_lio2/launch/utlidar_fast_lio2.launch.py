import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("go2w_fast_lio2")
    desc_share = get_package_share_directory("go2w_description")

    spark_fast_lio_config_path = os.path.join(pkg_share, "config", "fast_lio2_unilidar.yaml")
    rviz_config_path = os.path.join(pkg_share, "launch", "utlidar_fast_lio2.rviz")

    xacro_path = os.path.join(desc_share, "xacro", "go2w.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2",
    )
    declare_rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_path,
        description="Path to RViz config file",
    )

    # utlidar_lidar TF chain (mirrors go2w_point_lio/utlidar_tf_chain.py).
    # base -> utlidar_rot (radar_joint xyz, roll +180°)
    # utlidar_rot -> utlidar_lidar (yaw +55° CCW)
    static_tf_base_to_utlidar_rot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_utlidar_rot",
        output="log",
        arguments=[
            "--x", "0.28945",
            "--y", "0.0",
            "--z", "-0.046825",
            "--roll", str(math.pi),
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base",
            "--child-frame-id", "utlidar_rot",
        ],
    )

    static_tf_utlidar_rot_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_utlidar_rot_to_lidar",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", str(math.radians(55.0)),
            "--frame-id", "utlidar_rot",
            "--child-frame-id", "utlidar_lidar",
        ],
    )

    spark_fast_lio_node = Node(
        package="spark_fast_lio",
        executable="spark_lio_mapping",
        name="spark_lio_mapping",
        output="screen",
        parameters=[
            spark_fast_lio_config_path,
            {"use_sim_time": False},
        ],
        remappings=[
            ("lidar", "/utlidar/cloud"),
            ("imu", "/go2w/imu"),
            ("odometry", "/Odometry"),
            ("cloud_registered", "/cloud_registered"),
            ("cloud_registered_base", "/cloud_registered_body"),
            ("path", "/path"),
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

    actions = [
        declare_rviz_arg,
        declare_rviz_cfg_arg,
        static_tf_base_to_utlidar_rot,
        static_tf_utlidar_rot_to_lidar,
        robot_description_publisher_node,
        robot_state_publisher_node,
        spark_fast_lio_node,
        joints_imu_node,
        rviz_node,
    ]

    return LaunchDescription(actions)
