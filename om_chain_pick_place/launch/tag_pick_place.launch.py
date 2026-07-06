"""Launch the AprilTag-driven pick-and-place PoC.

The MoveIt stack must already be running:

  Terminal 1:   sudo systemctl stop go2w-arm-launcher.service
                ros2 launch om_chain_moveit_config demo.launch.py
  Terminal 2:   ros2 launch om_chain_pick_place tag_pick_place.launch.py

Then:
  ros2 service call /tag_world std_srvs/srv/Trigger      # verify extrinsic
  ros2 service call /run_tag_pick std_srvs/srv/Trigger   # run the sequence
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om_chain_pick_place"),
            "config", "tag_pick.yaml",
        ]),
        description="Parameters for apriltag_detector + tag_pick_place.",
    )
    waypoints_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om_chain_pick_place"),
            "config", "waypoints.yaml",
        ]),
        description="Waypoints YAML (start_pose / place_above / place).",
    )

    detector = Node(
        package="om_chain_pick_place",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        emulate_tty=True,
    )
    picker = Node(
        package="om_chain_pick_place",
        executable="tag_pick_place_node",
        name="tag_pick_place",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"waypoints_file": LaunchConfiguration("waypoints_file")},
        ],
        emulate_tty=True,
    )

    return LaunchDescription([config_arg, waypoints_arg, detector, picker])
