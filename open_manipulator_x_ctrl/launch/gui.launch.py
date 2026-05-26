"""All-in-one launch: starts the OM-X controller, the RealSense camera, and
the Tk GUI.

Usage:
    ros2 launch open_manipulator_x_ctrl gui.launch.py
    ros2 launch open_manipulator_x_ctrl gui.launch.py use_camera:=false
    ros2 launch open_manipulator_x_ctrl gui.launch.py device:=/dev/ttyUSB1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params",
        default_value=PathJoinSubstitution([
            FindPackageShare("open_manipulator_x_ctrl"),
            "config",
            "params.yaml",
        ]),
        description="YAML params file for the controller node.",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="/dev/ttyUSB0",
        description="Serial device for U2D2.",
    )
    use_camera_arg = DeclareLaunchArgument(
        "use_camera", default_value="true",
        description="Start the RealSense2 camera node.",
    )
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value="/camera/camera/color/image_raw",
        description="Image topic the GUI subscribes to. realsense2_camera 4.x "
                    "uses /<namespace>/<name>/color/image_raw, which with the "
                    "default name='camera' under namespace 'camera' becomes "
                    "/camera/camera/color/image_raw.",
    )
    rs_profile_arg = DeclareLaunchArgument(
        "rs_color_profile", default_value="640,480,30",
        description="RealSense color profile (W,H,FPS). D405 supports e.g. 848,480,30.",
    )
    grip_open_arg = DeclareLaunchArgument(
        "grip_open_target", default_value="-1.0",
        description="Initial gripper OPEN target (rad). Override per robot.",
    )
    grip_close_arg = DeclareLaunchArgument(
        "grip_close_target", default_value="0.0",
        description="Initial gripper CLOSE target (rad). Override per robot.",
    )

    controller_node = Node(
        package="open_manipulator_x_ctrl",
        executable="controller_node",
        name="open_manipulator_x_ctrl",
        output="screen",
        parameters=[
            LaunchConfiguration("params"),
            {"device": LaunchConfiguration("device")},
        ],
        emulate_tty=True,
    )

    # RealSense — name='camera' (no namespace) → /camera/color/image_raw
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera")),
        parameters=[{
            "enable_color": True,
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_sync": False,
            "pointcloud.enable": False,
            "rgb_camera.color_profile": LaunchConfiguration("rs_color_profile"),
        }],
        emulate_tty=True,
    )

    gui_node = Node(
        package="open_manipulator_x_ctrl",
        executable="gui",
        name="open_manipulator_x_gui",
        output="screen",
        parameters=[{
            "camera_topic": LaunchConfiguration("camera_topic"),
            "grip_open_target": LaunchConfiguration("grip_open_target"),
            "grip_close_target": LaunchConfiguration("grip_close_target"),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        device_arg,
        use_camera_arg,
        camera_topic_arg,
        rs_profile_arg,
        grip_open_arg,
        grip_close_arg,
        GroupAction([controller_node, realsense_node, gui_node]),
    ])
