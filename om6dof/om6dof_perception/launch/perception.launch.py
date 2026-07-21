from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "target_description",
            default_value="glass jar with a black lid on the floor",
        ),
        DeclareLaunchArgument(
            "target_class",
            default_value="bottle",
        ),
        DeclareLaunchArgument(
            "ee_mode",
            default_value="fixed_jaw_pixels",
            description="fixed_jaw_pixels, yolo, or disabled",
        ),
        DeclareLaunchArgument(
            "ee_left_pixel", default_value="[200, 430]",
        ),
        DeclareLaunchArgument(
            "ee_right_pixel", default_value="[540, 400]",
        ),
        DeclareLaunchArgument(
            "yolo_model_path",
            default_value=(
                "/home/unitree/.cache/om6dof_perception/yolox_s.onnx"
            ),
        ),
        DeclareLaunchArgument(
            "yolo_confidence",
            default_value="0.35",
        ),
        DeclareLaunchArgument(
            "web_stream_topic",
            default_value=(
                "/application_web_monitor/perception/image/compressed"
            ),
        ),
        Node(
            package="om6dof_perception",
            executable="perception_node",
            name="om6dof_perception",
            output="screen",
            parameters=[{
                "target_description": LaunchConfiguration(
                    "target_description"),
                "target_class": LaunchConfiguration("target_class"),
                "ee_mode": LaunchConfiguration("ee_mode"),
                "ee_left_pixel": LaunchConfiguration("ee_left_pixel"),
                "ee_right_pixel": LaunchConfiguration("ee_right_pixel"),
                "yolo_model_path": LaunchConfiguration("yolo_model_path"),
                "yolo_confidence": LaunchConfiguration("yolo_confidence"),
                "web_stream_topic": LaunchConfiguration("web_stream_topic"),
                "publish_debug_image": True,
            }],
        ),
    ])
