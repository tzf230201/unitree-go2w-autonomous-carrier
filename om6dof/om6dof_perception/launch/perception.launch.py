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
            "ee_description",
            default_value="robot arm gripper with two fingers",
        ),
        DeclareLaunchArgument(
            "vlm_model", default_value="qwen3-vl:8b-instruct-q4_K_M"),
        DeclareLaunchArgument(
            "ollama_url",
            default_value="http://192.168.123.99:11434",
        ),
        DeclareLaunchArgument(
            "web_stream_topic",
            default_value="/application_web_monitor/image/compressed",
        ),
        Node(
            package="om6dof_perception",
            executable="perception_node",
            name="om6dof_perception",
            output="screen",
            parameters=[{
                "target_description": LaunchConfiguration(
                    "target_description"),
                "ee_description": LaunchConfiguration("ee_description"),
                "vlm_model": LaunchConfiguration("vlm_model"),
                "ollama_url": LaunchConfiguration("ollama_url"),
                "web_stream_topic": LaunchConfiguration("web_stream_topic"),
                "publish_debug_image": True,
            }],
        ),
    ])
