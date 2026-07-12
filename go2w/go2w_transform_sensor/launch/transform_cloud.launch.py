from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    output_frame_id = LaunchConfiguration('output_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/utlidar/cloud'),
        DeclareLaunchArgument('output_topic',
                              default_value='/utlidar/transformed_cloud'),
        DeclareLaunchArgument('output_frame_id',
                              default_value='utlidar_lidar_aligned'),

        Node(
            package='go2w_transform_sensor',
            executable='transform_cloud',
            name='transform_cloud',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'output_frame_id': output_frame_id,
            }],
        ),
    ])
