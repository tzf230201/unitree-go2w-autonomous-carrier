from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    start_go2w_imu = LaunchConfiguration('start_go2w_imu')
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_tf = LaunchConfiguration('odom_tf')
    odom_tf_parent_frame = LaunchConfiguration('odom_tf_parent_frame')
    odom_tf_child_frame = LaunchConfiguration('odom_tf_child_frame')
    rviz = LaunchConfiguration('rviz')
    start_hesai = LaunchConfiguration('start_hesai')
    hesai_config_path = LaunchConfiguration('hesai_config_path')
    hesai_namespace = LaunchConfiguration('hesai_namespace')
    dlio_extrinsics_yaml = LaunchConfiguration('dlio_extrinsics_yaml')

    dlio_pkg = FindPackageShare('direct_lidar_inertial_odometry')
    wrapper_pkg = FindPackageShare('go2w_dlio')

    dlio_yaml_path = PathJoinSubstitution([dlio_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([dlio_pkg, 'cfg', 'params.yaml'])
    wrapper_override_yaml = PathJoinSubstitution([wrapper_pkg, 'config', 'go2w_dlio_params.yaml'])
    wrapper_extrinsics_yaml = PathJoinSubstitution([wrapper_pkg, 'config', 'go2w_dlio_extrinsics.yaml'])
    hesai_default_config = PathJoinSubstitution([wrapper_pkg, 'config', 'hesai_config.yaml'])

    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/lidar_points',
        description='Pointcloud topic from HesaiLidar_ROS_2.0',
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu_raw',
        description='IMU topic from go2w_imu_publisher',
    )
    declare_start_go2w_imu_arg = DeclareLaunchArgument(
        'start_go2w_imu',
        default_value='true',
        description='Start go2w_imu_publisher node',
    )
    declare_lowstate_topic_arg = DeclareLaunchArgument(
        'lowstate_topic',
        default_value='/lowstate',
        description='Input lowstate topic for go2w_imu_publisher',
    )
    declare_imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='IMU frame id for go2w_imu_publisher',
    )
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used for odom->tf',
    )
    declare_odom_tf_arg = DeclareLaunchArgument(
        'odom_tf',
        default_value='true',
        description='Enable odom to tf broadcaster',
    )
    declare_odom_tf_parent_frame_arg = DeclareLaunchArgument(
        'odom_tf_parent_frame',
        default_value='odom',
        description='Override parent frame for odom->tf',
    )
    declare_odom_tf_child_frame_arg = DeclareLaunchArgument(
        'odom_tf_child_frame',
        default_value='base_footprint',
        description='Override child frame for odom->tf',
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz',
    )
    declare_start_hesai_arg = DeclareLaunchArgument(
        'start_hesai',
        default_value='true',
        description='Start Hesai driver together with DLIO',
    )
    declare_hesai_config_path_arg = DeclareLaunchArgument(
        'hesai_config_path',
        default_value=hesai_default_config,
        description='Path to Hesai config.yaml (default uses go2w_dlio/config/hesai_config.yaml)',
    )
    declare_hesai_namespace_arg = DeclareLaunchArgument(
        'hesai_namespace',
        default_value='hesai_ros_driver',
        description='Namespace for Hesai driver node',
    )
    declare_dlio_extrinsics_yaml_arg = DeclareLaunchArgument(
        'dlio_extrinsics_yaml',
        default_value=wrapper_extrinsics_yaml,
        description='Path to YAML file that overrides DLIO baselink<->lidar and baselink<->imu extrinsics',
    )
    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        namespace=hesai_namespace,
        output='screen',
        parameters=[{'config_path': hesai_config_path}],
        condition=IfCondition(start_hesai),
    )

    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path, wrapper_override_yaml, dlio_extrinsics_yaml],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', odom_topic),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path, wrapper_override_yaml, dlio_extrinsics_yaml],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    urdf_file = '/home/unitree/ros2_ws/src/unitree-go2w-autonomous-carrier/go2w_description_modified/urdf/go2w_description.urdf'
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joints_state_publisher_node = Node(
        package='go2w_joints_state_and_imu_publisher',
        executable='go2w_joints_state_and_imu_publisher_node',
        name='go2w_joints_state_and_imu_publisher',
        output='screen',
        parameters=[{
            'input_lowstate_topic': lowstate_topic,
            'output_imu_topic': imu_topic,
            'frame_id': imu_frame_id,
        }],
        condition=IfCondition(start_go2w_imu),
    )

    rviz_config_path = PathJoinSubstitution([wrapper_pkg, 'launch', 'display_dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='go2w_dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_start_go2w_imu_arg,
        declare_lowstate_topic_arg,
        declare_imu_frame_id_arg,
        declare_odom_topic_arg,
        declare_odom_tf_arg,
        declare_odom_tf_parent_frame_arg,
        declare_odom_tf_child_frame_arg,
        declare_rviz_arg,
        declare_start_hesai_arg,
        declare_hesai_config_path_arg,
        declare_hesai_namespace_arg,
        declare_dlio_extrinsics_yaml_arg,
        robot_state_publisher_node,
        joints_state_publisher_node,
        hesai_node,
        dlio_odom_node,
        dlio_map_node,
        rviz_node,
    ])
