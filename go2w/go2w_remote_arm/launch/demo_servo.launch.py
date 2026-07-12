"""All-in-one MoveIt Servo demo driven by the Go2W remote.

This is the MoveIt path: Servo does IK + collision + singularity + joint
limits. The remote only sends Cartesian twists.

  use_fake_hardware:=true  (default) → test in RViz, no robot
  use_fake_hardware:=false           → drive the real arm

    ros2 launch go2w_remote_arm demo_servo.launch.py
    ros2 launch go2w_remote_arm demo_servo.launch.py use_fake_hardware:=false

⚠ For the real arm, stop go2w-arm-launcher.service first (port conflict):
    sudo systemctl stop go2w-arm-launcher.service
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true",
        description="true = RViz sim (no robot); false = real Dynamixels.",
    )

    # 1. ros2_control + controllers (+ robot_state_publisher)
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py",
        ])),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )

    # 2. move_group (provides the planning scene Servo uses for collision)
    move_group = TimerAction(period=3.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"), "launch", "om6dof_moveit.launch.py",
        ])),
        launch_arguments={"start_rviz": "true"}.items(),
    )])

    # 3. Servo node
    servo = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"), "launch", "servo.launch.py",
        ])),
    )])

    # 4. Remote → Servo bridge
    bridge = TimerAction(period=6.0, actions=[Node(
        package="go2w_remote_arm",
        executable="remote_servo_bridge",
        name="remote_servo_bridge",
        output="screen",
        emulate_tty=True,
    )])

    return LaunchDescription([fake_arg, hardware, move_group, servo, bridge])
