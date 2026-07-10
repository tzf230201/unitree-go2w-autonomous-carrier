"""Bringup launch for the 6-DOF OpenManipulator Chain on this rig.

Starts ros2_control_node with the dxl_hw_interface plugin, robot_state_publisher
(so TF is published), and spawns three controllers:
  - joint_state_broadcaster
  - arm_controller        (joint_trajectory_controller for joint1..6)
  - gripper_controller    (position_controllers/GripperActionController for `gripper`)

⚠ Do NOT launch this together with go2w_remote_arm's `teleop_node` — both want
to own /dev/ttyUSB0.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port_name", default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        description="U2D2 serial device.",
    )
    baud_arg = DeclareLaunchArgument(
        "baudrate", default_value="1000000",
        description="Dynamixel baud rate.",
    )
    fake_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="false",
        description="Use ros2_control fake_components instead of real Dynamixels.",
    )

    pkg = FindPackageShare("om_chain_bringup")

    # robot_description from xacro
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg, "urdf", "om_chain.urdf.xacro"]),
        " port_name:=", LaunchConfiguration("port_name"),
        " baudrate:=", LaunchConfiguration("baudrate"),
        " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
    ])
    robot_description = {"robot_description": ParameterValue(
        robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                    "--controller-manager", "/controller_manager"],
    )
    grip_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                    "--controller-manager", "/controller_manager"],
    )

    # Stage controllers: jsb first, then the two action controllers in parallel
    grip_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner, grip_spawner])
    )

    return LaunchDescription([
        port_arg, baud_arg, fake_arg,
        ros2_control_node,
        robot_state_publisher,
        jsb_spawner,
        grip_after_jsb,
    ])
