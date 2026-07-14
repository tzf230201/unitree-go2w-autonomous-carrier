from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    package_share = FindPackageShare("om6dof_bringup")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([package_share, "urdf", "om6dof.urdf.xacro"]),
            " port_name:=", port_name,
            " baud_rate:=", baud_rate,
            " use_fake_hardware:=", use_fake_hardware,
        ]),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[PathJoinSubstitution([package_share, "config", "controllers.yaml"])],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )
    forward_position_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
    )
    start_motion_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_spawner, gripper_spawner, forward_position_spawner],
        )
    )
    stop_launch_if_hardware_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason="ros2_control hardware owner exited",
            ))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("port_name", default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0"),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        state_publisher,
        control_node,
        joint_state_spawner,
        start_motion_controllers,
        stop_launch_if_hardware_exits,
    ])
