"""Self-contained launch for the current arm (joint1-6 + gripper, 7 motors).

`teleop_node` opens /dev/ttyUSB0 directly (dynamixel_sdk). Do NOT also start
`open_manipulator_x_bringup` — it would fight for the port and assumes the
old 4-DOF arm with IDs 11-15.

Hardware:
  joint1   XM430-W350   ID 31
  joint2   XM430-W350   ID 32
  joint3   XM430-W350   ID 33
  joint4   XM430-W210   ID 24
  joint5   XM430-W350   ID 35
  joint6   XM430-W210   ID 26
  gripper  XM430-W350   ID 37

Remote mapping:
  D-pad Left  / Right          → joint1  - / +    (button hold = move)
  D-pad Down  / Up             → joint2  - / +
  A           / Y              → joint3  - / +
  X           / B              → joint4  - / +
  Right stick down / up (ry)   → joint5  - / +    (analog, proportional)
  R1          / R2             → joint6  - / +
  L1                           → gripper OPEN
  L2                           → gripper CLOSE
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_velocity_arg = DeclareLaunchArgument(
        "joint_velocity",
        default_value="0.5",
        description="Joint velocity in rad/s while a remote button is held.",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="/dev/ttyUSB0",
        description="Serial device for U2D2.",
    )

    teleop = Node(
        package="go2w_remote_arm",
        executable="teleop_node",
        name="go2w_remote_arm",
        output="screen",
        parameters=[{
            "device": LaunchConfiguration("device"),
            "baudrate": 1000000,

            # ---- 7-motor config (joint1-6 + gripper) ----
            "joint_names": [
                "joint1", "joint2", "joint3", "joint4",
                "joint5", "joint6", "gripper",
            ],
            "motor_ids": [31, 32, 33, 24, 35, 26, 37],
            "joint_velocity": ParameterValue(
                LaunchConfiguration("joint_velocity"),
                value_type=float,
            ),
            "joint_signs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            # joint i uses analog axis (0=lx,1=ly,2=rx,3=ry) if >=0, else
            # falls back to button_pairs. joint5 is on ry (right stick up/down).
            "joint_axes": [-1, -1, -1, -1, 3, -1],
            # bit indices: 0=R1 1=L1 2=Start 3=Select 4=R2 5=L2 6=F1 7=F3
            # 8=A 9=B 10=X 11=Y 12=Up 13=Right 14=Down 15=Left
            # joint5 buttons unused (analog mode) → -1 placeholder.
            "button_pairs": [
                15, 13,  # j1: Left  / Right
                14, 12,  # j2: Down  / Up
                 8, 11,  # j3: A     / Y
                10,  9,  # j4: X     / B
                -1, -1,  # j5: (analog ry)
                 0,  4,  # j6: R1    / R2
            ],
            "gripper_btn_open": 1,    # L1
            "gripper_btn_close": 5,   # L2
            "gripper_open_target": -1.0,
            "gripper_close_target": 0.5,
            "enable_joint_command_subscriber": True,
            "joint_command_topic": "/go2w_remote_arm/joint_command",
        }],
        emulate_tty=True,
    )

    # Make `ros2 launch` exit immediately when teleop_node dies (e.g. after
    # the F1 park-and-release sequence). Without this, the launch parent stays
    # alive a few seconds waiting for cleanup, and the launcher node would
    # still see the child as "running" — so a quick F3 after F1 would be
    # interpreted as toggle-OFF instead of toggle-ON.
    exit_when_teleop_dies = RegisterEventHandler(
        OnProcessExit(target_action=teleop,
                       on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([
        joint_velocity_arg,
        device_arg,
        teleop,
        exit_when_teleop_dies,
    ])
