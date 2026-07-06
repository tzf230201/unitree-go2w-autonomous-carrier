"""Bridge: Go2W wireless remote → MoveIt Servo.

This is the MoveIt path (collision-aware, singularity-handled). Unlike
teleop_node (which does its own PyKDL IK and drives Dynamixels directly), this
node only translates remote inputs into a TwistStamped that MoveIt Servo
consumes. Servo does the IK, collision checking, singularity scaling and joint
limit enforcement, then commands the joint_trajectory_controller.

Mapping (WORLD frame, robot_link_command_frame=world in moveit_servo.yaml):
  Up / Down        → linear +z / -z
  Left / Right     → linear +y / -y
  Y / A            → linear +x / -x
  X / B            → angular +x / -x   (roll about world x)
  right stick ry   → angular y         (pitch about world y)
  R1 / R2          → angular +z / -z   (yaw about world z)
  L1 / L2          → gripper open / close (GripperCommand action)
  Start            → /servo_node/start_servo
  Select           → /servo_node/stop_servo

Servo auto-starts on the first non-zero command.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import WirelessController
from geometry_msgs.msg import TwistStamped
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


# Go2W remote bit positions
BTN_R1 = 1 << 0
BTN_L1 = 1 << 1
BTN_START = 1 << 2
BTN_SELECT = 1 << 3
BTN_R2 = 1 << 4
BTN_L2 = 1 << 5
BTN_A = 1 << 8
BTN_B = 1 << 9
BTN_X = 1 << 10
BTN_Y = 1 << 11
BTN_UP = 1 << 12
BTN_RIGHT = 1 << 13
BTN_DOWN = 1 << 14
BTN_LEFT = 1 << 15


class RemoteServoBridge(Node):
    def __init__(self) -> None:
        super().__init__("remote_servo_bridge")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("deadzone", 0.08)
        # unitless command magnitudes in [-1, 1] (Servo scales by its config)
        self.declare_parameter("lin_cmd", 1.0)
        self.declare_parameter("ang_cmd", 1.0)
        # sign flips: [vx, vy, vz, wx, wy, wz]
        self.declare_parameter("axis_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("gripper_action", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open", 0.019)
        self.declare_parameter("gripper_close", -0.010)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.lin_cmd = float(self.get_parameter("lin_cmd").value)
        self.ang_cmd = float(self.get_parameter("ang_cmd").value)
        self.signs = [float(x) for x in self.get_parameter("axis_signs").value]
        if len(self.signs) < 6:
            self.signs += [1.0] * (6 - len(self.signs))
        self.grip_open = float(self.get_parameter("gripper_open").value)
        self.grip_close = float(self.get_parameter("gripper_close").value)

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            WirelessController, "/wirelesscontroller", self._on_remote, qos_be
        )
        self.pub_twist = self.create_publisher(
            TwistStamped, str(self.get_parameter("twist_topic").value), 10
        )
        self.cli_start = self.create_client(Trigger, "/servo_node/start_servo")
        self.cli_stop = self.create_client(Trigger, "/servo_node/stop_servo")
        self.act_grip = ActionClient(
            self, GripperCommand, str(self.get_parameter("gripper_action").value)
        )

        self.keys = 0
        self.prev_keys = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry
        self.servo_started = False

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            "remote_servo_bridge ready — MoveIt Servo path (collision-aware). "
            "Up/Down/Left/Right/Y/A = translate, X/B = roll, stick = pitch, "
            "R1/R2 = yaw, L1/L2 = gripper. Servo auto-starts on first motion."
        )

    def _on_remote(self, msg: WirelessController) -> None:
        self.prev_keys = self.keys
        self.keys = int(msg.keys)
        self.axes = (float(msg.lx), float(msg.ly), float(msg.rx), float(msg.ry))
        rising = (~self.prev_keys) & self.keys
        if rising & BTN_L1:
            self._gripper(self.grip_open, "open")
        if rising & BTN_L2:
            self._gripper(self.grip_close, "close")
        if rising & BTN_START:
            self._servo(True)
        if rising & BTN_SELECT:
            self._servo(False)

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def _tick(self) -> None:
        keys = self.keys
        lin = self.lin_cmd
        ang = self.ang_cmd
        s = self.signs

        vx = (lin if keys & BTN_Y else 0.0) - (lin if keys & BTN_A else 0.0)
        vy = (lin if keys & BTN_LEFT else 0.0) - (lin if keys & BTN_RIGHT else 0.0)
        vz = (lin if keys & BTN_UP else 0.0) - (lin if keys & BTN_DOWN else 0.0)
        wx = (ang if keys & BTN_X else 0.0) - (ang if keys & BTN_B else 0.0)
        wy = ang * self._dz(self.axes[3])  # right stick ry
        wz = (ang if keys & BTN_R1 else 0.0) - (ang if keys & BTN_R2 else 0.0)

        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.twist.linear.x = s[0] * vx
        t.twist.linear.y = s[1] * vy
        t.twist.linear.z = s[2] * vz
        t.twist.angular.x = s[3] * wx
        t.twist.angular.y = s[4] * wy
        t.twist.angular.z = s[5] * wz

        nonzero = any(abs(v) > 1e-6 for v in
                      (vx, vy, vz, wx, wy, wz))
        if nonzero and not self.servo_started:
            self._servo(True)
        # Always publish (zeros included) so Servo holds/halts cleanly.
        self.pub_twist.publish(t)

    def _servo(self, start: bool) -> None:
        cli = self.cli_start if start else self.cli_stop
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())
            self.servo_started = start
            self.get_logger().info(f"servo {'STARTED' if start else 'STOPPED'}")
        else:
            self.get_logger().warn(
                f"{'start' if start else 'stop'}_servo service not ready",
                throttle_duration_sec=2.0,
            )

    def _gripper(self, position: float, label: str) -> None:
        if not self.act_grip.server_is_ready():
            self.get_logger().warn(f"gripper {label}: action server not ready")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 5.0
        self.act_grip.send_goal_async(goal)
        self.get_logger().info(f"gripper {label} ({position:+.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = RemoteServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
