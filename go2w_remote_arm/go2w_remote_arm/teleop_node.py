"""
Go2W wireless remote -> OpenManipulator-X direct joint teleop.

This node does not use IK, MoveIt, or MoveIt Servo. It reads the current joint
positions from /joint_states, integrates held remote buttons into joint position
targets, and publishes JointTrajectory commands directly to arm_controller.

Button bitmask layout for Go2W remote:
    bit 0:R1   1:L1   2:Start  3:Select
    bit 4:R2   5:L2   6:F1     7:F3
    bit 8:A    9:B   10:X     11:Y
    bit12:Up  13:Right 14:Down 15:Left

Mapping:
    Left / Right arrows -> joint1
    Down / Up arrows    -> joint2
    A / Y               -> joint3
    X / B               -> joint4
    L1 / L2             -> gripper open / close
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unitree_go.msg import WirelessController


BTN_R1 = 1 << 0
BTN_L1 = 1 << 1
BTN_START = 1 << 2
BTN_SELECT = 1 << 3
BTN_R2 = 1 << 4
BTN_L2 = 1 << 5
BTN_F1 = 1 << 6
BTN_F3 = 1 << 7
BTN_A = 1 << 8
BTN_B = 1 << 9
BTN_X = 1 << 10
BTN_Y = 1 << 11
BTN_UP = 1 << 12
BTN_RIGHT = 1 << 13
BTN_DOWN = 1 << 14
BTN_LEFT = 1 << 15

BUTTON_JOINT_BINDINGS = (
    ("joint1", BTN_LEFT, BTN_RIGHT),  # 0x8000 / 0x2000
    ("joint2", BTN_DOWN, BTN_UP),     # 0x4000 / 0x1000
    ("joint3", BTN_A, BTN_Y),         # 0x0100 / 0x0800
    ("joint4", BTN_X, BTN_B),         # 0x0400 / 0x0200
)


def _edge_high(prev: int, curr: int, mask: int) -> bool:
    return bool((~prev) & curr & mask)


def _duration_msg(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1_000_000_000)
    msg = Duration()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


class Go2WRemoteArm(Node):
    def __init__(self) -> None:
        super().__init__("go2w_remote_arm")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("remote_timeout_sec", 0.5)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("joint_velocity", 0.5)  # rad/s while button is held
        self.declare_parameter("button_joint_signs", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter(
            "joint_min_limits",
            [-math.pi, -1.5, -1.5, -1.7],
        )
        self.declare_parameter(
            "joint_max_limits",
            [math.pi, 1.5, 1.4, 1.97],
        )
        self.declare_parameter("trajectory_time_sec", 0.08)
        self.declare_parameter("gripper_open", 0.019)
        self.declare_parameter("gripper_close", -0.01)
        self.declare_parameter("gripper_max_effort", 5.0)

        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.remote_timeout_sec = float(self.get_parameter("remote_timeout_sec").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_velocity = float(self.get_parameter("joint_velocity").value)
        self.button_joint_signs = list(self.get_parameter("button_joint_signs").value)
        self.joint_min_limits = list(self.get_parameter("joint_min_limits").value)
        self.joint_max_limits = list(self.get_parameter("joint_max_limits").value)
        self.trajectory_time_sec = float(self.get_parameter("trajectory_time_sec").value)
        self.grip_open = float(self.get_parameter("gripper_open").value)
        self.grip_close = float(self.get_parameter("gripper_close").value)
        self.grip_effort = float(self.get_parameter("gripper_max_effort").value)

        self._lock = threading.Lock()
        self._keys = 0
        self._prev_keys = 0
        self._last_remote_t = 0.0
        self._last_tick_t = time.monotonic()
        self._current_positions: dict[str, float] = {}
        self._target_positions: dict[str, float] = {}
        self._warned_no_joint_state = False

        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_rl = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            WirelessController,
            "/wirelesscontroller",
            self._on_remote,
            qos_be,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_rl,
        )
        self.pub_trajectory = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            qos_rl,
        )
        self.act_grip = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )

        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.get_logger().info(
            "go2w_remote_arm direct joint teleop ready. "
            "Left/Right=joint1, Up/Down=joint2, A/Y=joint3, X/B=joint4, "
            "L1=open gripper, L2=close gripper."
        )

    def _on_remote(self, msg: WirelessController) -> None:
        with self._lock:
            self._prev_keys = self._keys
            self._keys = int(msg.keys)
            self._last_remote_t = time.monotonic()

            if _edge_high(self._prev_keys, self._keys, BTN_L1):
                self._send_gripper(self.grip_open, "open")
            if _edge_high(self._prev_keys, self._keys, BTN_L2):
                self._send_gripper(self.grip_close, "close")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name not in self.joint_names:
                    continue
                position = float(position)
                self._current_positions[name] = position
                self._target_positions.setdefault(name, position)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.2, now - self._last_tick_t))
        self._last_tick_t = now

        with self._lock:
            stale = (
                (now - self._last_remote_t) > self.remote_timeout_sec
                if self._last_remote_t
                else True
            )
            keys = 0 if stale else self._keys
            commands = self._button_joint_commands(keys)

            if not commands:
                for name in self.joint_names:
                    if name in self._current_positions:
                        self._target_positions[name] = self._current_positions[name]
                return

            missing = [name for name in self.joint_names if name not in self._target_positions]
            if missing:
                if not self._warned_no_joint_state:
                    self.get_logger().warn(
                        "waiting for /joint_states before sending direct joint commands"
                    )
                    self._warned_no_joint_state = True
                return

            for joint_name, velocity in commands:
                self._target_positions[joint_name] = self._clamp_joint(
                    joint_name,
                    self._target_positions[joint_name] + velocity * dt,
                )

            positions = [self._target_positions[name] for name in self.joint_names]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = _duration_msg(self.trajectory_time_sec)
        msg.points.append(point)

        self.pub_trajectory.publish(msg)

    def _button_joint_commands(self, keys: int) -> list[tuple[str, float]]:
        commands = []
        for i, (joint_name, negative_button, positive_button) in enumerate(
            BUTTON_JOINT_BINDINGS
        ):
            direction = 0.0
            if keys & negative_button:
                direction -= 1.0
            if keys & positive_button:
                direction += 1.0
            if direction == 0.0:
                continue

            sign = (
                float(self.button_joint_signs[i])
                if i < len(self.button_joint_signs)
                else 1.0
            )
            commands.append((joint_name, self.joint_velocity * sign * direction))

        return commands

    def _clamp_joint(self, joint_name: str, position: float) -> float:
        try:
            index = self.joint_names.index(joint_name)
        except ValueError:
            return position

        if index < len(self.joint_min_limits):
            position = max(float(self.joint_min_limits[index]), position)
        if index < len(self.joint_max_limits):
            position = min(float(self.joint_max_limits[index]), position)
        return position

    def _send_gripper(self, position: float, label: str) -> None:
        if not self.act_grip.server_is_ready():
            self.get_logger().warn(f"gripper {label}: action server not ready")
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.grip_effort
        self.act_grip.send_goal_async(goal)
        self.get_logger().info(f"gripper {label}: pos={position:+.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = Go2WRemoteArm()
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
