"""Go2W joystick adapter for the canonical OM6DOF command topics.

This node has no kinematics, controller-manager client, or final joint-position
publisher. It only converts remote input into ``/om6dof/operation_mode`` and
``/om6dof/control_cmd``. The gripper continues to use its standard ros2_control
action server.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from unitree_go.msg import LowState, WirelessController

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
    MODE_STARTUP,
    next_motion_mode,
)


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


def _decode_lowstate_remote(
    wireless_remote: Sequence[int],
) -> Tuple[int, Tuple[float, float, float, float]]:
    """Decode Unitree's 40-byte controller payload as keys and lx/ly/rx/ry."""
    raw = bytes(wireless_remote)
    if len(raw) != 40:
        raise ValueError("wireless_remote must contain exactly 40 bytes")
    keys = int(raw[2]) | (int(raw[3]) << 8)
    axes = (
        struct.unpack_from("<f", raw, 4)[0],
        struct.unpack_from("<f", raw, 20)[0],
        struct.unpack_from("<f", raw, 8)[0],
        struct.unpack_from("<f", raw, 12)[0],
    )
    if not all(math.isfinite(value) for value in axes):
        raise ValueError("wireless_remote contains a non-finite joystick axis")
    return keys, axes


class JointTeleop(Node):
    """Translate Go2W remote samples into the two generic arm command topics."""

    def __init__(self) -> None:
        super().__init__("om6dof_teleop")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("wirelesscontroller_topic", "/wirelesscontroller")
        self.declare_parameter(
            "operation_mode_topic", "/om6dof/operation_mode"
        )
        self.declare_parameter("control_cmd_topic", "/om6dof/control_cmd")
        self.declare_parameter(
            "operation_mode_state_topic", "/om6dof/operation_mode/state"
        )
        self.declare_parameter(
            "remote_enabled_state_topic", "/om6dof/remote_enabled/state"
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("remote_command_timeout_seconds", 0.5)
        self.declare_parameter("lowstate_preference_timeout_seconds", 0.1)
        self.declare_parameter("remote_action_debounce_seconds", 0.3)
        self.declare_parameter("mode_request_timeout_seconds", 2.0)
        self.declare_parameter("joint_axis_deadzone", 0.08)
        self.declare_parameter("joint_velocity", 0.5)
        self.declare_parameter("cartesian_linear_speed", 0.05)
        self.declare_parameter("cartesian_angular_speed", 0.5)
        self.declare_parameter("cylindrical_theta_speed", 0.25)
        self.declare_parameter(
            "button_pairs", [15, 13, 14, 12, 8, 11, 10, 9, -1, -1, 0, 4]
        )
        self.declare_parameter("joint_axes", [-1, -1, -1, -1, 3, -1])
        self.declare_parameter("joint_signs", [1.0] * 6)

        self.declare_parameter(
            "gripper_action", "/gripper_controller/gripper_cmd"
        )
        self.declare_parameter(
            "gripper_command_topic", "/om6dof_teleop/gripper_cmd"
        )
        self.declare_parameter(
            "gripper_state_topic", "/om6dof_teleop/gripper_state"
        )
        self.declare_parameter("gripper_joint", "gripper_left_joint")
        self.declare_parameter("gripper_open_target", 0.019)
        self.declare_parameter("gripper_close_target", -0.010)
        self.declare_parameter("grasp_current_threshold", 70.0)
        self.declare_parameter("grasp_consecutive", 3)
        self.declare_parameter("grip_close_tolerance", 0.0015)
        self.declare_parameter("grip_open_tolerance", 0.0020)

        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.remote_timeout = float(
            self.get_parameter("remote_command_timeout_seconds").value
        )
        self.lowstate_preference_timeout = float(
            self.get_parameter("lowstate_preference_timeout_seconds").value
        )
        self.action_debounce = float(
            self.get_parameter("remote_action_debounce_seconds").value
        )
        self.mode_request_timeout = float(
            self.get_parameter("mode_request_timeout_seconds").value
        )
        self.deadzone = float(self.get_parameter("joint_axis_deadzone").value)
        self.joint_velocity = float(self.get_parameter("joint_velocity").value)
        self.cartesian_linear_speed = float(
            self.get_parameter("cartesian_linear_speed").value
        )
        self.cartesian_angular_speed = float(
            self.get_parameter("cartesian_angular_speed").value
        )
        self.cylindrical_theta_speed = float(
            self.get_parameter("cylindrical_theta_speed").value
        )
        scalar_values = (
            self.rate,
            self.remote_timeout,
            self.lowstate_preference_timeout,
            self.action_debounce,
            self.mode_request_timeout,
            self.deadzone,
            self.joint_velocity,
            self.cartesian_linear_speed,
            self.cartesian_angular_speed,
            self.cylindrical_theta_speed,
        )
        if (
            not all(math.isfinite(value) for value in scalar_values)
            or self.rate <= 0.0
            or self.remote_timeout <= 0.0
            or self.lowstate_preference_timeout < 0.0
            or self.action_debounce < 0.0
            or self.mode_request_timeout <= 0.0
            or not 0.0 <= self.deadzone < 1.0
            or self.joint_velocity < 0.0
            or self.cartesian_linear_speed < 0.0
            or self.cartesian_angular_speed < 0.0
            or self.cylindrical_theta_speed < 0.0
        ):
            raise RuntimeError("invalid remote mapping parameters")
        self.dt = 1.0 / self.rate

        packed_pairs = [int(value) for value in self.get_parameter(
            "button_pairs"
        ).value]
        if len(packed_pairs) != 12:
            raise RuntimeError("button_pairs must contain 12 integers")
        self.button_pairs = [
            (packed_pairs[2 * index], packed_pairs[2 * index + 1])
            for index in range(6)
        ]
        self.joint_axes = [
            int(value) for value in self.get_parameter("joint_axes").value
        ]
        self.joint_signs = [
            float(value) for value in self.get_parameter("joint_signs").value
        ]
        if len(self.joint_axes) != 6 or len(self.joint_signs) != 6:
            raise RuntimeError("joint_axes and joint_signs must have six values")

        self.lock = threading.RLock()
        self.remote_enabled = False
        self.control_mode = MODE_JOINT
        self.f1_destination: Optional[str] = None
        self.mode_request_pending_until = 0.0
        self.remote_waiting_for_neutral = True
        self.keys = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)
        self.lowstate_keys = 0
        self.lowstate_axes = (0.0, 0.0, 0.0, 0.0)
        self.event_keys = 0
        self.event_axes = (0.0, 0.0, 0.0, 0.0)
        self.last_lowstate_remote = 0.0
        self.last_event_remote = 0.0
        self.last_remote_action: Dict[int, float] = {}

        self.gripper_joint = str(self.get_parameter("gripper_joint").value)
        self.grip_open = float(
            self.get_parameter("gripper_open_target").value
        )
        self.grip_close = float(
            self.get_parameter("gripper_close_target").value
        )
        self.grasp_threshold = float(
            self.get_parameter("grasp_current_threshold").value
        )
        self.grasp_consecutive = int(
            self.get_parameter("grasp_consecutive").value
        )
        self.grip_close_tolerance = float(
            self.get_parameter("grip_close_tolerance").value
        )
        self.grip_open_tolerance = float(
            self.get_parameter("grip_open_tolerance").value
        )
        self.grip_action: Optional[str] = None
        self.grip_state = "?"
        self.grip_hold_count = 0
        self.last_grip_publish = 0.0

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        lowstate_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.operation_pub = self.create_publisher(
            String,
            str(self.get_parameter("operation_mode_topic").value),
            mode_qos,
        )
        self.control_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("control_cmd_topic").value),
            command_qos,
        )
        self.gripper_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("gripper_state_topic").value),
            state_qos,
        )
        self.create_subscription(
            LowState,
            str(self.get_parameter("lowstate_topic").value),
            self._on_lowstate,
            lowstate_qos,
        )
        self.create_subscription(
            WirelessController,
            str(self.get_parameter("wirelesscontroller_topic").value),
            self._on_wireless_event,
            event_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("remote_enabled_state_topic").value),
            self._on_remote_state,
            state_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operation_mode_state_topic").value),
            self._on_operation_state,
            state_qos,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 20
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_command_topic").value),
            self._on_gripper_command,
            10,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            str(self.get_parameter("gripper_action").value),
        )

        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            "teleop adapter ready: Go2W -> /om6dof/operation_mode + "
            "/om6dof/control_cmd; kinematics live in om6dof_controller"
        )

    def _deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * min(
            1.0, (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        )

    def _remote_joint_velocity(
        self, keys: int, axes: Sequence[float]
    ) -> List[float]:
        result = [0.0] * 6
        for index in range(6):
            sign = self.joint_signs[index]
            axis_index = self.joint_axes[index]
            if 0 <= axis_index < len(axes):
                result[index] = (
                    self.joint_velocity
                    * sign
                    * self._deadzone(float(axes[axis_index]))
                )
                continue
            decrease, increase = self.button_pairs[index]
            if 0 <= increase < 16 and keys & (1 << increase):
                result[index] += self.joint_velocity * sign
            if 0 <= decrease < 16 and keys & (1 << decrease):
                result[index] -= self.joint_velocity * sign
        return result

    def _remote_coordinate_velocity(
        self, keys: int, axes: Sequence[float]
    ) -> List[float]:
        linear = self.cartesian_linear_speed
        angular = self.cartesian_angular_speed
        first = linear if keys & BTN_Y else 0.0
        if keys & BTN_A:
            first -= linear
        second_speed = (
            self.cylindrical_theta_speed
            if self.control_mode == MODE_CYLINDRICAL
            else linear
        )
        second = second_speed if keys & BTN_LEFT else 0.0
        if keys & BTN_RIGHT:
            second -= second_speed
        vertical = linear if keys & BTN_UP else 0.0
        if keys & BTN_DOWN:
            vertical -= linear
        roll = angular if keys & BTN_X else 0.0
        if keys & BTN_B:
            roll -= angular
        pitch = angular * (
            self._deadzone(float(axes[3])) if len(axes) > 3 else 0.0
        )
        yaw = angular if keys & BTN_R1 else 0.0
        if keys & BTN_R2:
            yaw -= angular
        return [first, second, vertical, roll, pitch, yaw]

    def _remote_motion_active(
        self, keys: int, axes: Sequence[float]
    ) -> bool:
        motion_buttons = (
            BTN_R1
            | BTN_R2
            | BTN_X
            | BTN_B
            | BTN_Y
            | BTN_A
            | BTN_UP
            | BTN_DOWN
            | BTN_LEFT
            | BTN_RIGHT
        )
        return bool(keys & motion_buttons) or (
            len(axes) > 3
            and abs(self._deadzone(float(axes[3]))) > 0.0
        )

    def _selected_remote_state_locked(self, now: float):
        lowstate_age = (
            now - self.last_lowstate_remote
            if self.last_lowstate_remote else math.inf
        )
        event_age = (
            now - self.last_event_remote
            if self.last_event_remote else math.inf
        )
        if lowstate_age <= self.lowstate_preference_timeout:
            return self.lowstate_keys, self.lowstate_axes, self.last_lowstate_remote
        if event_age <= self.remote_timeout:
            return self.event_keys, self.event_axes, self.last_event_remote
        return 0, (0.0, 0.0, 0.0, 0.0), 0.0

    def _on_lowstate(self, msg: LowState) -> None:
        try:
            keys, axes = _decode_lowstate_remote(msg.wireless_remote)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"invalid /lowstate wireless_remote payload: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        self._process_remote_sample(keys, axes, "lowstate")

    def _on_wireless_event(self, msg: WirelessController) -> None:
        self._process_remote_sample(
            int(msg.keys),
            (float(msg.lx), float(msg.ly), float(msg.rx), float(msg.ry)),
            "event",
        )

    def _process_remote_sample(
        self, keys: int, axes: Sequence[float], source: str
    ) -> None:
        operation_request = None
        gripper = None
        with self.lock:
            now = time.monotonic()
            keys = int(keys) & 0xFFFF
            axes = tuple(float(value) for value in axes)
            if len(axes) != 4 or not all(
                math.isfinite(value) for value in axes
            ):
                self.get_logger().warn(f"invalid {source} remote axes ignored")
                return

            if source == "lowstate":
                previous_source_keys = self.lowstate_keys
                self.lowstate_keys = keys
                self.lowstate_axes = axes
                self.last_lowstate_remote = now
            elif source == "event":
                stale = (
                    not self.last_event_remote
                    or now - self.last_event_remote > self.remote_timeout
                )
                previous_source_keys = 0 if stale else self.event_keys
                self.event_keys = keys
                self.event_axes = axes
                self.last_event_remote = now
            else:
                raise ValueError(f"unknown remote source: {source}")

            self.keys, self.axes, _ = self._selected_remote_state_locked(now)
            rising = (~previous_source_keys) & keys
            if (
                source == "event"
                and self.last_lowstate_remote
                and now - self.last_lowstate_remote
                <= self.lowstate_preference_timeout
            ):
                rising &= ~self.lowstate_keys
            elif (
                source == "lowstate"
                and self.last_event_remote
                and now - self.last_event_remote <= self.remote_timeout
            ):
                rising &= ~self.event_keys

            def accepted_rising(button: int) -> bool:
                if not rising & button:
                    return False
                previous = float(self.last_remote_action.get(button, 0.0))
                if previous and now - previous < self.action_debounce:
                    return False
                self.last_remote_action[button] = now
                return True

            ownership_rising = accepted_rising(BTN_F3)
            if ownership_rising:
                if now < self.mode_request_pending_until:
                    self.get_logger().warn(
                        "F3 ignored: previous ownership request is pending"
                    )
                else:
                    operation_request = (
                        MODE_AUTONOMOUS if self.remote_enabled else MODE_JOINT
                    )
                    self.mode_request_pending_until = (
                        now + self.mode_request_timeout
                    )
                    if operation_request == MODE_JOINT:
                        self.remote_waiting_for_neutral = True

            elif accepted_rising(BTN_SELECT):
                if self.remote_enabled:
                    self.control_mode = next_motion_mode(self.control_mode)
                    operation_request = self.control_mode
                    self.remote_waiting_for_neutral = True
                else:
                    self.get_logger().warn(
                        "Select ignored: enable remote control with F3 first"
                    )

            elif accepted_rising(BTN_F1):
                if not self.remote_enabled:
                    self.get_logger().warn(
                        "F1 ignored: enable remote control with F3 first"
                    )
                else:
                    operation_request = (
                        MODE_STARTUP
                        if self.f1_destination == MODE_READY
                        else MODE_READY
                    )
                    self.control_mode = MODE_JOINT
                    self.remote_waiting_for_neutral = True

            if self.remote_enabled and accepted_rising(BTN_L1):
                gripper = "open"
            if self.remote_enabled and accepted_rising(BTN_L2):
                gripper = "close"

            if self.remote_enabled and self.remote_waiting_for_neutral:
                if not self._remote_motion_active(self.keys, self.axes):
                    self.remote_waiting_for_neutral = False
                    self.get_logger().info(
                        "remote motion armed after neutral input"
                    )

        if operation_request is not None:
            self.operation_pub.publish(String(data=operation_request))
            self.get_logger().info(
                f"remote operation request -> {operation_request}"
            )
        if gripper is not None:
            self._command_gripper(gripper)

    def _on_remote_state(self, msg: Bool) -> None:
        with self.lock:
            previous = self.remote_enabled
            self.remote_enabled = bool(msg.data)
            self.mode_request_pending_until = 0.0
            if self.remote_enabled and not previous:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_READY
                self.remote_waiting_for_neutral = True

    def _on_operation_state(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        with self.lock:
            if mode == MODE_AUTONOMOUS:
                self.remote_enabled = False
                self.mode_request_pending_until = 0.0
            elif mode in (MODE_JOINT, MODE_CARTESIAN, MODE_CYLINDRICAL):
                changed = mode != self.control_mode
                self.control_mode = mode
                if changed:
                    self.remote_waiting_for_neutral = True
            elif mode == MODE_READY:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_READY
                self.remote_waiting_for_neutral = True
            elif mode == MODE_STARTUP:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_STARTUP
                self.remote_waiting_for_neutral = True

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.gripper_joint)
        except ValueError:
            return
        if index >= len(msg.position):
            return
        current = (
            abs(float(msg.effort[index])) if index < len(msg.effort) else 0.0
        )
        self._update_gripper(float(msg.position[index]), current)

    def _on_gripper_command(self, msg: String) -> None:
        self._command_gripper(msg.data.strip().lower())

    def _command_gripper(self, command: str) -> None:
        if command == "open":
            target = self.grip_open
        elif command == "close":
            target = self.grip_close
        else:
            self.get_logger().warn(f"gripper command '{command}' rejected")
            return
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                "gripper action server is not ready",
                throttle_duration_sec=2.0,
            )
            return
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = 0.0
        self.gripper_client.send_goal_async(goal)
        with self.lock:
            self.grip_action = command
            self.grip_hold_count = 0
        self.get_logger().info(f"gripper -> {command.upper()}")

    def _update_gripper(self, position: float, current: float) -> None:
        with self.lock:
            at_open = abs(position - self.grip_open) <= self.grip_open_tolerance
            at_close = (
                abs(position - self.grip_close) <= self.grip_close_tolerance
            )
            if current >= self.grasp_threshold and not at_close:
                self.grip_hold_count += 1
            else:
                self.grip_hold_count = 0
            holding = self.grip_hold_count >= self.grasp_consecutive
            previous = self.grip_state
            if self.grip_action == "open":
                self.grip_state = "OPEN" if at_open else "OPENING"
            elif self.grip_action == "close":
                if holding:
                    self.grip_state = "HOLDING"
                elif at_close:
                    self.grip_state = (
                        "DROPPED"
                        if previous in ("HOLDING", "DROPPED")
                        else "CLOSED"
                    )
                else:
                    self.grip_state = "CLOSING"
            else:
                self.grip_state = (
                    "OPEN" if at_open else ("CLOSED" if at_close else "MID")
                )
            state = self.grip_state
            now = time.monotonic()
            publish = state != previous or now - self.last_grip_publish >= 0.5
            if publish:
                self.last_grip_publish = now
        if publish:
            self.gripper_state_pub.publish(
                String(data=f"{state} pos={position:+.4f}m cur={current:.0f}mA")
            )

    def _tick(self) -> None:
        with self.lock:
            if not self.remote_enabled:
                return
            now = time.monotonic()
            keys, axes, stamp = self._selected_remote_state_locked(now)
            fresh = bool(
                not self.remote_waiting_for_neutral
                and stamp
                and now - stamp
                <= max(self.remote_timeout, self.lowstate_preference_timeout)
            )
            if not fresh:
                velocity = [0.0] * 6
            elif self.control_mode == MODE_JOINT:
                velocity = self._remote_joint_velocity(keys, axes)
            else:
                velocity = self._remote_coordinate_velocity(keys, axes)
        self.control_pub.publish(Float64MultiArray(data=velocity))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JointTeleop()
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
