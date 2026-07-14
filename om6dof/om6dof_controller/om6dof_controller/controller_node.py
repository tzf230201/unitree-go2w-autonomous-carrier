"""OM6DOF command converter and ros2_control ownership supervisor.

The public streaming interface is deliberately small:

* ``/om6dof/operation_mode`` selects JOINT, CARTESIAN, or CYLINDRICAL.
* ``/om6dof/control_cmd`` carries six velocity values whose units follow the
  selected mode.

This node is the only publisher of final arm positions. It reads feedback,
performs local velocity IK, enforces safety limits, and atomically switches
the six hardware interfaces between MoveIt's trajectory controller and the
remote forward-position controller.
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from .control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
    MODE_STARTUP,
    MOTION_MODES,
    clamp_positions,
    integrate_cylindrical_position,
    limit_norm,
    normalize_operation_mode,
    rotation_error,
    rotation_from_rotvec,
    step_toward,
    validated_control_command,
    validated_joint_positions,
)


DEFAULT_READY_JOINT_POSITIONS = (0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0)
DEFAULT_JOINT_LOWER = (
    -math.pi * 0.9,
    -math.pi * 0.65,
    -math.pi * 0.60,
    -math.pi * 0.9,
    -math.pi * 0.63,
    -math.pi * 0.9,
)
DEFAULT_JOINT_UPPER = (
    math.pi * 0.9,
    math.pi * 0.67,
    math.pi * 0.68,
    math.pi * 0.9,
    math.pi * 0.67,
    math.pi * 0.9,
)


class OM6DOFController(Node):
    """Convert canonical six-axis jog commands into safe joint positions."""

    def __init__(self) -> None:
        super().__init__("om6dof_controller")

        self.declare_parameter(
            "joint_names", [f"joint{index}" for index in range(1, 7)]
        )
        self.declare_parameter("arm_controller", "arm_controller")
        self.declare_parameter(
            "remote_controller", "forward_position_controller"
        )
        self.declare_parameter(
            "forward_command_topic", "/forward_position_controller/commands"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
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
        self.declare_parameter(
            "switch_controller_service", "/controller_manager/switch_controller"
        )
        self.declare_parameter(
            "list_controllers_service", "/controller_manager/list_controllers"
        )
        self.declare_parameter("switch_timeout_seconds", 2.0)
        self.declare_parameter("remote_enabled_on_start", False)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("joint_state_timeout_seconds", 1.0)
        self.declare_parameter("control_cmd_timeout_seconds", 0.3)
        self.declare_parameter("max_joint_command_velocity", 1.2)
        self.declare_parameter("joint_lower", list(DEFAULT_JOINT_LOWER))
        self.declare_parameter("joint_upper", list(DEFAULT_JOINT_UPPER))
        self.declare_parameter("joint_limit_margin", 0.02)
        self.declare_parameter("ready_pose", list(DEFAULT_READY_JOINT_POSITIONS))
        self.declare_parameter("pose_target_velocity", 0.5)
        self.declare_parameter("pose_target_tolerance", 0.01)
        self.declare_parameter("pose_target_timeout_seconds", 20.0)

        self.declare_parameter("ik_enabled", True)
        self.declare_parameter("ik_base_link", "world")
        self.declare_parameter("ik_tip_link", "end_effector_link")
        self.declare_parameter("ik_urdf_pkg", "om6dof_description")
        self.declare_parameter("ik_damping", 0.05)
        self.declare_parameter("max_cartesian_linear_velocity", 0.10)
        self.declare_parameter("max_cartesian_angular_velocity", 1.0)
        self.declare_parameter("max_cylindrical_theta_velocity", 0.5)
        self.declare_parameter("cylindrical_origin_xy", [0.012, 0.0])
        self.declare_parameter("cylindrical_min_radius", 0.03)
        self.declare_parameter("ik_position_gain", 4.0)
        self.declare_parameter("ik_rotation_gain", 3.0)
        self.declare_parameter("ik_tool_frame_rotation", True)
        self.declare_parameter("ik_max_target_lead", 0.04)
        self.declare_parameter("ik_max_joint_following_error", 0.30)
        self.declare_parameter("ik_manipulability_warning_threshold", 1.0e-6)
        self.declare_parameter("ik_self_collision", True)
        self.declare_parameter("ik_collision_radius", 0.025)

        self.joint_names = [
            str(value) for value in self.get_parameter("joint_names").value
        ]
        if len(self.joint_names) != 6:
            raise RuntimeError("joint_names must contain the six arm joints")
        self.arm_controller = str(self.get_parameter("arm_controller").value)
        self.remote_controller = str(
            self.get_parameter("remote_controller").value
        )
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.joint_state_timeout = float(
            self.get_parameter("joint_state_timeout_seconds").value
        )
        self.control_cmd_timeout = float(
            self.get_parameter("control_cmd_timeout_seconds").value
        )
        self.max_joint_velocity = float(
            self.get_parameter("max_joint_command_velocity").value
        )
        if (
            not all(math.isfinite(value) for value in (
                self.rate,
                self.joint_state_timeout,
                self.control_cmd_timeout,
                self.max_joint_velocity,
            ))
            or self.rate <= 0.0
            or self.joint_state_timeout <= 0.0
            or self.control_cmd_timeout <= 0.0
            or self.max_joint_velocity <= 0.0
        ):
            raise RuntimeError("invalid controller rate/timeout/velocity")
        self.nominal_dt = 1.0 / self.rate

        lower = validated_joint_positions(
            self.get_parameter("joint_lower").value, "joint_lower"
        )
        upper = validated_joint_positions(
            self.get_parameter("joint_upper").value, "joint_upper"
        )
        margin = float(self.get_parameter("joint_limit_margin").value)
        if (
            not math.isfinite(margin)
            or margin < 0.0
            or any(
                lo + margin >= hi - margin
                for lo, hi in zip(lower, upper)
            )
        ):
            raise RuntimeError("joint_limit_margin is incompatible with limits")
        self.joint_lower = [value + margin for value in lower]
        self.joint_upper = [value - margin for value in upper]
        self.ready_pose = clamp_positions(
            validated_joint_positions(
                self.get_parameter("ready_pose").value, "ready_pose"
            ),
            self.joint_lower,
            self.joint_upper,
        )
        self.pose_target_velocity = float(
            self.get_parameter("pose_target_velocity").value
        )
        self.pose_target_tolerance = float(
            self.get_parameter("pose_target_tolerance").value
        )
        self.pose_target_timeout = float(
            self.get_parameter("pose_target_timeout_seconds").value
        )

        self.max_cartesian_linear_velocity = float(
            self.get_parameter("max_cartesian_linear_velocity").value
        )
        self.max_cartesian_angular_velocity = float(
            self.get_parameter("max_cartesian_angular_velocity").value
        )
        self.max_cylindrical_theta_velocity = float(
            self.get_parameter("max_cylindrical_theta_velocity").value
        )
        self.cylindrical_origin_xy = np.asarray(
            self.get_parameter("cylindrical_origin_xy").value, dtype=float
        )
        self.cylindrical_min_radius = float(
            self.get_parameter("cylindrical_min_radius").value
        )
        self.ik_position_gain = float(
            self.get_parameter("ik_position_gain").value
        )
        self.ik_rotation_gain = float(
            self.get_parameter("ik_rotation_gain").value
        )
        self.ik_tool_frame_rotation = bool(
            self.get_parameter("ik_tool_frame_rotation").value
        )
        self.ik_max_target_lead = float(
            self.get_parameter("ik_max_target_lead").value
        )
        self.ik_max_joint_following_error = float(
            self.get_parameter("ik_max_joint_following_error").value
        )
        self.ik_manipulability_warning_threshold = float(
            self.get_parameter("ik_manipulability_warning_threshold").value
        )
        self.ik_self_collision = bool(
            self.get_parameter("ik_self_collision").value
        )
        self.ik_collision_radius = float(
            self.get_parameter("ik_collision_radius").value
        )
        coordinate_scalars = np.asarray([
            self.max_cartesian_linear_velocity,
            self.max_cartesian_angular_velocity,
            self.max_cylindrical_theta_velocity,
            self.cylindrical_min_radius,
            self.ik_position_gain,
            self.ik_rotation_gain,
            self.ik_max_target_lead,
            self.ik_max_joint_following_error,
            self.ik_manipulability_warning_threshold,
            self.ik_collision_radius,
            self.pose_target_velocity,
            self.pose_target_tolerance,
            self.pose_target_timeout,
        ])
        if (
            not np.all(np.isfinite(coordinate_scalars))
            or self.max_cartesian_linear_velocity <= 0.0
            or self.max_cartesian_angular_velocity <= 0.0
            or self.max_cylindrical_theta_velocity <= 0.0
            or self.cylindrical_min_radius < 0.0
            or self.ik_position_gain < 0.0
            or self.ik_rotation_gain < 0.0
            or self.ik_max_target_lead <= 0.0
            or self.ik_max_joint_following_error <= 0.0
            or self.ik_manipulability_warning_threshold < 0.0
            or self.ik_collision_radius <= 0.0
            or self.pose_target_velocity <= 0.0
            or self.pose_target_tolerance <= 0.0
            or self.pose_target_timeout <= 0.0
            or self.cylindrical_origin_xy.shape != (2,)
            or not np.all(np.isfinite(self.cylindrical_origin_xy))
        ):
            raise RuntimeError("invalid pose/Cartesian/cylindrical parameters")

        self.lock = threading.RLock()
        self.motion_mode = MODE_AUTONOMOUS
        self.remote_enabled = False
        self.arm_controller_active = False
        self.remote_controller_active = False
        self.switch_in_progress = False
        self.switch_target: Optional[bool] = None
        self.pending_manual_mode = MODE_JOINT
        self.remote_enabled_on_start = bool(
            self.get_parameter("remote_enabled_on_start").value
        )
        self.startup_switch_attempted = False
        self.controller_list_future = None
        self.next_controller_poll = 0.0

        self.joint_positions: dict[str, float] = {}
        self.last_joint_state = 0.0
        self.startup_pose: Optional[list[float]] = None
        self.command_positions: Optional[list[float]] = None
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        self.last_tick = time.monotonic()

        self.pose_target: Optional[list[float]] = None
        self.pose_operation: Optional[str] = None
        self.pose_target_until = 0.0
        self.post_pose_mode = MODE_JOINT
        self.ready_pending_on_enable = False
        self.ready_pending_mode = MODE_JOINT

        self.ik = None
        self.ik_target_pos: Optional[np.ndarray] = None
        self.ik_target_rotation: Optional[np.ndarray] = None
        self.cylindrical_theta_hint: Optional[float] = None
        self.ik_collision_blocked = False

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

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("forward_command_topic").value),
            10,
        )
        self.operation_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("operation_mode_state_topic").value),
            state_qos,
        )
        self.remote_state_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("remote_enabled_state_topic").value),
            state_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operation_mode_topic").value),
            self._on_operation_mode,
            mode_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("control_cmd_topic").value),
            self._on_control_cmd,
            command_qos,
        )
        self.switch_client = self.create_client(
            SwitchController,
            str(self.get_parameter("switch_controller_service").value),
        )
        self.list_client = self.create_client(
            ListControllers,
            str(self.get_parameter("list_controllers_service").value),
        )

        self._initialize_ik()
        self.create_timer(self.nominal_dt, self._tick)
        self._publish_state()
        self.get_logger().info(
            "controller ready: /om6dof/operation_mode + /om6dof/control_cmd "
            "-> forward_position_controller"
        )

    def _reported_mode_locked(self) -> str:
        if self.pose_operation is not None:
            return self.pose_operation
        return self.motion_mode if self.remote_enabled else MODE_AUTONOMOUS

    def _publish_state(self) -> None:
        self.operation_state_pub.publish(
            String(data=self._reported_mode_locked())
        )
        self.remote_state_pub.publish(Bool(data=bool(self.remote_enabled)))

    def _initialize_ik(self) -> None:
        if not bool(self.get_parameter("ik_enabled").value):
            self.get_logger().warn(
                "IK disabled; only JOINT operation mode is available"
            )
            return
        try:
            from .ik_solver import IKSolver

            self.ik = IKSolver(
                base_link=str(self.get_parameter("ik_base_link").value),
                tip_link=str(self.get_parameter("ik_tip_link").value),
                urdf_pkg=str(self.get_parameter("ik_urdf_pkg").value),
                damping=float(self.get_parameter("ik_damping").value),
            )
            self.get_logger().info(
                f"IK ready: {self.ik.base_link} -> {self.ik.tip_link}"
            )
        except Exception as exc:
            self.ik = None
            self.get_logger().error(
                f"IK initialization failed; coordinate modes disabled: {exc}"
            )

    def _joint_vector_locked(self) -> Optional[list[float]]:
        if len(self.joint_positions) != 6:
            return None
        return [self.joint_positions[name] for name in self.joint_names]

    def _joint_state_fresh_locked(self, now: float) -> bool:
        return bool(self.last_joint_state) and (
            now - self.last_joint_state <= self.joint_state_timeout
        )

    def _on_joint_state(self, msg: JointState) -> None:
        indices = {name: index for index, name in enumerate(msg.name)}
        positions: dict[str, float] = {}
        for name in self.joint_names:
            index = indices.get(name)
            if index is None or index >= len(msg.position):
                continue
            value = float(msg.position[index])
            if math.isfinite(value):
                positions[name] = value
        if len(positions) != 6:
            return
        with self.lock:
            self.joint_positions = positions
            self.last_joint_state = time.monotonic()
            vector = [positions[name] for name in self.joint_names]
            if self.startup_pose is None:
                self.startup_pose = list(vector)

    def _clear_stream_command_locked(self) -> None:
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0

    def _seed_ik_anchor_locked(
        self, joint_positions: Optional[Sequence[float]] = None
    ) -> bool:
        if self.ik is None:
            return False
        values = joint_positions
        if values is None:
            values = self._joint_vector_locked()
        if values is None or len(values) != 6:
            return False
        q = np.asarray(values, dtype=float)
        try:
            position, rotation = self.ik.fk_pose(q)
        except Exception as exc:
            self.get_logger().error(
                f"failed to seed coordinate target: {exc}",
                throttle_duration_sec=2.0,
            )
            return False
        self.ik_target_pos = np.asarray(position, dtype=float).copy()
        self.ik_target_rotation = np.asarray(rotation, dtype=float).copy()
        try:
            score = float(self.ik.manipulability(q))
        except Exception:
            score = math.inf
        if (
            math.isfinite(score)
            and score < self.ik_manipulability_warning_threshold
        ):
            self.get_logger().warn(
                f"coordinate control is near a singular pose "
                f"(manipulability={score:.3e}); request READY first",
                throttle_duration_sec=5.0,
            )
        dx = float(self.ik_target_pos[0] - self.cylindrical_origin_xy[0])
        dy = float(self.ik_target_pos[1] - self.cylindrical_origin_xy[1])
        radius = math.hypot(dx, dy)
        self.cylindrical_theta_hint = (
            math.atan2(dy, dx) if radius >= self.cylindrical_min_radius
            else float(values[0])
        )
        self.ik_collision_blocked = False
        return True

    def _set_motion_mode_locked(self, mode: str, source: str) -> bool:
        if mode not in MOTION_MODES:
            return False
        if mode != MODE_JOINT and self.ik is None:
            self.get_logger().warn(
                f"{source} request rejected: IK is unavailable"
            )
            return False
        if mode == self.motion_mode and self.pose_target is None:
            self._publish_state()
            return True
        self.motion_mode = mode
        self.pose_target = None
        self.pose_operation = None
        self._clear_stream_command_locked()
        self.ik_target_pos = None
        self.ik_target_rotation = None
        if mode != MODE_JOINT:
            self._seed_ik_anchor_locked(
                self.command_positions or self._joint_vector_locked()
            )
        self._publish_state()
        self.get_logger().info(f"operation mode -> {mode} ({source})")
        return True

    def _schedule_pose_locked(
        self,
        operation: str,
        target: Sequence[float],
        post_mode: str = MODE_JOINT,
    ) -> bool:
        now = time.monotonic()
        if not self.remote_enabled or not self.remote_controller_active:
            self.get_logger().warn(
                f"{operation} rejected: remote controller is not active"
            )
            return False
        if not self._joint_state_fresh_locked(now):
            self.get_logger().warn(
                f"{operation} rejected: /joint_states is stale"
            )
            return False
        vector = self._joint_vector_locked()
        if vector is None:
            return False
        self.command_positions = clamp_positions(
            vector, self.joint_lower, self.joint_upper
        )
        self.motion_mode = MODE_JOINT
        self._clear_stream_command_locked()
        self.ik_target_pos = None
        self.ik_target_rotation = None
        self.pose_target = clamp_positions(
            target, self.joint_lower, self.joint_upper
        )
        self.pose_operation = operation
        self.pose_target_until = now + self.pose_target_timeout
        self.post_pose_mode = post_mode
        self._publish_state()
        self.get_logger().info(f"{operation} target -> {self.pose_target}")
        return True

    def _schedule_ready_after_enable_locked(self, post_mode: str) -> bool:
        now = time.monotonic()
        if not self._joint_state_fresh_locked(now):
            self.command_positions = None
            self.ready_pending_on_enable = True
            self.ready_pending_mode = post_mode
            self.get_logger().warn(
                "remote active; READY waits for fresh /joint_states",
                throttle_duration_sec=2.0,
            )
            return False
        self.ready_pending_on_enable = False
        self.ready_pending_mode = MODE_JOINT
        return self._schedule_pose_locked(MODE_READY, self.ready_pose, post_mode)

    def _on_operation_mode(self, msg: String) -> None:
        try:
            mode = normalize_operation_mode(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f"operation mode rejected: {exc}")
            return
        with self.lock:
            if mode == MODE_AUTONOMOUS:
                self._request_controller_mode_locked(False, "operation_mode")
                return
            if mode == MODE_JOINT:
                if self.remote_enabled and self.remote_controller_active:
                    self._set_motion_mode_locked(MODE_JOINT, "operation_mode")
                else:
                    self.pending_manual_mode = MODE_JOINT
                    self._request_controller_mode_locked(True, "operation_mode")
                return
            if mode in (MODE_CARTESIAN, MODE_CYLINDRICAL):
                if not self.remote_enabled or not self.remote_controller_active:
                    self.get_logger().warn(
                        f"{mode} rejected: request JOINT first to take remote "
                        "ownership and enter READY"
                    )
                    return
                self._set_motion_mode_locked(mode, "operation_mode")
                return
            if mode == MODE_READY:
                if self.remote_enabled and self.remote_controller_active:
                    self._schedule_pose_locked(
                        MODE_READY, self.ready_pose, MODE_JOINT
                    )
                else:
                    self.pending_manual_mode = MODE_JOINT
                    self._request_controller_mode_locked(True, "READY")
                return
            if mode == MODE_STARTUP:
                if self.startup_pose is None:
                    self.get_logger().warn(
                        "STARTUP rejected: startup pose has not been captured"
                    )
                    return
                self._schedule_pose_locked(
                    MODE_STARTUP, self.startup_pose, MODE_JOINT
                )

    def _on_control_cmd(self, msg: Float64MultiArray) -> None:
        try:
            values = validated_control_command(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f"control_cmd rejected: {exc}")
            return
        with self.lock:
            if (
                not self.remote_enabled
                or not self.remote_controller_active
                or self.switch_in_progress
            ):
                self.get_logger().warn(
                    "control_cmd ignored: remote ownership is inactive",
                    throttle_duration_sec=2.0,
                )
                return
            if self.pose_target is not None:
                if not np.any(np.abs(values) > 1.0e-9):
                    return
                feedback = self._joint_vector_locked()
                if feedback is not None:
                    self.command_positions = clamp_positions(
                        feedback, self.joint_lower, self.joint_upper
                    )
                operation = self.pose_operation or "pose"
                self.pose_target = None
                self.pose_operation = None
                self.motion_mode = MODE_JOINT
                self.ik_target_pos = None
                self.ik_target_rotation = None
                self._publish_state()
                self.get_logger().info(
                    f"{operation} interrupted by a non-zero control_cmd"
                )
            self.control_velocity = values
            self.last_control_cmd = time.monotonic()

    def _request_controller_mode_locked(
        self, enable_remote: bool, source: str
    ) -> None:
        if self.switch_in_progress:
            self.get_logger().warn(
                f"{source} request ignored: controller switch is in progress"
            )
            return
        if enable_remote == self.remote_enabled:
            self._publish_state()
            return
        now = time.monotonic()
        if enable_remote:
            if not self._joint_state_fresh_locked(now):
                self.get_logger().warn(
                    f"{source} remote enable rejected: /joint_states is stale"
                )
                return
            vector = self._joint_vector_locked()
            if vector is None:
                self.get_logger().warn(
                    f"{source} remote enable rejected: incomplete joint state"
                )
                return
            self.command_positions = clamp_positions(
                vector, self.joint_lower, self.joint_upper
            )
        if not self.switch_client.service_is_ready():
            self.get_logger().warn(
                f"{source} request rejected: switch_controller unavailable"
            )
            return

        request = SwitchController.Request()
        if enable_remote:
            request.activate_controllers = [self.remote_controller]
            request.deactivate_controllers = [self.arm_controller]
        else:
            request.activate_controllers = [self.arm_controller]
            request.deactivate_controllers = [self.remote_controller]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        timeout = max(
            0.0, float(self.get_parameter("switch_timeout_seconds").value)
        )
        request.timeout.sec = int(timeout)
        request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        self.switch_in_progress = True
        self.switch_target = enable_remote
        future = self.switch_client.call_async(request)
        future.add_done_callback(
            lambda result, target=enable_remote, origin=source:
            self._on_switch_done(result, target, origin)
        )
        self.get_logger().info(
            f"{source}: switching to "
            f"{'REMOTE' if enable_remote else 'AUTONOMOUS'}"
        )

    def _on_switch_done(self, future, enable_remote: bool, source: str) -> None:
        try:
            response = future.result()
            success = bool(response and response.ok)
        except Exception as exc:
            success = False
            self.get_logger().error(f"controller switch failed: {exc}")

        command = None
        with self.lock:
            self.switch_in_progress = False
            self.switch_target = None
            if not success:
                self.get_logger().error(
                    f"{source}: switch to "
                    f"{'REMOTE' if enable_remote else 'AUTONOMOUS'} rejected"
                )
                return
            self.remote_enabled = enable_remote
            self.remote_controller_active = enable_remote
            self.arm_controller_active = not enable_remote
            self._clear_stream_command_locked()
            if enable_remote:
                self.motion_mode = MODE_JOINT
                post_mode = self.pending_manual_mode
                self.pending_manual_mode = MODE_JOINT
                scheduled = self._schedule_ready_after_enable_locked(post_mode)
                if scheduled and self.command_positions is not None:
                    command = list(self.command_positions)
            else:
                self.motion_mode = MODE_AUTONOMOUS
                self.pose_target = None
                self.pose_operation = None
                self.ready_pending_on_enable = False
                self.command_positions = None
                self.ik_target_pos = None
                self.ik_target_rotation = None
            self._publish_state()

        if command is not None:
            self.command_pub.publish(Float64MultiArray(data=command))
        self.get_logger().info(
            f"controller ownership -> "
            f"{'REMOTE' if enable_remote else 'AUTONOMOUS'}"
        )

    def _poll_controllers(self, now: float) -> None:
        if (
            self.controller_list_future is not None
            or now < self.next_controller_poll
            or not self.list_client.service_is_ready()
        ):
            return
        self.next_controller_poll = now + 1.0
        self.controller_list_future = self.list_client.call_async(
            ListControllers.Request()
        )
        self.controller_list_future.add_done_callback(self._on_controller_list)

    def _on_controller_list(self, future) -> None:
        self.controller_list_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"list_controllers failed: {exc}")
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        }
        arm_active = states.get(self.arm_controller) == "active"
        remote_active = states.get(self.remote_controller) == "active"
        command = None
        with self.lock:
            self.arm_controller_active = arm_active
            self.remote_controller_active = remote_active
            if self.switch_in_progress:
                return
            actual_remote = remote_active and not arm_active
            if actual_remote == self.remote_enabled:
                return
            self.remote_enabled = actual_remote
            self._clear_stream_command_locked()
            if actual_remote:
                self.motion_mode = MODE_JOINT
                if self._schedule_ready_after_enable_locked(MODE_JOINT):
                    command = list(self.command_positions or [])
            else:
                self.motion_mode = MODE_AUTONOMOUS
                self.pose_target = None
                self.pose_operation = None
                self.ready_pending_on_enable = False
                self.command_positions = None
                self.ik_target_pos = None
                self.ik_target_rotation = None
            self._publish_state()
            self.get_logger().warn(
                "controller state changed externally; internal state reconciled"
            )
        if command:
            self.command_pub.publish(Float64MultiArray(data=command))

    def _coordinate_step_locked(
        self,
        feedback_positions: Sequence[float],
        coordinate_velocity: Sequence[float],
        dt: float,
    ) -> list[float]:
        if self.ik is None or self.command_positions is None or dt <= 0.0:
            return list(self.command_positions or feedback_positions)
        q_feedback = np.asarray(feedback_positions, dtype=float)
        q_command = np.asarray(self.command_positions, dtype=float)
        velocity = np.asarray(coordinate_velocity, dtype=float)
        if (
            q_feedback.shape != (6,)
            or q_command.shape != (6,)
            or velocity.shape != (6,)
            or not np.all(np.isfinite(q_feedback))
            or not np.all(np.isfinite(q_command))
            or not np.all(np.isfinite(velocity))
        ):
            return list(self.command_positions)

        following_error = float(np.max(np.abs(q_command - q_feedback)))
        if following_error > self.ik_max_joint_following_error:
            self.get_logger().warn(
                f"coordinate command stopped: joint following error "
                f"{following_error:.3f} rad",
                throttle_duration_sec=2.0,
            )
            self.command_positions = clamp_positions(
                q_feedback, self.joint_lower, self.joint_upper
            )
            self._seed_ik_anchor_locked(q_feedback)
            return list(self.command_positions)

        if self.ik_target_pos is None or self.ik_target_rotation is None:
            if not self._seed_ik_anchor_locked(q_command):
                return list(self.command_positions)
        try:
            command_position, command_rotation = self.ik.fk_pose(q_command)
            feedback_position, _ = self.ik.fk_pose(q_feedback)
            command_position = np.asarray(command_position, dtype=float)
            command_rotation = np.asarray(command_rotation, dtype=float)
            feedback_position = np.asarray(feedback_position, dtype=float)

            if self.motion_mode == MODE_CARTESIAN:
                linear_feedforward = limit_norm(
                    velocity[:3], self.max_cartesian_linear_velocity
                )
                self.ik_target_pos = (
                    self.ik_target_pos + linear_feedforward * dt
                )
            elif self.motion_mode == MODE_CYLINDRICAL:
                radial_velocity = max(
                    -self.max_cartesian_linear_velocity,
                    min(self.max_cartesian_linear_velocity, float(velocity[0])),
                )
                theta_velocity = max(
                    -self.max_cylindrical_theta_velocity,
                    min(
                        self.max_cylindrical_theta_velocity,
                        float(velocity[1]),
                    ),
                )
                vertical_velocity = max(
                    -self.max_cartesian_linear_velocity,
                    min(self.max_cartesian_linear_velocity, float(velocity[2])),
                )
                previous_target = self.ik_target_pos.copy()
                self.ik_target_pos, self.cylindrical_theta_hint = (
                    integrate_cylindrical_position(
                        self.ik_target_pos,
                        radial_velocity,
                        theta_velocity,
                        vertical_velocity,
                        dt,
                        self.cylindrical_origin_xy,
                        self.cylindrical_min_radius,
                        self.cylindrical_theta_hint,
                    )
                )
                linear_feedforward = (
                    self.ik_target_pos - previous_target
                ) / dt
                linear_feedforward = limit_norm(
                    linear_feedforward, self.max_cartesian_linear_velocity
                )
            else:
                return list(self.command_positions)

            angular_feedforward = limit_norm(
                velocity[3:], self.max_cartesian_angular_velocity
            )
            if (
                self.ik_tool_frame_rotation
                and float(np.linalg.norm(angular_feedforward)) > 1e-9
            ):
                angular_feedforward = self.ik.ee_to_base_angular(
                    q_command, angular_feedforward
                )
            if float(np.linalg.norm(angular_feedforward)) > 1e-9:
                self.ik_target_rotation = rotation_from_rotvec(
                    angular_feedforward * dt
                ) @ self.ik_target_rotation

            lead = self.ik_target_pos - feedback_position
            lead_distance = float(np.linalg.norm(lead))
            if lead_distance > self.ik_max_target_lead:
                self.ik_target_pos = feedback_position + lead * (
                    self.ik_max_target_lead / lead_distance
                )
                if self.motion_mode == MODE_CYLINDRICAL:
                    dx = float(
                        self.ik_target_pos[0] - self.cylindrical_origin_xy[0]
                    )
                    dy = float(
                        self.ik_target_pos[1] - self.cylindrical_origin_xy[1]
                    )
                    if math.hypot(dx, dy) > 1e-9:
                        self.cylindrical_theta_hint = math.atan2(dy, dx)

            position_error = self.ik_target_pos - command_position
            orientation_error = rotation_error(
                self.ik_target_rotation, command_rotation
            )
            orientation_norm = float(np.linalg.norm(orientation_error))
            if orientation_norm > 0.35:
                excess = orientation_error * (
                    (orientation_norm - 0.35) / orientation_norm
                )
                self.ik_target_rotation = (
                    rotation_from_rotvec(-excess) @ self.ik_target_rotation
                )
                orientation_error *= 0.35 / orientation_norm

            linear_command = (
                linear_feedforward + self.ik_position_gain * position_error
            )
            angular_command = (
                angular_feedforward + self.ik_rotation_gain * orientation_error
            )
            if float(np.linalg.norm(np.concatenate([
                linear_command, angular_command
            ]))) < 1e-8:
                return list(self.command_positions)

            joint_velocity = self.ik.velocity_ik_priority(
                q_command, linear_command, angular_command
            )
            if not np.all(np.isfinite(joint_velocity)):
                raise ValueError("IK produced non-finite joint velocity")
            peak = float(np.max(np.abs(joint_velocity)))
            if peak > self.max_joint_velocity:
                joint_velocity *= self.max_joint_velocity / peak
            raw_candidate = q_command + joint_velocity * dt
            candidate = np.asarray(
                clamp_positions(
                    raw_candidate, self.joint_lower, self.joint_upper
                ),
                dtype=float,
            )
            hit_limit = bool(
                np.any(np.abs(candidate - raw_candidate) > 1e-9)
            )
            if self.ik_self_collision and self.ik.self_collides(
                candidate, self.ik_collision_radius
            ):
                if not self.ik_collision_blocked:
                    self.get_logger().warn(
                        "coordinate motion blocked by self-collision boundary"
                    )
                self._seed_ik_anchor_locked(q_command)
                self.ik_collision_blocked = True
                return list(self.command_positions)
            self.ik_collision_blocked = False
            if hit_limit:
                self._seed_ik_anchor_locked(candidate)
            return candidate.tolist()
        except Exception as exc:
            self.get_logger().error(
                f"coordinate IK step failed; holding: {exc}",
                throttle_duration_sec=2.0,
            )
            self._seed_ik_anchor_locked(q_command)
            return list(self.command_positions)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.1, now - self.last_tick))
        self.last_tick = now
        self._poll_controllers(now)

        if (
            self.remote_enabled_on_start
            and not self.startup_switch_attempted
            and self.arm_controller_active
            and self._joint_state_fresh_locked(now)
            and self.switch_client.service_is_ready()
        ):
            with self.lock:
                self.startup_switch_attempted = True
                self.pending_manual_mode = MODE_JOINT
                self._request_controller_mode_locked(True, "startup")

        command = None
        reached_operation = None
        timeout_operation = None
        with self.lock:
            if (
                not self.remote_enabled
                or not self.remote_controller_active
                or self.switch_in_progress
            ):
                return
            if not self._joint_state_fresh_locked(now):
                self.command_positions = None
                self.ik_target_pos = None
                self.ik_target_rotation = None
                self.get_logger().warn(
                    "/joint_states stale; holding the last hardware command",
                    throttle_duration_sec=2.0,
                )
                return
            if self.ready_pending_on_enable:
                if not self._schedule_ready_after_enable_locked(
                    self.ready_pending_mode
                ):
                    return
                dt = 0.0
            feedback = self._joint_vector_locked()
            if feedback is None:
                return
            if self.command_positions is None:
                self.command_positions = clamp_positions(
                    feedback, self.joint_lower, self.joint_upper
                )

            if self.pose_target is not None:
                if now > self.pose_target_until:
                    timeout_operation = self.pose_operation or "pose"
                    self.pose_target = None
                    self.pose_operation = None
                    self.motion_mode = MODE_JOINT
                    self.command_positions = clamp_positions(
                        feedback, self.joint_lower, self.joint_upper
                    )
                    self._publish_state()
                else:
                    self.command_positions = step_toward(
                        self.command_positions,
                        self.pose_target,
                        self.pose_target_velocity * dt,
                    )
                    if (
                        all(
                            abs(target - actual) <= self.pose_target_tolerance
                            for target, actual in zip(
                                self.pose_target, feedback
                            )
                        )
                        and all(
                            abs(target - requested)
                            <= self.pose_target_tolerance
                            for target, requested in zip(
                                self.pose_target, self.command_positions
                            )
                        )
                    ):
                        reached_operation = self.pose_operation or "pose"
                        next_mode = self.post_pose_mode
                        self.pose_target = None
                        self.pose_operation = None
                        self.motion_mode = next_mode
                        self._clear_stream_command_locked()
                        if next_mode != MODE_JOINT:
                            self._seed_ik_anchor_locked(feedback)
                        self._publish_state()
            else:
                stream_fresh = bool(
                    self.last_control_cmd
                    and now - self.last_control_cmd
                    <= self.control_cmd_timeout
                )
                if not stream_fresh:
                    self.command_positions = clamp_positions(
                        feedback, self.joint_lower, self.joint_upper
                    )
                    if self.motion_mode != MODE_JOINT:
                        self._seed_ik_anchor_locked(feedback)
                elif self.motion_mode == MODE_JOINT:
                    velocity = np.clip(
                        self.control_velocity,
                        -self.max_joint_velocity,
                        self.max_joint_velocity,
                    )
                    self.command_positions = [
                        self.command_positions[index]
                        + float(velocity[index]) * dt
                        for index in range(6)
                    ]
                elif self.motion_mode in (
                    MODE_CARTESIAN, MODE_CYLINDRICAL
                ):
                    velocity = self.control_velocity.copy()
                    if self.motion_mode == MODE_CARTESIAN:
                        velocity[:3] = limit_norm(
                            velocity[:3],
                            self.max_cartesian_linear_velocity,
                        )
                    else:
                        velocity[0] = max(
                            -self.max_cartesian_linear_velocity,
                            min(
                                self.max_cartesian_linear_velocity,
                                velocity[0],
                            ),
                        )
                        velocity[1] = max(
                            -self.max_cylindrical_theta_velocity,
                            min(
                                self.max_cylindrical_theta_velocity,
                                velocity[1],
                            ),
                        )
                        velocity[2] = max(
                            -self.max_cartesian_linear_velocity,
                            min(
                                self.max_cartesian_linear_velocity,
                                velocity[2],
                            ),
                        )
                    velocity[3:] = limit_norm(
                        velocity[3:], self.max_cartesian_angular_velocity
                    )
                    self.command_positions = self._coordinate_step_locked(
                        feedback, velocity, dt
                    )

            self.command_positions = clamp_positions(
                self.command_positions, self.joint_lower, self.joint_upper
            )
            command = list(self.command_positions)

        if command is not None:
            self.command_pub.publish(Float64MultiArray(data=command))
        if timeout_operation:
            self.get_logger().warn(f"{timeout_operation} target timed out")
        if reached_operation:
            self.get_logger().info(f"{reached_operation} target reached")

    def destroy_node(self):
        try:
            if (
                rclpy.ok()
                and self.remote_enabled
                and self.switch_client.service_is_ready()
            ):
                request = SwitchController.Request()
                request.activate_controllers = [self.arm_controller]
                request.deactivate_controllers = [self.remote_controller]
                request.strictness = SwitchController.Request.BEST_EFFORT
                request.activate_asap = True
                future = self.switch_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OM6DOFController()
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
