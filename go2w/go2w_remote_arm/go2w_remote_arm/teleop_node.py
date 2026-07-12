"""Go2W wireless remote → 6-DOF OpenManipulator Chain direct joint control.

Owns /dev/ttyUSB0 via dynamixel_sdk (no ros2_control). 7 Dynamixels:

  joint1  XM430-W350   ID 31
  joint2  XM430-W350   ID 32
  joint3  XM430-W350   ID 33
  joint4  XM430-W210   ID 24
  joint5  XM430-W350   ID 35
  joint6  XM430-W210   ID 26
  gripper XM430-W350   ID 37

(All XM430s share Protocol 2.0 and the same control table; gear ratio differs
but that affects only the speed-per-unit profile, not addressing.)

Default button mapping (configurable via ROS parameters)
-------------------------------------------------------
  D-pad Left  (15) / Right (13)   → joint1  − / +
  D-pad Down  (14) / Up    (12)   → joint2  − / +
  A           (8)  / Y     (11)   → joint3  − / +
  X           (10) / B     (9)    → joint4  − / +
  F1          (6)  / F3    (7)    → joint5  − / +
  Start       (2)  / Select(3)    → joint6  − / +
  L1          (1)              → gripper OPEN  (edge-triggered, absolute target)
  L2          (5)              → gripper CLOSE (edge-triggered, absolute target)

A button held → that joint moves at ±`joint_velocity` rad/s. Released → it
holds the last commanded position. Gentle torque-on at startup: pre-loads
goal = present, slow profile, torque on, then ramp profile.
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from unitree_go.msg import WirelessController
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
)


# ---------- Dynamixel XM430 / XL430 control table (Protocol 2.0) ----------
ADDR_DRIVE_MODE = 10            # 1 byte (bit 2: 0=Velocity-based, 1=Time-based profile)
ADDR_OPERATING_MODE = 11        # 1 byte
ADDR_TORQUE_ENABLE = 64         # 1 byte
ADDR_HARDWARE_ERROR = 70        # 1 byte
ADDR_GOAL_CURRENT = 102         # 2 bytes, signed (current limit in mode 5)
ADDR_PROFILE_ACCEL = 108        # 4 bytes (velocity-based: rev/min²·1/214.577; time-based: ms)
ADDR_PROFILE_VELOCITY = 112     # 4 bytes (velocity-based: rev/min·1/0.229;   time-based: ms)
ADDR_GOAL_POSITION = 116        # 4 bytes
ADDR_PRESENT_CURRENT = 126      # 2 bytes, signed (grasp detection)
ADDR_PRESENT_POSITION = 132     # 4 bytes

# XM430-W350 current unit: 2.69 mA per raw tick.
CURRENT_UNIT_MA = 2.69

DRIVE_MODE_VELOCITY_BASED = 0x00
DRIVE_MODE_TIME_BASED = 0x04
LEN_GOAL = 4
LEN_PRESENT = 4
LEN_PROFILE = 4
# One contiguous block: Present Current(126,2) + Velocity(128,4) + Position(132,4).
# Sync-reading all 10 bytes gets each motor's position AND the gripper's current
# in a single transaction.
PRESENT_BLOCK_START = ADDR_PRESENT_CURRENT
PRESENT_BLOCK_LEN = 10

OP_MODE_POSITION = 3
OP_MODE_CURRENT_POSITION = 5    # position control with a settable current limit
PROTOCOL_VERSION = 2.0
TICKS_PER_REV = 4096
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV


def rad_to_ticks(r: float) -> int:
    return int(round(2048 + r / TICK_RAD))


def ticks_to_rad(t: int) -> float:
    return (t - 2048) * TICK_RAD


def _u32(v: int) -> bytes:
    v &= 0xFFFFFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])


def _s32(b: int) -> int:
    return b - (1 << 32) if b >= (1 << 31) else b


def _s16(b: int) -> int:
    return b - (1 << 16) if b >= (1 << 15) else b


# Button bit positions on the Go2W wireless_remote.
# F3 (bit 7) is owned by the launcher node (starts/stops this teleop process).
# F1 (bit 6) is intercepted here for the park-and-release routine.
# Select (bit 3) toggles between JOINT mode (per-joint buttons) and IK mode
# (button-driven EE twist solved via PyKDL).
BTN_F1 = 1 << 6
BTN_SELECT = 1 << 3
BTN_R1 = 1 << 0
BTN_R2 = 1 << 4
BTN_X = 1 << 10
BTN_B = 1 << 9
BTN_Y = 1 << 11
BTN_A = 1 << 8
BTN_UP = 1 << 12
BTN_DOWN = 1 << 14
BTN_LEFT = 1 << 15
BTN_RIGHT = 1 << 13

MODE_JOINT = "JOINT"
MODE_IK = "IK"


def _rot_from_rotvec(rotvec) -> "object":
    """Rodrigues' formula: 3-vector (axis·angle) → 3×3 rotation matrix.
    Local helper so the module doesn't hard-depend on scipy."""
    import numpy as np
    v = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(v))
    if theta < 1e-9:
        return np.eye(3)
    k = v / theta
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _quat_to_matrix(x: float, y: float, z: float, w: float):
    import numpy as np
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(R):
    """3×3 rotation → (x, y, z, w)."""
    import numpy as np
    R = np.asarray(R, dtype=float)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_remote_arm")

        # ---- parameters ----
        # sim_mode: no serial port / no Dynamixels. The node still subscribes
        # to /wirelesscontroller and runs the exact same JOINT/IK logic, but
        # instead of writing to motors it just publishes goal_rad on
        # /joint_states so RViz (robot_state_publisher) visualizes the motion.
        # Lets you verify the control feel against the URDF before touching HW.
        self.declare_parameter("sim_mode", False)
        self.declare_parameter("device", "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4",
             "joint5", "joint6", "gripper"],
        )
        self.declare_parameter("motor_ids", [31, 32, 33, 24, 35, 26, 37])
        self.declare_parameter("joint_velocity", 0.5)  # rad/s while held
        self.declare_parameter("publish_rate_hz", 50.0)
        # button_pairs: 12 ints = (j1_dec, j1_inc, j2_dec, j2_inc, ... j6_dec, j6_inc)
        # bit indices per Go2W remote: 0=R1 1=L1 2=Start 3=Select 4=R2 5=L2
        #   6=F1 7=F3 8=A 9=B 10=X 11=Y 12=Up 13=Right 14=Down 15=Left
        self.declare_parameter(
            "button_pairs",
            [15, 13,  # j1: Left / Right
             14, 12,  # j2: Down / Up
              8, 11,  # j3: A / Y
             10,  9,  # j4: X / B
              6,  7,  # j5: F1 / F3
              2,  3,  # j6: Start / Select
             ],
        )
        # per-arm-joint sign (multiplied with the increment)
        self.declare_parameter("joint_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        # per-arm-joint analog axis index: -1 = use button_pairs, otherwise
        # 0=lx, 1=ly, 2=rx, 3=ry. Axis value [-1..1] is multiplied with
        # joint_velocity * dt * sign each tick (continuous motion).
        self.declare_parameter("joint_axes", [-1, -1, -1, -1, -1, -1])
        self.declare_parameter("joint_axis_deadzone", 0.08)
        # per-arm-joint soft min/max position (radians)
        self.declare_parameter("joint_min", [-math.pi] * 6)
        self.declare_parameter("joint_max", [+math.pi] * 6)

        # gripper (edge-triggered open/close targets)
        self.declare_parameter("gripper_btn_open", 1)   # L1
        self.declare_parameter("gripper_btn_close", 5)  # L2
        self.declare_parameter("gripper_open_target", -1.0)
        self.declare_parameter("gripper_close_target", 0.0)
        self.declare_parameter("gripper_min", -math.pi)
        self.declare_parameter("gripper_max", +math.pi)

        # grasp-with-detection: when True the CLOSE button (L2) closes the jaw
        # slowly toward gripper_close_target while watching Present Current, and
        # stops/holds the instant the jaw presses on an object. When False L2
        # snaps straight to the close target (original behaviour). Runs as a
        # per-tick state machine so the remote stays responsive during a grasp.
        self.declare_parameter("grasp_on_close", True)
        # Gripper runs in Current-Based Position mode (op mode 5): it always
        # pushes toward the commanded target but never exceeds this current, so
        # holding an object indefinitely is safe (no overload) and letting go is
        # detectable (jaws collapse to the close limit). State is published on
        # gripper_state_topic and logged on every transition.
        self.declare_parameter("gripper_current_limit_ma", 120.0)  # squeeze force cap (gentle)
        self.declare_parameter("grasp_current_threshold", 70.0)    # |mA| ⇒ HOLDING (keep < limit)
        self.declare_parameter("grasp_consecutive", 3)   # ticks over threshold to latch HOLDING
        self.declare_parameter("grip_close_tol", 0.06)   # rad from close_target ⇒ fully CLOSED
        self.declare_parameter("grip_open_tol", 0.12)    # rad from open_target ⇒ OPEN
        self.declare_parameter("gripper_profile_velocity", 40)  # slow, gentle close
        self.declare_parameter("gripper_state_topic", "/go2w_remote_arm/gripper_state")

        # gentle startup profile
        self.declare_parameter("startup_slow_vel", 30)
        self.declare_parameter("startup_slow_acc", 10)
        self.declare_parameter("run_vel", 100)
        self.declare_parameter("run_acc", 30)
        # ---- smooth Cartesian streaming (Time-based Profile) ----
        # Velocity-based profiles make each joint finish its sub-move on its
        # own schedule (far joints lag) → bent Cartesian paths. Time-based
        # profiles make ALL joints arrive together every tick → coordinated,
        # smooth motion. stream_profile_ms should be ~3-5x the tick period so
        # consecutive goals blend into one continuous motion.
        self.declare_parameter("smooth_stream", True)
        self.declare_parameter("stream_profile_ms", 150)
        self.declare_parameter("stream_accel_ms", 50)
        # max joint speed in IK (cmd_vel) mode, rad/s
        self.declare_parameter("ik_qdot_max", 1.2)
        # pose-anchor feedback gains: pin the EE at the commanded pose so a
        # pure rotation really pivots ABOUT the EoE (no translation leak from
        # the damped pseudoinverse / joint limits), and translation holds the
        # current orientation.
        self.declare_parameter("ik_pos_gain", 4.0)   # 1/s
        self.declare_parameter("ik_rot_gain", 3.0)   # 1/s
        # rotation inputs are about TOOL axes (roll = around the gripper's
        # own axis) instead of world axes
        self.declare_parameter("ik_tool_frame_rotation", True)
        self.declare_parameter("settle_seconds", 0.5)

        # ---- Cartesian-goal servo (arm-server mode for autonomous pick) ----
        # An external node publishes a tool-pose goal on /arm_cart_goal; the
        # tick loop drives the EoE there in a STRAIGHT line at a capped
        # Cartesian speed using the same PyKDL solver as the remote's IK jog.
        # Progress is reported on /arm_cart_status ("moving"/"reached"/
        # "unreachable"/"idle") and the live tool pose on /arm_cart_state.
        # A remote-stick input always overrides an active Cartesian goal
        # (deadman safety).
        self.declare_parameter("cart_enabled", True)
        self.declare_parameter("cart_goal_topic", "/arm_cart_goal")
        self.declare_parameter("cart_lin_speed", 0.05)   # m/s straight-line
        self.declare_parameter("cart_ang_speed", 0.6)    # rad/s
        self.declare_parameter("cart_pos_tol", 0.006)    # m "reached"
        self.declare_parameter("cart_ang_tol", 0.10)     # rad "reached"
        self.declare_parameter("cart_stall_iters", 40)   # ticks of no progress

        # startup centering via Time-based Profile (trapezoidal/triangular)
        self.declare_parameter("center_on_startup", True)
        self.declare_parameter("center_total_time_s", 4.0)
        self.declare_parameter("center_accel_time_s", 2.0)
        # Center target per motor in rad. Length 1 → broadcast to all; longer
        # vector is taken in order (joint1, joint2, ..., gripper).
        self.declare_parameter("center_position_rad", [0.0])

        # After centering, optionally move into a folded "ready" pose so IK
        # mode is usable immediately (q=0 vertical is a true singularity,
        # IK is locked there). Default matches the MoveIt `rest` group_state
        # (manipulability approx 6e-4, well away from 0).
        self.declare_parameter("fold_after_center", True)
        self.declare_parameter("fold_total_time_s", 3.0)
        self.declare_parameter("fold_accel_time_s", 1.5)
        # joint1..joint6 + gripper. Negative `gripper` keeps the gripper
        # at its current position (no motion). Override per-rig via launch.
        self.declare_parameter(
            "fold_position_rad",
            [0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0, float('nan')],
        )

        # After the F1 park sequence, disable torque on every motor whose ID
        # is NOT in this list. Default keeps joint1 (ID 31) and joint4 (ID 24)
        # torque ON to match the boot-time torque state set by the launcher.
        self.declare_parameter("release_torque_except_ids", [31, 24])

        # IK mode (toggled by Select). Velocities are applied each tick when a
        # button is held; sign flips switch direction without re-mapping bits.
        self.declare_parameter("ik_enabled", True)
        self.declare_parameter("ik_base_link", "world")
        self.declare_parameter("ik_tip_link", "end_effector_link")
        # canonical kinematics source = the open-source 6dof description
        # (same numbers as om_chain, but single-sourced with MoveIt's SRDF)
        self.declare_parameter("ik_urdf_pkg", "om6dof_description")
        self.declare_parameter("ik_lin_speed", 0.05)   # m/s while button held
        self.declare_parameter("ik_ang_speed", 0.5)    # rad/s while button held
        # Damped least-squares lambda. Lower = more responsive, but more
        # sensitive to singularities (e.g. arm fully vertical at q=0 right
        # after boot centering). 0.05 hits a good balance for this arm.
        self.declare_parameter("ik_damping", 0.05)
        self.declare_parameter("ik_axis_signs",
                                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # vx vy vz wx wy wz
        # Self-collision: reject IK solutions where the capsule model of the
        # links would intersect. The arm freezes at the boundary instead of
        # folding through itself. radius = link "thickness" / 2 (m).
        self.declare_parameter("ik_self_collision", True)
        self.declare_parameter("ik_collision_radius", 0.025)

        # Optional automatic pose commands, used by demo_june_2026. Commands
        # update the same goal vector teleop writes each tick, so remote control
        # still works normally after or during a demo.
        self.declare_parameter("enable_joint_command_subscriber", True)
        self.declare_parameter(
            "joint_command_topic", "/go2w_remote_arm/joint_command"
        )

        # ---- read parameters ----
        self.device = str(self.get_parameter("device").value)
        self.baud = int(self.get_parameter("baudrate").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.motor_ids: List[int] = [int(x) for x in self.get_parameter("motor_ids").value]
        if len(self.joint_names) != len(self.motor_ids):
            raise RuntimeError(
                f"joint_names ({len(self.joint_names)}) and motor_ids "
                f"({len(self.motor_ids)}) must have the same length"
            )
        self.n_total = len(self.motor_ids)
        self.n_arm = self.n_total - 1  # last entry is gripper

        self.joint_velocity = float(self.get_parameter("joint_velocity").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        bps = [int(x) for x in self.get_parameter("button_pairs").value]
        if len(bps) != 2 * self.n_arm:
            raise RuntimeError(
                f"button_pairs must have {2 * self.n_arm} entries for {self.n_arm} "
                f"arm joints, got {len(bps)}"
            )
        self.button_pairs = [(bps[2 * i], bps[2 * i + 1]) for i in range(self.n_arm)]
        self.joint_signs = [float(x) for x in self.get_parameter("joint_signs").value]
        ja = [int(x) for x in self.get_parameter("joint_axes").value]
        if len(ja) < self.n_arm:
            ja = ja + [-1] * (self.n_arm - len(ja))
        self.joint_axes = ja[: self.n_arm]
        self.axis_deadzone = float(self.get_parameter("joint_axis_deadzone").value)
        self.joint_min = [float(x) for x in self.get_parameter("joint_min").value]
        self.joint_max = [float(x) for x in self.get_parameter("joint_max").value]
        self.grip_btn_open = int(self.get_parameter("gripper_btn_open").value)
        self.grip_btn_close = int(self.get_parameter("gripper_btn_close").value)
        self.grip_open = float(self.get_parameter("gripper_open_target").value)
        self.grip_close = float(self.get_parameter("gripper_close_target").value)
        self.grip_min = float(self.get_parameter("gripper_min").value)
        self.grip_max = float(self.get_parameter("gripper_max").value)
        self.grasp_on_close = bool(self.get_parameter("grasp_on_close").value)
        self.grip_current_limit = float(self.get_parameter("gripper_current_limit_ma").value)
        self.grasp_threshold = float(self.get_parameter("grasp_current_threshold").value)
        self.grasp_consecutive = int(self.get_parameter("grasp_consecutive").value)
        self.grip_close_tol = float(self.get_parameter("grip_close_tol").value)
        self.grip_open_tol = float(self.get_parameter("grip_open_tol").value)
        self.grip_profile_vel = int(self.get_parameter("gripper_profile_velocity").value)
        self.grip_state_topic = str(self.get_parameter("gripper_state_topic").value)
        self.keep_torque_ids = [
            int(x) for x in self.get_parameter("release_torque_except_ids").value
        ]
        self.sim_mode = bool(self.get_parameter("sim_mode").value)

        # gripper state machine (see _update_gripper / _tick). MUST be set BEFORE
        # _init_hardware(), because _setup_gripper_current_mode() flips
        # _grip_mode5 True during init — initializing it afterwards would clobber
        # that back to False and silence the state publisher.
        #   action : last user command — "open" / "close" / None
        #   state  : OPEN / OPENING / CLOSING / HOLDING / CLOSED / DROPPED
        # DROPPED is sticky (an object was held then lost) until the next open.
        self._grip_action: Optional[str] = None
        self._grip_state = "?"
        self._grip_pos = 0.0
        self._grip_cur = 0.0
        self._grip_hold_count = 0   # debounce for latching HOLDING
        self._grip_pub_count = 0    # heartbeat publish divider
        self._grip_mode5 = False    # True once the gripper is in mode 5
        self.pub_grip = None        # created in _setup_ros_io

        if self.sim_mode:
            self._init_sim()
        else:
            self._init_hardware()

        # ---- snapshot: positions read BEFORE torque was enabled ----
        # F1 (park-and-release) will use this as the return target.
        self.initial_pose: List[float] = list(self._pretorque_pose)
        self._parking = False

        # state — goals are in joint-frame radians, indexed same as motor_ids
        self.goal_rad: List[float] = list(self._present_pose)
        self.lock = threading.Lock()
        self.keys = 0
        self.prev_keys = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry
        self.last_remote_t = 0.0

        self._setup_ros_io()

    # ------------------------------------------------------------------ #
    #  hardware init (real Dynamixels)                                    #
    # ------------------------------------------------------------------ #
    def _init_hardware(self) -> None:
        # ---- bus init ----
        self.port = PortHandler(self.device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise IOError(f"failed to open {self.device}")
        if not self.port.setBaudRate(self.baud):
            raise IOError(f"failed to set baud {self.baud}")

        # ---- discovery: ping each configured motor; drop non-responsive ones ----
        # Allows running with partial hardware (e.g. joint5/6 not yet plugged).
        self.get_logger().info(f"Pinging {self.motor_ids}...")
        live_arm_idx: List[int] = []
        gripper_live = False
        for orig_idx, mid in enumerate(self.motor_ids):
            is_gripper = (orig_idx == self.n_total - 1)
            ok = False
            for retry in range(2):
                _, rc, err = self.packet.ping(self.port, mid)
                if rc == COMM_SUCCESS and err == 0:
                    ok = True
                    break
            label = self.joint_names[orig_idx]
            if ok:
                self.get_logger().info(f"  ID {mid:3d} ({label}): OK")
                if is_gripper:
                    gripper_live = True
                else:
                    live_arm_idx.append(orig_idx)
            else:
                self.get_logger().warn(
                    f"  ID {mid:3d} ({label}): no response — skipping"
                )

        if not live_arm_idx and not gripper_live:
            raise RuntimeError(
                "no motors responded — check 12V power, U2D2 cable, daisy chain"
            )

        # filter per-joint arrays to live arm joints, then append gripper if live
        def _take(seq, idxs):
            return [seq[i] for i in idxs]

        new_motor_ids = _take(self.motor_ids, live_arm_idx)
        new_joint_names = _take(self.joint_names, live_arm_idx)
        new_joint_signs = _take(self.joint_signs, live_arm_idx)
        new_joint_axes = _take(self.joint_axes, live_arm_idx)
        new_joint_min = _take(self.joint_min, live_arm_idx)
        new_joint_max = _take(self.joint_max, live_arm_idx)
        new_button_pairs = _take(self.button_pairs, live_arm_idx)

        if gripper_live:
            new_motor_ids.append(self.motor_ids[-1])
            new_joint_names.append(self.joint_names[-1])

        self.motor_ids = new_motor_ids
        self.joint_names = new_joint_names
        self.joint_signs = new_joint_signs
        self.joint_axes = new_joint_axes
        self.joint_min = new_joint_min
        self.joint_max = new_joint_max
        self.button_pairs = new_button_pairs
        self.n_arm = len(live_arm_idx)
        self.n_total = len(self.motor_ids)
        self.gripper_live = gripper_live

        self.get_logger().info(
            f"Active: {self.n_arm} arm joint(s) + "
            f"{'1 gripper' if gripper_live else 'no gripper'}  "
            f"({self.motor_ids})"
        )

        # ---- now build sync read/write groups with the LIVE motor list ----
        self.sync_read = GroupSyncRead(
            self.port, self.packet, PRESENT_BLOCK_START, PRESENT_BLOCK_LEN
        )
        for mid in self.motor_ids:
            if not self.sync_read.addParam(mid):
                raise IOError(f"sync_read addParam failed for ID {mid}")

        self.sync_write_goal = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL
        )
        self.sync_write_vel = GroupSyncWrite(
            self.port, self.packet, ADDR_PROFILE_VELOCITY, LEN_PROFILE
        )
        self.sync_write_acc = GroupSyncWrite(
            self.port, self.packet, ADDR_PROFILE_ACCEL, LEN_PROFILE
        )

        # auto-clear stale Hardware Error flags via Reboot
        self._clear_hw_errors_via_reboot()

        # save run profile so centering can revert to it afterwards
        self._run_vel = int(self.get_parameter("run_vel").value)
        self._run_acc = int(self.get_parameter("run_acc").value)

        # gentle torque-on
        present = self._gentle_torque_on(
            int(self.get_parameter("startup_slow_vel").value),
            int(self.get_parameter("startup_slow_acc").value),
            self._run_vel,
            self._run_acc,
            float(self.get_parameter("settle_seconds").value),
        )

        self.get_logger().info(
            f"Initial pose snapshot (pre-torque): "
            f"{[round(v, 3) for v in present]}"
        )
        # True resting pose, read while torque was OFF. F1 (park-and-release)
        # returns HERE before dropping torque — NOT the folded ready pose,
        # otherwise the arm would sag from the raised fold pose when torque is
        # released. Captured before centering/fold overwrite `present`.
        self._pretorque_pose = list(present)

        # ---- optional startup centering (Time-based Trapezoidal Profile) ----
        if bool(self.get_parameter("center_on_startup").value):
            total_s = float(self.get_parameter("center_total_time_s").value)
            accel_s = float(self.get_parameter("center_accel_time_s").value)
            cp = [float(x) for x in self.get_parameter("center_position_rad").value]
            if len(cp) == 1:
                target = [cp[0]] * self.n_total
            else:
                target = (cp + [0.0] * self.n_total)[: self.n_total]
            self._center_all_servos(target, total_s, accel_s)
            present = target  # goal_rad will reflect the centered pose

        # ---- optional fold into a non-singular ready pose ----
        # Runs only after centering so the arm starts from a known state. The
        # fold target leaves IK well clear of the q=0 singularity so Select →
        # IK works without the user having to reposition first.
        if (bool(self.get_parameter("fold_after_center").value)
                and bool(self.get_parameter("center_on_startup").value)):
            import math as _math
            fold_total_s = float(self.get_parameter("fold_total_time_s").value)
            fold_accel_s = float(self.get_parameter("fold_accel_time_s").value)
            raw = [float(x) for x in self.get_parameter("fold_position_rad").value]
            # Pad / truncate to n_total. NaN entries → keep current value
            # (useful for `gripper: nan` = don't disturb the gripper).
            if len(raw) < self.n_total:
                raw = raw + [float('nan')] * (self.n_total - len(raw))
            fold_target = []
            for i in range(self.n_total):
                v = raw[i]
                fold_target.append(present[i] if _math.isnan(v) else v)
            self.get_logger().info(
                f"Folding to {[round(x, 3) for x in fold_target]} over {fold_total_s}s"
            )
            self._center_all_servos(fold_target, fold_total_s, fold_accel_s)
            present = fold_target

        self._present_pose = list(present)

        # Put the gripper into Current-Based Position control so it can hold an
        # object indefinitely without overloading and so a dropped object is
        # detectable. Only when grasp_on_close is enabled and the gripper is
        # live. Done last so centering/fold (which force position mode) are past.
        if self.gripper_live and bool(self.get_parameter("grasp_on_close").value):
            self._setup_gripper_current_mode(present[-1])

        # Smooth streaming: arm joints on Time-based Profile so goals stream
        # as coordinated moves (all joints arrive together each tick).
        if bool(self.get_parameter("smooth_stream").value):
            self._enable_stream_profile()

    # ------------------------------------------------------------------ #
    #  simulation init (no hardware — RViz visualization)                #
    # ------------------------------------------------------------------ #
    def _init_sim(self) -> None:
        """No serial port, no Dynamixels. Start at the fold pose so IK is
        immediately usable, and treat goal_rad as ground truth (published on
        /joint_states for RViz). gentle/centering/fold are skipped because
        there's nothing to move."""
        self.port = None
        self.packet = None
        self.gripper_live = True
        self._run_vel = int(self.get_parameter("run_vel").value)
        self._run_acc = int(self.get_parameter("run_acc").value)

        import math as _math
        raw = [float(x) for x in self.get_parameter("fold_position_rad").value]
        if len(raw) < self.n_total:
            raw = raw + [0.0] * (self.n_total - len(raw))
        present = [0.0 if _math.isnan(raw[i]) else raw[i] for i in range(self.n_total)]
        self._present_pose = present
        self._pretorque_pose = list(present)
        self.get_logger().warn(
            "SIM MODE: no hardware. Publishing /joint_states for RViz only. "
            f"Start pose = {[round(v, 3) for v in present]}"
        )

    # ------------------------------------------------------------------ #
    #  ROS subscriptions / publishers / IK solver                        #
    # ------------------------------------------------------------------ #
    def _setup_ros_io(self) -> None:
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            WirelessController, "/wirelesscontroller", self._on_remote, qos_be
        )
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10)
        # EoE marker — a small coordinate triad (x=red, y=green, z=blue) at
        # the controlled end-effector point, so you can see both the IK target
        # position AND its orientation in RViz.
        self.pub_marker = self.create_publisher(
            MarkerArray, "/go2w_remote_arm/ee_marker", 5
        )
        # Gripper state (OPEN/CLOSING/HOLDING/CLOSED/DROPPED). TRANSIENT_LOCAL so
        # a monitor that subscribes later still receives the last known state.
        grip_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_grip = self.create_publisher(String, self.grip_state_topic, grip_qos)
        # External gripper command — same code path as the remote button, so
        # autonomous grasps (direct_pick) behave like the working remote grasp.
        self.create_subscription(
            String, "/go2w_remote_arm/gripper_cmd", self._on_gripper_cmd, 10)
        if bool(self.get_parameter("enable_joint_command_subscriber").value):
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_command_topic").value),
                self._on_joint_command,
                10,
            )
            self.get_logger().info(
                f"Automatic joint commands enabled on "
                f"{self.get_parameter('joint_command_topic').value}"
            )
        # ---- Cartesian-goal servo I/O (arm-server for autonomous pick) ----
        self._cart_target_pos = None   # np.array(3) world, or None = idle
        self._cart_target_R = None     # np.array(3,3) or None
        self._cart_hold = False        # True = tracking: follow a moving
        # target continuously, never auto-cancel on reach
        self._cart_status = "idle"
        self._cart_stall = 0
        self._cart_last_err = None
        if bool(self.get_parameter("cart_enabled").value):
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("cart_goal_topic").value),
                self._on_cart_goal, 10,
            )
            # continuous TRACKING target — follow a moving pose (e.g. a
            # visually-tracked object) without ever auto-cancelling.
            self.create_subscription(
                PoseStamped, "/arm_cart_track", self._on_cart_track, 10)
            self.create_subscription(
                String, "/arm_cart_stop", self._on_cart_stop, 10)
            self.pub_cart_state = self.create_publisher(
                PoseStamped, "/arm_cart_state", 10)
            self.pub_cart_status = self.create_publisher(
                String, "/arm_cart_status",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST))
            self.get_logger().info(
                "Cartesian-goal servo enabled on "
                f"{self.get_parameter('cart_goal_topic').value} "
                "(→ /arm_cart_state, /arm_cart_status)")
        else:
            self.pub_cart_state = None
            self.pub_cart_status = None

        self.dt = 1.0 / self.publish_rate
        self.create_timer(self.dt, self._tick)

        # ---- IK solver (optional) ----
        self.mode = MODE_JOINT
        self.ik = None
        self.ik_target_pos = None  # persistent Cartesian target (m), seeded on IK entry
        self.ik_target_R = None    # persistent target orientation (3×3)
        # max distance the target may lead the actual EoE before being pulled
        # back — prevents the target running off into unreachable space.
        self.ik_max_lead = 0.04
        if bool(self.get_parameter("ik_enabled").value):
            try:
                from .ik_solver import IKSolver  # imported lazily
                self.ik = IKSolver(
                    base_link=str(self.get_parameter("ik_base_link").value),
                    tip_link=str(self.get_parameter("ik_tip_link").value),
                    urdf_pkg=str(self.get_parameter("ik_urdf_pkg").value),
                    damping=float(self.get_parameter("ik_damping").value),
                )
                # Adopt the real URDF joint limits for the soft-clamp so the
                # arm never folds past its mechanical range (the ±π default is
                # far looser than the real Chain limits and caused the joints
                # to "clash" / fold through themselves in IK mode).
                for j in range(min(6, self.n_arm, len(self.ik.q_min))):
                    self.joint_min[j] = float(self.ik.q_min[j])
                    self.joint_max[j] = float(self.ik.q_max[j])
                self.get_logger().info(
                    f"IK solver ready (chain {self.ik.base_link} → "
                    f"{self.ik.tip_link}, {self.ik.n_joints} joints). "
                    f"Joint limits from URDF: "
                    f"min={[round(v,2) for v in self.ik.q_min]} "
                    f"max={[round(v,2) for v in self.ik.q_max]}. "
                    f"Tap Select to toggle JOINT ↔ IK mode."
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"IK init failed: {exc} — IK mode disabled"
                )
                self.ik = None
        self.ik_lin = float(self.get_parameter("ik_lin_speed").value)
        self.ik_ang = float(self.get_parameter("ik_ang_speed").value)
        self.ik_signs = [float(x) for x in self.get_parameter("ik_axis_signs").value]
        if len(self.ik_signs) < 6:
            self.ik_signs = self.ik_signs + [1.0] * (6 - len(self.ik_signs))
        self.ik_self_collision = bool(self.get_parameter("ik_self_collision").value)
        self.ik_collision_radius = float(self.get_parameter("ik_collision_radius").value)
        self._in_collision = False  # latched state for marker colouring

        self.get_logger().info(
            f"go2w_remote_arm ready: {self.n_total} joints, "
            f"vel={self.joint_velocity} rad/s, rate={self.publish_rate} Hz, "
            f"mode={self.mode}, sim={self.sim_mode}"
        )

    # ---------------- low-level helpers ----------------
    def _write1(self, dxl_id: int, addr: int, value: int) -> None:
        rc, err = self.packet.write1ByteTxRx(self.port, dxl_id, addr, value & 0xFF)
        if rc != COMM_SUCCESS or err != 0:
            self.get_logger().warn(
                f"write1 id={dxl_id} addr={addr}: rc={rc} err={err}"
            )

    def _set_profile_ids(self, ids, vel_ticks: int, acc_ticks: int) -> None:
        """Profile write for a subset of motors (e.g. arm only, not gripper)."""
        self.sync_write_vel.clearParam()
        self.sync_write_acc.clearParam()
        for mid in ids:
            self.sync_write_vel.addParam(mid, _u32(vel_ticks))
            self.sync_write_acc.addParam(mid, _u32(acc_ticks))
        rc = self.sync_write_vel.txPacket()
        if rc != 0:
            raise IOError(f"profile vel write: {self.packet.getTxRxResult(rc)}")
        rc = self.sync_write_acc.txPacket()
        if rc != 0:
            raise IOError(f"profile acc write: {self.packet.getTxRxResult(rc)}")

    def _arm_ids(self):
        return self.motor_ids[:self.n_arm]

    def _enable_stream_profile(self) -> None:
        """Switch the ARM joints (not the gripper) to Time-based Profile
        streaming so every tick's goal is reached by all joints
        simultaneously — coordinated Cartesian motion. Requires a brief
        torque-off (Drive Mode is EEPROM)."""
        ms = int(self.get_parameter("stream_profile_ms").value)
        acc = int(self.get_parameter("stream_accel_ms").value)
        acc = min(acc, ms // 2)
        arm = self._arm_ids()
        present = self._read_positions_rad()
        for mid in arm:
            self._write1(mid, ADDR_TORQUE_ENABLE, 0)
        for mid in arm:
            self._write1(mid, ADDR_DRIVE_MODE, DRIVE_MODE_TIME_BASED)
        self._set_profile_ids(arm, ms, acc)
        # goals = present so nothing snaps when torque returns
        self._write_goals_rad(present)
        for mid in arm:
            self._write1(mid, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info(
            f"smooth stream ON: arm joints Time-based Profile "
            f"({ms} ms move / {acc} ms accel per goal)"
        )

    def _set_profile(self, vel_ticks: int, acc_ticks: int) -> None:
        self.sync_write_vel.clearParam()
        self.sync_write_acc.clearParam()
        for mid in self.motor_ids:
            self.sync_write_vel.addParam(mid, _u32(vel_ticks))
            self.sync_write_acc.addParam(mid, _u32(acc_ticks))
        rc = self.sync_write_vel.txPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"profile vel write: {self.packet.getTxRxResult(rc)}")
        rc = self.sync_write_acc.txPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"profile acc write: {self.packet.getTxRxResult(rc)}")

    def _read_positions_rad(self) -> List[float]:
        rc = self.sync_read.txRxPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"sync read: {self.packet.getTxRxResult(rc)}")
        out: List[float] = []
        for mid in self.motor_ids:
            raw = self.sync_read.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT)
            out.append(ticks_to_rad(_s32(raw)))
        return out

    def _read_present_block(self):
        """One sync-read transaction over the 126–135 block → (positions [rad]
        for every motor, gripper |Present Current| [mA] or None). The gripper's
        current comes free with everyone's position, so no extra single reads."""
        rc = self.sync_read.txRxPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"sync read: {self.packet.getTxRxResult(rc)}")
        positions: List[float] = []
        for mid in self.motor_ids:
            raw = self.sync_read.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT)
            positions.append(ticks_to_rad(_s32(raw)))
        grip_cur = None
        if self.gripper_live:
            raw_c = self.sync_read.getData(self.motor_ids[-1], ADDR_PRESENT_CURRENT, 2)
            grip_cur = abs(_s16(raw_c) * CURRENT_UNIT_MA)
        return positions, grip_cur

    def _write_goals_rad(self, goals: List[float]) -> None:
        self.sync_write_goal.clearParam()
        for mid, r in zip(self.motor_ids, goals):
            self.sync_write_goal.addParam(mid, _u32(rad_to_ticks(r)))
        rc = self.sync_write_goal.txPacket()
        if rc != COMM_SUCCESS:
            self.get_logger().warn(
                f"sync write goal: {self.packet.getTxRxResult(rc)}"
            )

    # ---------------- gripper: current-based hold + state machine ----------------
    def _setup_gripper_current_mode(self, hold_rad: float) -> None:
        """Switch the gripper (last motor) into Current-Based Position control
        with a bounded Goal Current, a gentle profile, and its goal preloaded to
        the present position so it doesn't jump. After this the per-tick sync
        goal-position write still drives it, but force is capped."""
        gid = self.motor_ids[-1]
        limit_raw = max(1, int(round(self.grip_current_limit / CURRENT_UNIT_MA)))
        try:
            self._write1(gid, ADDR_TORQUE_ENABLE, 0)
            self._write1(gid, ADDR_OPERATING_MODE, OP_MODE_CURRENT_POSITION)
            # Goal Current (2-byte) — the squeeze-force cap.
            rc, err = self.packet.write2ByteTxRx(
                self.port, gid, ADDR_GOAL_CURRENT, limit_raw & 0xFFFF
            )
            if rc != COMM_SUCCESS or err != 0:
                self.get_logger().warn(f"gripper goal-current write rc={rc} err={err}")
            # gentle profile so the close is slow/visible
            self.packet.write4ByteTxRx(
                self.port, gid, ADDR_PROFILE_VELOCITY, self.grip_profile_vel & 0xFFFFFFFF
            )
            self.packet.write4ByteTxRx(
                self.port, gid, ADDR_GOAL_POSITION, rad_to_ticks(hold_rad) & 0xFFFFFFFF
            )
            self._write1(gid, ADDR_TORQUE_ENABLE, 1)
            self._grip_mode5 = True
            self.get_logger().info(
                f"gripper ID {gid}: Current-Based Position mode, "
                f"limit={self.grip_current_limit:.0f} mA ({limit_raw} raw)"
            )
        except Exception as exc:
            self.get_logger().warn(f"gripper mode-5 setup failed: {exc}")
            self._grip_mode5 = False

    def _update_gripper(self, pos: float, cur: float) -> None:
        """Classify the gripper state from a position/current sample already
        obtained via this tick's shared sync-read, publish it, and log
        transitions. The commanded goal (goal_rad[-1]) stays at the last
        open/close target; because the motor is current-limited it holds an
        object safely and, when the object is removed, the jaws collapse to the
        close limit — which we report as DROPPED."""
        self._grip_pos, self._grip_cur = pos, cur

        at_close = abs(pos - self.grip_close) <= self.grip_close_tol
        at_open = abs(pos - self.grip_open) <= self.grip_open_tol
        prev = self._grip_state
        action = self._grip_action

        # debounced holding: sustained current while NOT fully closed
        if cur >= self.grasp_threshold and not at_close:
            self._grip_hold_count += 1
        else:
            self._grip_hold_count = 0
        holding = self._grip_hold_count >= self.grasp_consecutive

        if action == "open":
            state = "OPEN" if at_open else "OPENING"
        elif action == "close":
            if holding:
                state = "HOLDING"
            elif at_close:
                # jaws met the close limit with no resistance
                state = "DROPPED" if prev in ("HOLDING", "DROPPED") else "CLOSED"
            else:
                state = "CLOSING"
        else:  # nothing commanded yet
            state = "OPEN" if at_open else ("CLOSED" if at_close else "MID")

        if state != prev:
            if state == "HOLDING":
                self.get_logger().info(
                    f"gripper: OBJECT GRASPED @ {pos:+.3f} rad, {cur:.0f} mA"
                )
            elif state == "DROPPED":
                self.get_logger().warn(
                    f"gripper: OBJECT DROPPED — jaws collapsed to close "
                    f"@ {pos:+.3f} rad, {cur:.0f} mA"
                )
            elif state == "CLOSED":
                self.get_logger().info(
                    f"gripper: CLOSED empty (position limit) "
                    f"@ {pos:+.3f} rad, {cur:.0f} mA"
                )
            elif state == "OPEN":
                self.get_logger().info(f"gripper: OPEN @ {pos:+.3f} rad")
            self._grip_state = state
            self._publish_grip_state()
        else:
            # low-rate heartbeat so a fresh subscriber still gets the state
            self._grip_pub_count += 1
            if self._grip_pub_count >= max(1, int(self.publish_rate // 2)):
                self._grip_pub_count = 0
                self._publish_grip_state()

    def _publish_grip_state(self) -> None:
        if self.pub_grip is None:
            return
        m = String()
        m.data = (
            f"{self._grip_state} pos={self._grip_pos:+.3f} "
            f"cur={self._grip_cur:.0f}mA"
        )
        self.pub_grip.publish(m)

    # ---------------- gentle startup ----------------
    def _gentle_torque_on(
        self,
        slow_vel: int,
        slow_acc: int,
        run_vel: int,
        run_acc: int,
        settle_s: float,
    ) -> List[float]:
        # disable torque, set position mode
        self.get_logger().info("disable torque & set position mode")
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 0)
            self._write1(mid, ADDR_OPERATING_MODE, OP_MODE_POSITION)

        present = self._read_positions_rad()
        self.get_logger().info(f"present (rad): {[round(v,3) for v in present]}")

        # preload goal = present so nothing snaps
        self._write_goals_rad(present)

        # slow profile
        self.get_logger().info(f"slow profile vel={slow_vel} acc={slow_acc}")
        self._set_profile(slow_vel, slow_acc)

        # torque on
        self.get_logger().info("torque ON")
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 1)

        time.sleep(settle_s)

        # ramp to run profile
        self.get_logger().info(f"run profile vel={run_vel} acc={run_acc}")
        self._set_profile(run_vel, run_acc)
        return present

    # ---------------- centering ----------------
    def _center_all_servos(
        self,
        target_rad: List[float],
        total_s: float,
        accel_s: float,
    ) -> None:
        """Drive every live motor to `target_rad` using the XM430 Time-based
        Profile so motion is a trapezoid (or triangle if accel_s == total_s/2).

        Profile Velocity → total movement time (ms)
        Profile Acceleration → acceleration time (ms)
        Deceleration time = total - accel (firmware infers it).

        After motion completes, Drive Mode is reverted to velocity-based and
        the normal run profile is restored so teleop jog behaves as usual.
        """
        if not target_rad:
            return
        total_ms = max(0, int(round(total_s * 1000)))
        accel_ms = max(0, int(round(accel_s * 1000)))
        # Firmware requires accel_ms <= total_ms / 2 for a valid trapezoid.
        if accel_ms > total_ms // 2:
            accel_ms = total_ms // 2
        decel_ms = total_ms - accel_ms

        self.get_logger().info(
            f"Centering → {[round(v, 2) for v in target_rad]}  "
            f"(accel={accel_ms}ms cruise={total_ms - 2*accel_ms}ms "
            f"decel={decel_ms}ms total={total_ms}ms)"
        )

        # 1) torque OFF (Drive Mode lives in EEPROM, can't change while torqued)
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 0)
        # 2) Drive Mode = Time-based Profile
        for mid in self.motor_ids:
            self._write1(mid, ADDR_DRIVE_MODE, DRIVE_MODE_TIME_BASED)
        # 3) Profile Velocity = total time (ms), Profile Acceleration = accel time (ms)
        self._set_profile(total_ms, accel_ms)
        # 4) torque ON
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 1)
        # 5) goal positions = target
        self._write_goals_rad(target_rad)
        # 6) wait for motion to finish, plus a small buffer
        time.sleep(total_s + 0.3)
        # 7) revert: torque OFF
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 0)
        # 8) Drive Mode = Velocity-based
        for mid in self.motor_ids:
            self._write1(mid, ADDR_DRIVE_MODE, DRIVE_MODE_VELOCITY_BASED)
        # 9) restore normal teleop profile
        self._set_profile(self._run_vel, self._run_acc)
        # 10) torque ON
        for mid in self.motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, 1)

        self.get_logger().info("Centering complete; ready for teleop.")

    def _park_and_release(self) -> None:
        """F1 routine: move arm back to the pre-torque initial pose using the
        same Time-based trapezoidal profile, then disable torque on every motor
        and ask rclpy to shut the node down. After this the launcher will
        notice the child died and reset to "press F3 to start again"."""
        if self._parking:
            return
        if self.sim_mode:
            # In sim there's nothing to park / no torque to release. Just snap
            # the goal back to the initial pose so RViz shows the reset.
            with self.lock:
                self.goal_rad = list(self.initial_pose)
            self.get_logger().info("F1 (sim) → reset to initial pose")
            return
        self._parking = True
        try:
            total_s = float(self.get_parameter("center_total_time_s").value)
            accel_s = float(self.get_parameter("center_accel_time_s").value)
            self.get_logger().info(
                f"F1 → parking to initial pose "
                f"{[round(v, 3) for v in self.initial_pose]}"
            )
            # keep _tick (if it fires) from yanking the arm back: align goal
            # with the destination first.
            with self.lock:
                self.goal_rad = list(self.initial_pose)

            self._center_all_servos(self.initial_pose, total_s, accel_s)

            self.get_logger().info(
                f"Releasing torque (keeping ON for IDs {self.keep_torque_ids})"
            )
            for mid in self.motor_ids:
                if mid in self.keep_torque_ids:
                    self.get_logger().info(f"  ID {mid}: torque kept ON")
                    continue
                self._write1(mid, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().info(f"  ID {mid}: torque OFF")

            self.get_logger().info("Park complete — shutting down.")
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ---------------- error recovery ----------------
    def _clear_hw_errors_via_reboot(self) -> None:
        """Read each motor's Hardware Error register; reboot any that has a
        non-zero flag. Stale flags from a prior session (electrical-shock,
        overheat that's no longer present, etc.) auto-disable torque and
        would otherwise need DYNAMIXEL Wizard to clear."""
        flagged: List[int] = []
        for mid in self.motor_ids:
            he, rc, err = self.packet.read1ByteTxRx(self.port, mid, ADDR_HARDWARE_ERROR)
            if rc == COMM_SUCCESS and he != 0:
                self.get_logger().warn(
                    f"ID {mid}: HW error 0x{he:02X} → rebooting"
                )
                flagged.append(mid)
        if not flagged:
            return
        for mid in flagged:
            self.packet.reboot(self.port, mid)
        time.sleep(2.0)
        for mid in flagged:
            he, rc, err = self.packet.read1ByteTxRx(self.port, mid, ADDR_HARDWARE_ERROR)
            if rc != COMM_SUCCESS:
                self.get_logger().error(
                    f"ID {mid}: comm fail after reboot — check wiring/power"
                )
            elif he != 0:
                self.get_logger().error(
                    f"ID {mid}: HW error STILL 0x{he:02X} after reboot — "
                    f"check load/wiring/power"
                )
            else:
                self.get_logger().info(f"ID {mid}: HW error cleared")

    # ---------------- remote ----------------
    def _on_remote(self, msg: WirelessController) -> None:
        f1_pressed = False
        with self.lock:
            self.prev_keys = self.keys
            self.keys = int(msg.keys)
            self.axes = (
                float(msg.lx), float(msg.ly), float(msg.rx), float(msg.ry)
            )
            self.last_remote_t = time.monotonic()

            rising = (~self.prev_keys) & self.keys

            # F1 → park & release (must be handled OUTSIDE the lock)
            if rising & BTN_F1 and not self._parking:
                f1_pressed = True

            # Select → toggle JOINT ↔ IK mode
            if (rising & BTN_SELECT) and not self._parking:
                if self.ik is None:
                    self.get_logger().warn(
                        "IK solver unavailable — cannot switch mode"
                    )
                else:
                    self.mode = MODE_IK if self.mode == MODE_JOINT else MODE_JOINT
                    if self.mode == MODE_IK:
                        import numpy as np
                        q = np.array(self.goal_rad[:6])
                        # Seed the persistent target pose from the present tip
                        # so the first jog starts exactly where the gripper is.
                        xyz, R = self.ik.fk_pose(q)
                        self.ik_target_pos = xyz.copy()
                        self.ik_target_R = R.copy()
                        m = self.ik.manipulability(q)
                        if m < 1e-4:
                            self.get_logger().warn(
                                f"IK mode entered at near-singular pose "
                                f"(manipulability={m:.2e}). Linear motion may "
                                f"feel stuck — switch back to JOINT and "
                                f"reposition (e.g. bend joint2 by ~30°), then "
                                f"re-enter IK."
                            )
                        else:
                            self.get_logger().info(
                                f"Mode → IK  (tip at {xyz.round(3)}, "
                                f"manipulability={m:.3f})"
                            )
                    else:
                        self.ik_target_pos = None
                        self.ik_target_R = None
                        self.get_logger().info(f"Mode → {self.mode}")

            # edge-triggered gripper (only if gripper motor responded at startup)
            if self.gripper_live and not self._parking:
                if rising & (1 << self.grip_btn_open):
                    self._do_gripper("open")
                if rising & (1 << self.grip_btn_close):
                    self._do_gripper("close")

        if f1_pressed:
            self._park_and_release()

    def _clamp(self, j_idx: int, v: float) -> float:
        if j_idx < len(self.joint_min):
            v = max(self.joint_min[j_idx], v)
        if j_idx < len(self.joint_max):
            v = min(self.joint_max[j_idx], v)
        return v

    def _clamp_grip(self, v: float) -> float:
        return max(self.grip_min, min(self.grip_max, v))

    def _deadzone(self, v: float) -> float:
        if abs(v) < self.axis_deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        scaled = (abs(v) - self.axis_deadzone) / (1.0 - self.axis_deadzone)
        return sign * max(0.0, min(1.0, scaled))

    # ---------------- automatic joint commands ----------------
    def _do_gripper(self, which: str) -> None:
        """The single gripper open/close action — used by BOTH the remote
        button and the external /gripper_cmd topic, so autonomous grasps
        behave exactly like the (working) remote grasp: sets _grip_action so
        the current-based grasp state machine engages, resets the hold
        counter, and commands the open/close target."""
        if not self.gripper_live or self._parking:
            return
        if which == "open":
            self._grip_action = "open"
            self._grip_hold_count = 0
            self.goal_rad[-1] = self._clamp_grip(self.grip_open)
        elif which == "close":
            self._grip_action = "close"
            self._grip_hold_count = 0
            self.goal_rad[-1] = self._clamp_grip(self.grip_close)
        else:
            self.get_logger().warn(f"gripper_cmd: unknown '{which}'")
            return
        self.get_logger().info(
            f"gripper → {which.upper()} cmd ({self.goal_rad[-1]:+.2f})")

    def _on_gripper_cmd(self, msg: String) -> None:
        self._do_gripper(msg.data.strip().lower())

    def _on_joint_command(self, msg: JointState) -> None:
        if self._parking:
            self.get_logger().warn("joint_command ignored while parking")
            return
        if not msg.name or len(msg.position) != len(msg.name):
            self.get_logger().warn("joint_command: name/position size mismatch")
            return

        updated = []
        with self.lock:
            name_to_index = {name: i for i, name in enumerate(self.joint_names)}
            for name, pos in zip(msg.name, msg.position):
                if name not in name_to_index:
                    self.get_logger().warn(f"joint_command: unknown joint '{name}'")
                    continue
                idx = name_to_index[name]
                target = float(pos)
                if idx < self.n_arm:
                    target = self._clamp(idx, target)
                elif self.gripper_live and idx == self.n_total - 1:
                    target = self._clamp_grip(target)
                self.goal_rad[idx] = target
                updated.append(name)

        if updated:
            self.get_logger().info(
                f"joint_command applied for {updated}",
                throttle_duration_sec=1.0,
            )

    # ---------------- Cartesian-goal servo ----------------
    def _on_cart_goal(self, msg: PoseStamped) -> None:
        """Accept a tool-pose goal in the arm base frame. Setting it activates
        the straight-line servo in the tick loop."""
        if self.ik is None or self.n_arm < 6:
            self.get_logger().warn("cart goal ignored: IK not available")
            return
        import numpy as np
        p = msg.pose.position
        q = msg.pose.orientation
        with self.lock:
            self._cart_target_pos = np.array([p.x, p.y, p.z], dtype=float)
            self._cart_target_R = _quat_to_matrix(q.x, q.y, q.z, q.w)
            self._cart_hold = False
            self._cart_status = "moving"
            self._cart_stall = 0
            self._cart_last_err = None
        self.get_logger().info(
            f"cart goal → ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")

    def _on_cart_track(self, msg: PoseStamped) -> None:
        """Continuously-tracked target: the arm follows it wherever it moves,
        never auto-cancelling. Publish this at ~10 Hz from a live object
        detection to chase the object (including lateral/y moves)."""
        if self.ik is None or self.n_arm < 6:
            return
        import numpy as np
        p = msg.pose.position
        q = msg.pose.orientation
        with self.lock:
            self._cart_target_pos = np.array([p.x, p.y, p.z], dtype=float)
            self._cart_target_R = _quat_to_matrix(q.x, q.y, q.z, q.w)
            self._cart_hold = True
            if self._cart_status not in ("tracking",):
                self._cart_status = "tracking"
            self._cart_stall = 0

    def _on_cart_stop(self, msg: String) -> None:
        with self.lock:
            self._cart_cancel("idle")

    def _cart_cancel(self, status: str) -> None:
        self._cart_target_pos = None
        self._cart_target_R = None
        self._cart_hold = False
        self._cart_status = status

    def _cart_servo_step(self) -> None:
        """One tick of straight-line Cartesian servoing toward the active
        goal. Updates goal_rad[:6]. Interpolates the TOOL pose at a capped
        speed so the path is straight, solving pose IK for each waypoint."""
        import numpy as np
        q = np.array(self.goal_rad[:6])
        p_now, R_now = self.ik.fk_pose(q)
        dp = self._cart_target_pos - p_now
        dist = float(np.linalg.norm(dp))

        # rotation error angle
        R_err = self._cart_target_R @ R_now.T
        cos_a = max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0))
        ang = math.acos(cos_a)

        pos_tol = float(self.get_parameter("cart_pos_tol").value)
        ang_tol = float(self.get_parameter("cart_ang_tol").value)
        if dist < pos_tol and ang < ang_tol:
            if self._cart_hold:
                # tracking: sit on the target, keep following if it moves
                self._cart_status = "tracking"
                return
            self._cart_cancel("reached")
            self.get_logger().info("cart goal REACHED")
            return

        # stall detection: no meaningful progress for N ticks → unreachable.
        # Skipped while tracking (a moving target legitimately keeps a
        # non-zero error and we must keep chasing, not give up).
        if not self._cart_hold:
            if self._cart_last_err is not None and \
                    self._cart_last_err - dist < 1e-4 and dist > pos_tol:
                self._cart_stall += 1
            else:
                self._cart_stall = 0
            self._cart_last_err = dist
            if self._cart_stall > int(self.get_parameter("cart_stall_iters").value):
                self._cart_cancel("unreachable")
                self.get_logger().warn(
                    f"cart goal UNREACHABLE (stalled {dist*1000:.0f} mm away)")
                return

        # step-limited intermediate tool pose (straight line)
        lin_step = float(self.get_parameter("cart_lin_speed").value) * self.dt
        ang_step = float(self.get_parameter("cart_ang_speed").value) * self.dt
        if dist > lin_step:
            p_cmd = p_now + dp * (lin_step / dist)
        else:
            p_cmd = self._cart_target_pos
        if ang > ang_step and ang > 1e-6:
            frac = ang_step / ang
            axis = np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ]) / (2.0 * math.sin(ang))
            R_cmd = _rot_from_rotvec(axis * ang * frac) @ R_now
        else:
            R_cmd = self._cart_target_R

        q_sol, _ok = self.ik.solve_pose_ik(q, p_cmd, R_cmd)
        q_clamped = [self._clamp(j, float(q_sol[j])) for j in range(6)]

        # self-collision gate (same as IK jog)
        if (self.ik_self_collision
                and self.ik.self_collides(np.array(q_clamped),
                                          self.ik_collision_radius)):
            self._cart_cancel("unreachable")
            self.get_logger().warn("cart goal blocked by self-collision")
            return
        for j in range(6):
            self.goal_rad[j] = q_clamped[j]

    def _publish_cart_feedback(self, positions) -> None:
        if self.pub_cart_state is None or self.ik is None:
            return
        import numpy as np
        try:
            p, R = self.ik.fk_pose(np.array(positions[:6]))
        except Exception:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = str(self.get_parameter("ik_base_link").value)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
            float(p[0]), float(p[1]), float(p[2])
        qx, qy, qz, qw = _matrix_to_quat(R)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_cart_state.publish(ps)
        s = String()
        s.data = self._cart_status
        self.pub_cart_status.publish(s)

    # ---------------- IK mode helpers ----------------
    def _compute_ik_inputs(self, keys: int, axes):
        """Map remote inputs into a Cartesian linear velocity and an angular
        velocity, BOTH in the fixed WORLD frame.

        Translation moves the EoE along world axes (Up = world +z always).
        Orientation rotates the gripper about fixed world axes at the EoE
        point, so tilting the gripper never changes which way "Up" or "pitch"
        go — the control stays intuitive no matter the arm's pose:
          Up / Down        → world +z / -z
          Left / Right      → world +y / -y
          Y / A             → world +x / -x
          roll  (X / B)     → rotation about world x
          pitch (stick ry)  → rotation about world y
          yaw   (R1 / R2)   → rotation about world z

        Returns (lin_vel_world[3], ang_vel_world[3]).
        Sign vector `ik_axis_signs[0..5]` flips [vx,vy,vz, roll,pitch,yaw].
        """
        import numpy as np
        lin = self.ik_lin
        ang = self.ik_ang
        s = self.ik_signs

        vx = vy = vz = 0.0
        if keys & BTN_Y:   vx += lin
        if keys & BTN_A:   vx -= lin
        if keys & BTN_LEFT:  vy += lin
        if keys & BTN_RIGHT: vy -= lin
        if keys & BTN_UP:    vz += lin
        if keys & BTN_DOWN:  vz -= lin

        roll = 0.0   # about tool x
        if keys & BTN_X:   roll += ang
        if keys & BTN_B:   roll -= ang
        ry = self._deadzone(axes[3]) if len(axes) > 3 else 0.0
        pitch = ang * ry          # about tool y (right stick analog)
        yaw = 0.0                 # about tool z
        if keys & BTN_R1:  yaw += ang
        if keys & BTN_R2:  yaw -= ang

        lin_vel = np.array([s[0]*vx, s[1]*vy, s[2]*vz])
        ang_vel = np.array([s[3]*roll, s[4]*pitch, s[5]*yaw])
        return lin_vel, ang_vel

    # ---------------- main loop ----------------
    def _tick(self) -> None:
        if self._parking:
            return  # _park_and_release owns the bus during the trapezoidal move
        with self.lock:
            keys = self.keys
            axes = self.axes
            mode = self.mode
            stale = (
                (time.monotonic() - self.last_remote_t) > 0.5
                if self.last_remote_t
                else True
            )
            cart_active = self._cart_target_pos is not None
            if cart_active and not stale:
                # deadman override: any live remote input cancels the
                # autonomous Cartesian goal so the operator always wins.
                self._cart_cancel("idle")
                cart_active = False
                self.get_logger().warn("cart goal cancelled by remote input")

            if cart_active:
                # ---- autonomous Cartesian-goal servo (straight line) ----
                self._cart_servo_step()
            elif not stale and mode == MODE_IK and self.ik is not None and self.n_arm >= 6:
                # ---- IK mode: persistent WORLD-frame target, arm tracks it ----
                import numpy as np
                lin_vel, ang_vel = self._compute_ik_inputs(keys, axes)

                q = np.array(self.goal_rad[:6])
                if self.ik_target_pos is None or self.ik_target_R is None:
                    p0, R0 = self.ik.fk_pose(q)
                    self.ik_target_pos = p0.copy()
                    self.ik_target_R = R0.copy()

                # ---- cmd_vel Cartesian jog with pose-anchor feedback ----
                # The joystick is a twist command, but we also integrate a
                # commanded ANCHOR pose and servo on it. This is what makes a
                # pure rotation pivot exactly ABOUT the EoE: the anchor
                # position stays put and the feedback continuously cancels
                # any translation leaked by the damped pseudoinverse or by a
                # joint hitting its limit. Conversely, translation holds the
                # current orientation.
                if bool(self.get_parameter("ik_tool_frame_rotation").value) \
                        and (np.abs(ang_vel) > 1e-6).any():
                    ang_vel = self.ik.ee_to_base_angular(q, ang_vel)

                p_now, R_now = self.ik.fk_pose(q)
                if self.ik_target_pos is None or self.ik_target_R is None:
                    self.ik_target_pos = p_now.copy()
                    self.ik_target_R = R_now.copy()

                # integrate the anchor from the commanded twist
                self.ik_target_pos = self.ik_target_pos + lin_vel * self.dt
                if (np.abs(ang_vel) > 1e-6).any():
                    self.ik_target_R = _rot_from_rotvec(ang_vel * self.dt) \
                        @ self.ik_target_R
                # never let the anchor run away from the reachable pose
                lead = self.ik_target_pos - p_now
                d = float(np.linalg.norm(lead))
                if d > self.ik_max_lead:
                    self.ik_target_pos = p_now + lead * (self.ik_max_lead / d)

                # feedback twist = feedforward + P on pose error
                kp = float(self.get_parameter("ik_pos_gain").value)
                kr = float(self.get_parameter("ik_rot_gain").value)
                v = lin_vel + kp * (self.ik_target_pos - p_now)
                R_err = self.ik_target_R @ R_now.T
                # log map (rotation vector) of R_err
                cos_a = max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0))
                a = math.acos(cos_a)
                if a > 1e-6:
                    axis = np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ]) / (2.0 * math.sin(a))
                    rot_err = axis * a
                else:
                    rot_err = np.zeros(3)
                # anti-windup: kalau rotasi mentok (arm tak bisa berputar
                # lebih jauh sambil menahan ujung), jangan biarkan anchor_R
                # terus berjalan menjauh — tarik mundur ke batas.
                if a > 0.35:
                    self.ik_target_R = _rot_from_rotvec(
                        rot_err * ((a - 0.35) / a) * -1.0) @ self.ik_target_R
                w = ang_vel + kr * rot_err

                if (np.abs(np.concatenate([v, w])) > 1e-5).any():
                    # task-priority: posisi EoE tugas utama, rotasi hanya di
                    # null-space-nya → rotasi TIDAK PERNAH menggeser EoE;
                    # kalau tak bisa berputar lagi, dia berhenti berputar.
                    qdot = self.ik.velocity_ik_priority(q, v, w)
                    qmax = float(self.get_parameter("ik_qdot_max").value)
                    peak = float(np.max(np.abs(qdot)))
                    if peak > qmax:      # scale, don't clip → direction kept
                        qdot = qdot * (qmax / peak)
                    q_sol = q + qdot * self.dt
                else:
                    q_sol = q
                q_clamped = np.array(
                    [self._clamp(j, float(q_sol[j])) for j in range(6)]
                )

                # Self-collision gate: if the candidate pose would make links
                # intersect, reject it — freeze the joints at the current pose
                # and pull the target back to the EoE so it doesn't run away.
                if (self.ik_self_collision
                        and self.ik.self_collides(q_clamped, self.ik_collision_radius)):
                    if not self._in_collision:
                        self.get_logger().warn(
                            "self-collision blocked — arm held at boundary",
                            throttle_duration_sec=1.0,
                        )
                    self._in_collision = True
                    # keep goal_rad unchanged; snap target back to actual EoE
                    self.ik_target_pos, self.ik_target_R = self.ik.fk_pose(q)
                else:
                    self._in_collision = False
                    for j in range(6):
                        self.goal_rad[j] = float(q_clamped[j])
            elif not stale:
                # ---- JOINT mode: per-joint buttons / analog ----
                step = self.joint_velocity * self.dt
                for j in range(self.n_arm):
                    sign = self.joint_signs[j] if j < len(self.joint_signs) else 1.0
                    axis_idx = self.joint_axes[j]
                    delta = 0.0
                    if 0 <= axis_idx < 4:
                        # analog axis mode
                        a = self._deadzone(axes[axis_idx])
                        delta = step * sign * a
                    else:
                        # button-pair mode
                        dec_bit, inc_bit = self.button_pairs[j]
                        if 0 <= inc_bit < 16 and keys & (1 << inc_bit):
                            delta += step * sign
                        if 0 <= dec_bit < 16 and keys & (1 << dec_bit):
                            delta -= step * sign
                    if delta != 0.0:
                        self.goal_rad[j] = self._clamp(j, self.goal_rad[j] + delta)
            # else: hold last goals — arm freezes if remote stops

        if self.sim_mode:
            # No motors: goal_rad IS the state. Publish it so RViz shows it.
            with self.lock:
                positions = list(self.goal_rad)
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = list(self.joint_names)
            m.position = positions
            self.pub_state.publish(m)
            self._publish_ee_marker(positions)
            self._publish_cart_feedback(positions)
            return

        # write goals every tick so motors actively track even without input
        try:
            self._write_goals_rad(self.goal_rad)
        except Exception as exc:
            self.get_logger().warn(f"write goals: {exc}")

        # single sync-read: every motor's position + the gripper's current in
        # one transaction. Feed the gripper state machine from that same sample
        # (no extra I/O), then publish JointState.
        try:
            positions, grip_cur = self._read_present_block()
        except Exception as exc:
            self.get_logger().warn(
                f"read positions: {exc}", throttle_duration_sec=2.0
            )
            return

        if self._grip_mode5 and self.gripper_live and grip_cur is not None:
            self._update_gripper(positions[-1], grip_cur)

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(self.joint_names)
        m.position = positions
        self.pub_state.publish(m)
        self._publish_ee_marker(positions)
        self._publish_cart_feedback(positions)

    def _triad_markers(self, ns, xyz, R, length, alpha, now):
        """Build 3 ARROW markers (x=red, y=green, z=blue) for a pose triad."""
        import numpy as np
        origin = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        colors = [(1.0, 0.1, 0.1), (0.1, 1.0, 0.1), (0.1, 0.4, 1.0)]
        out = []
        for axis in range(3):
            tip = np.asarray(xyz) + np.asarray(R)[:, axis] * length
            m = Marker()
            m.header.frame_id = self.ik.base_link
            m.header.stamp = now
            m.ns = ns
            m.id = axis
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points = [origin, Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))]
            m.scale.x = 0.006
            m.scale.y = 0.012
            m.scale.z = 0.012
            r, g, b = colors[axis]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, alpha
            out.append(m)
        return out

    def _publish_ee_marker(self, positions: List[float]) -> None:
        """Publish two coordinate triads:
          * ee_triad      — the ACTUAL gripper pose (solid arrows)
          * target_triad  — the COMMANDED IK target pose (faded arrows)
        They overlap when the arm is keeping up, and separate when the target
        is jogged faster than the arm can follow or into unreachable space."""
        if self.ik is None or len(positions) < 6:
            return
        try:
            import numpy as np
            xyz, R = self.ik.fk_pose(np.array(positions[:6]))
        except Exception:
            return
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # actual EoE (solid, slightly shorter)
        arr.markers.extend(self._triad_markers("ee_triad", xyz, R, 0.05, 1.0, now))
        # collision indicator: red sphere at the EoE when blocked
        coll = Marker()
        coll.header.frame_id = self.ik.base_link
        coll.header.stamp = now
        coll.ns = "collision"
        coll.id = 0
        coll.type = Marker.SPHERE
        if self._in_collision:
            coll.action = Marker.ADD
            coll.pose.position.x = float(xyz[0])
            coll.pose.position.y = float(xyz[1])
            coll.pose.position.z = float(xyz[2])
            coll.pose.orientation.w = 1.0
            coll.scale.x = coll.scale.y = coll.scale.z = 0.05
            coll.color.r, coll.color.g, coll.color.b, coll.color.a = 1.0, 0.0, 0.0, 0.6
        else:
            coll.action = Marker.DELETE
        arr.markers.append(coll)
        # commanded target (faded, slightly longer) — only in IK mode
        if (self.mode == MODE_IK and self.ik_target_pos is not None
                and self.ik_target_R is not None):
            arr.markers.extend(self._triad_markers(
                "target_triad", self.ik_target_pos, self.ik_target_R,
                0.07, 0.35, now,
            ))
        else:
            # clear stale target arrows when not in IK mode
            for axis in range(3):
                m = Marker()
                m.header.frame_id = self.ik.base_link
                m.header.stamp = now
                m.ns = "target_triad"
                m.id = axis
                m.action = Marker.DELETE
                arr.markers.append(m)
        self.pub_marker.publish(arr)

    # ---------------- lifecycle ----------------
    def destroy_node(self):
        if self.sim_mode:
            return super().destroy_node()
        try:
            self.get_logger().info(
                "Shutdown — leaving torque enabled. Use dynamixel wizard to "
                "disable if needed."
            )
            try:
                if bool(self.get_parameter("smooth_stream").value):
                    # arm is on Time-based Profile: units are ms, so leave a
                    # gentle 400 ms move profile instead of the velocity-based
                    # "slow" values (30/10 would mean a violent 30 ms snap)
                    self._set_profile_ids(self._arm_ids(), 400, 150)
                else:
                    self._set_profile(30, 10)
            except Exception:
                pass
        finally:
            try:
                if self.port is not None:
                    self.port.closePort()
            except Exception:
                pass
            return super().destroy_node()


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node: Optional[TeleopNode] = None
    try:
        node = TeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
