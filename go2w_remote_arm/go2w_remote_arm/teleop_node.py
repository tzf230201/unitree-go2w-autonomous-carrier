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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import WirelessController
from sensor_msgs.msg import JointState

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
)


# ---------- Dynamixel XM430 / XL430 control table (Protocol 2.0) ----------
ADDR_OPERATING_MODE = 11        # 1 byte
ADDR_TORQUE_ENABLE = 64         # 1 byte
ADDR_HARDWARE_ERROR = 70        # 1 byte
ADDR_PROFILE_ACCEL = 108        # 4 bytes
ADDR_PROFILE_VELOCITY = 112     # 4 bytes
ADDR_GOAL_POSITION = 116        # 4 bytes
ADDR_PRESENT_POSITION = 132     # 4 bytes
LEN_GOAL = 4
LEN_PRESENT = 4
LEN_PROFILE = 4

OP_MODE_POSITION = 3
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


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_remote_arm")

        # ---- parameters ----
        self.declare_parameter("device", "/dev/ttyUSB0")
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

        # gentle startup profile
        self.declare_parameter("startup_slow_vel", 30)
        self.declare_parameter("startup_slow_acc", 10)
        self.declare_parameter("run_vel", 100)
        self.declare_parameter("run_acc", 30)
        self.declare_parameter("settle_seconds", 0.5)

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

        # ---- bus init ----
        self.port = PortHandler(self.device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise IOError(f"failed to open {self.device}")
        if not self.port.setBaudRate(self.baud):
            raise IOError(f"failed to set baud {self.baud}")

        self.sync_read = GroupSyncRead(
            self.port, self.packet, ADDR_PRESENT_POSITION, LEN_PRESENT
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

        # ping
        self.get_logger().info(f"Pinging {self.motor_ids}...")
        for mid in self.motor_ids:
            _, rc, err = self.packet.ping(self.port, mid)
            if rc != COMM_SUCCESS or err != 0:
                raise RuntimeError(
                    f"ping ID {mid} failed: rc={rc} err={err} "
                    f"({self.packet.getTxRxResult(rc)})"
                )
        self.get_logger().info("all motors responded.")

        # auto-clear stale Hardware Error flags via Reboot
        self._clear_hw_errors_via_reboot()

        # gentle torque-on
        present = self._gentle_torque_on(
            int(self.get_parameter("startup_slow_vel").value),
            int(self.get_parameter("startup_slow_acc").value),
            int(self.get_parameter("run_vel").value),
            int(self.get_parameter("run_acc").value),
            float(self.get_parameter("settle_seconds").value),
        )

        # state — goals are in joint-frame radians, indexed same as motor_ids
        self.goal_rad: List[float] = list(present)
        self.lock = threading.Lock()
        self.keys = 0
        self.prev_keys = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry
        self.last_remote_t = 0.0

        # ---- ROS io ----
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            WirelessController, "/wirelesscontroller", self._on_remote, qos_be
        )
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10)
        self.dt = 1.0 / self.publish_rate
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"go2w_remote_arm ready: {self.n_total} motors, "
            f"vel={self.joint_velocity} rad/s, rate={self.publish_rate} Hz"
        )

    # ---------------- low-level helpers ----------------
    def _write1(self, dxl_id: int, addr: int, value: int) -> None:
        rc, err = self.packet.write1ByteTxRx(self.port, dxl_id, addr, value & 0xFF)
        if rc != COMM_SUCCESS or err != 0:
            self.get_logger().warn(
                f"write1 id={dxl_id} addr={addr}: rc={rc} err={err}"
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

    def _write_goals_rad(self, goals: List[float]) -> None:
        self.sync_write_goal.clearParam()
        for mid, r in zip(self.motor_ids, goals):
            self.sync_write_goal.addParam(mid, _u32(rad_to_ticks(r)))
        rc = self.sync_write_goal.txPacket()
        if rc != COMM_SUCCESS:
            self.get_logger().warn(
                f"sync write goal: {self.packet.getTxRxResult(rc)}"
            )

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
        with self.lock:
            self.prev_keys = self.keys
            self.keys = int(msg.keys)
            self.axes = (
                float(msg.lx), float(msg.ly), float(msg.rx), float(msg.ry)
            )
            self.last_remote_t = time.monotonic()

            # edge-triggered gripper
            rising = (~self.prev_keys) & self.keys
            if rising & (1 << self.grip_btn_open):
                self.goal_rad[-1] = self._clamp_grip(self.grip_open)
                self.get_logger().info(
                    f"gripper OPEN → {self.goal_rad[-1]:+.2f}"
                )
            if rising & (1 << self.grip_btn_close):
                self.goal_rad[-1] = self._clamp_grip(self.grip_close)
                self.get_logger().info(
                    f"gripper CLOSE → {self.goal_rad[-1]:+.2f}"
                )

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

    # ---------------- main loop ----------------
    def _tick(self) -> None:
        with self.lock:
            keys = self.keys
            axes = self.axes
            stale = (
                (time.monotonic() - self.last_remote_t) > 0.5
                if self.last_remote_t
                else True
            )
            if not stale:
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

        # write goals every tick so motors actively track even without input
        try:
            self._write_goals_rad(self.goal_rad)
        except Exception as exc:
            self.get_logger().warn(f"write goals: {exc}")

        # publish JointState
        try:
            positions = self._read_positions_rad()
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = list(self.joint_names)
            m.position = positions
            self.pub_state.publish(m)
        except Exception as exc:
            self.get_logger().warn(
                f"read positions: {exc}", throttle_duration_sec=2.0
            )

    # ---------------- lifecycle ----------------
    def destroy_node(self):
        try:
            self.get_logger().info(
                "Shutdown — leaving torque enabled. Use dynamixel wizard to "
                "disable if needed."
            )
            try:
                self._set_profile(30, 10)
            except Exception:
                pass
        finally:
            try:
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
