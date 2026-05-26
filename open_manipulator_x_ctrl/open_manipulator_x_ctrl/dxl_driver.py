"""
Low-level Dynamixel driver for OpenManipulator-X.

Motors: XM430-W350-T (joints 1-4) and XM430 / XL430 for gripper (we treat as XM430).
Protocol 2.0. Default baud 1000000.

This module wraps Sync Read / Sync Write of common control-table registers and
provides a *gentle* torque-enable routine: it first reads each motor's current
position, sets that as the goal, lowers the profile velocity/acceleration so the
motor cannot snap, then enables torque. Only after torque is on do we ramp the
profile back up.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
)


# ----- XM430 / XL430 Control Table (Protocol 2.0) -----
ADDR_OPERATING_MODE = 11        # 1 byte
ADDR_HOMING_OFFSET = 20         # 4 bytes
ADDR_TORQUE_ENABLE = 64         # 1 byte
ADDR_LED = 65                   # 1 byte
ADDR_HARDWARE_ERROR = 70        # 1 byte
ADDR_POSITION_P = 84            # 2 bytes
ADDR_POSITION_I = 82            # 2 bytes
ADDR_POSITION_D = 80            # 2 bytes
ADDR_PROFILE_ACCEL = 108        # 4 bytes
ADDR_PROFILE_VEL = 112          # 4 bytes
ADDR_GOAL_POSITION = 116        # 4 bytes
ADDR_PRESENT_CURRENT = 126      # 2 bytes
ADDR_PRESENT_VELOCITY = 128     # 4 bytes
ADDR_PRESENT_POSITION = 132     # 4 bytes
ADDR_MOVING = 122               # 1 byte

LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
LEN_PROFILE_VEL = 4
LEN_PROFILE_ACCEL = 4

OPERATING_MODE_POSITION = 3
PROTOCOL_VERSION = 2.0

# Conversion: 4096 ticks / full rev. Center = 2048 (= 0 rad in our convention).
TICKS_PER_REV = 4096
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV


def rad_to_ticks(rad: float) -> int:
    """Joint angle (rad) → Dynamixel position tick. 0 rad = 2048."""
    return int(round(2048 + rad / TICK_RAD))


def ticks_to_rad(ticks: int) -> float:
    return (ticks - 2048) * TICK_RAD


def _u32_to_bytes(v: int) -> List[int]:
    v &= 0xFFFFFFFF
    return [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]


def _bytes_to_s32(b: int) -> int:
    return b - (1 << 32) if b >= (1 << 31) else b


@dataclass
class MotorConfig:
    name: str
    dxl_id: int
    # joint angle limits (rad). Conservative defaults for OM-X.
    min_rad: float = -math.pi
    max_rad: float = math.pi
    # offset applied: motor_rad = joint_rad * direction + offset
    direction: int = 1     # +1 or -1
    offset_rad: float = 0.0


class DxlBus:
    """Single U2D2 bus serving N motors."""

    def __init__(
        self,
        device: str,
        baudrate: int,
        motors: List[MotorConfig],
        logger=None,
    ):
        self.device = device
        self.baudrate = baudrate
        self.motors = motors
        self._by_id = {m.dxl_id: m for m in motors}
        self._log = logger

        self.port = PortHandler(device)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        self._sync_read_pos = None
        self._sync_write_goal = None
        self._sync_write_pvel = None
        self._sync_write_pacc = None

    # ---------------- lifecycle ----------------
    def open(self) -> None:
        if not self.port.openPort():
            raise IOError(f"Failed to open port {self.device}")
        if not self.port.setBaudRate(self.baudrate):
            raise IOError(f"Failed to set baud {self.baudrate} on {self.device}")

        self._sync_read_pos = GroupSyncRead(
            self.port, self.packet, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        self._sync_write_goal = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )
        self._sync_write_pvel = GroupSyncWrite(
            self.port, self.packet, ADDR_PROFILE_VEL, LEN_PROFILE_VEL
        )
        self._sync_write_pacc = GroupSyncWrite(
            self.port, self.packet, ADDR_PROFILE_ACCEL, LEN_PROFILE_ACCEL
        )

        for m in self.motors:
            ok = self._sync_read_pos.addParam(m.dxl_id)
            if not ok:
                raise IOError(f"SyncRead addParam failed for id {m.dxl_id}")

    def close(self) -> None:
        try:
            self.port.closePort()
        except Exception:
            pass

    # ---------------- low-level write helpers ----------------
    def _write1(self, dxl_id: int, addr: int, value: int) -> None:
        rc, err = self.packet.write1ByteTxRx(self.port, dxl_id, addr, value & 0xFF)
        self._check(rc, err, f"write1 id={dxl_id} addr={addr}")

    def _write2(self, dxl_id: int, addr: int, value: int) -> None:
        rc, err = self.packet.write2ByteTxRx(self.port, dxl_id, addr, value & 0xFFFF)
        self._check(rc, err, f"write2 id={dxl_id} addr={addr}")

    def _write4(self, dxl_id: int, addr: int, value: int) -> None:
        rc, err = self.packet.write4ByteTxRx(self.port, dxl_id, addr, value & 0xFFFFFFFF)
        self._check(rc, err, f"write4 id={dxl_id} addr={addr}")

    def _read4(self, dxl_id: int, addr: int) -> int:
        v, rc, err = self.packet.read4ByteTxRx(self.port, dxl_id, addr)
        self._check(rc, err, f"read4 id={dxl_id} addr={addr}")
        return v

    def _check(self, rc: int, err: int, where: str) -> None:
        if rc != COMM_SUCCESS:
            raise IOError(f"{where}: comm err {self.packet.getTxRxResult(rc)}")
        if err != 0:
            raise IOError(f"{where}: dxl err {self.packet.getRxPacketError(err)}")

    # ---------------- ping / status ----------------
    def ping_all(self) -> Dict[int, bool]:
        out = {}
        for m in self.motors:
            _, rc, err = self.packet.ping(self.port, m.dxl_id)
            out[m.dxl_id] = (rc == COMM_SUCCESS and err == 0)
        return out

    def read_hw_errors(self) -> Dict[int, int]:
        out = {}
        for m in self.motors:
            v, rc, err = self.packet.read1ByteTxRx(self.port, m.dxl_id, ADDR_HARDWARE_ERROR)
            out[m.dxl_id] = v if (rc == COMM_SUCCESS and err == 0) else -1
        return out

    # ---------------- bulk reads ----------------
    def read_positions_rad(self) -> Dict[int, float]:
        """Return {dxl_id: joint angle in rad (after direction/offset)}."""
        rc = self._sync_read_pos.txRxPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"SyncRead present pos: {self.packet.getTxRxResult(rc)}")
        out = {}
        for m in self.motors:
            if not self._sync_read_pos.isAvailable(
                m.dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise IOError(f"SyncRead: id {m.dxl_id} unavailable")
            raw = self._sync_read_pos.getData(
                m.dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            raw_s = _bytes_to_s32(raw)
            motor_rad = ticks_to_rad(raw_s)
            joint_rad = (motor_rad - m.offset_rad) * m.direction
            out[m.dxl_id] = joint_rad
        return out

    # ---------------- configuration ----------------
    def set_operating_mode_position(self) -> None:
        """Must be called with torque OFF on every motor."""
        for m in self.motors:
            self._write1(m.dxl_id, ADDR_TORQUE_ENABLE, 0)
            self._write1(m.dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION)

    def set_profile(self, vel_ticks: int, acc_ticks: int) -> None:
        """Set Profile Velocity & Acceleration for all motors via sync write.

        Units (XM430, time-based profile = 0): Profile Velocity in units of
        0.229 rev/min, Profile Acceleration in units of 214.577 rev/min^2.
        0 = unlimited (do NOT use during startup!).
        """
        self._sync_write_pvel.clearParam()
        self._sync_write_pacc.clearParam()
        for m in self.motors:
            self._sync_write_pvel.addParam(m.dxl_id, bytes(_u32_to_bytes(vel_ticks)))
            self._sync_write_pacc.addParam(m.dxl_id, bytes(_u32_to_bytes(acc_ticks)))
        rc = self._sync_write_pvel.txPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"sync write prof vel: {self.packet.getTxRxResult(rc)}")
        rc = self._sync_write_pacc.txPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"sync write prof acc: {self.packet.getTxRxResult(rc)}")

    def set_torque(self, on: bool) -> None:
        v = 1 if on else 0
        for m in self.motors:
            self._write1(m.dxl_id, ADDR_TORQUE_ENABLE, v)

    # ---------------- motion ----------------
    def write_goal_positions_rad(self, joints: Dict[int, float]) -> None:
        """Write goal positions by dxl_id (joint angles in rad)."""
        self._sync_write_goal.clearParam()
        for dxl_id, joint_rad in joints.items():
            m = self._by_id[dxl_id]
            jr = max(m.min_rad, min(m.max_rad, joint_rad))
            motor_rad = jr * m.direction + m.offset_rad
            ticks = rad_to_ticks(motor_rad)
            self._sync_write_goal.addParam(dxl_id, bytes(_u32_to_bytes(ticks)))
        rc = self._sync_write_goal.txPacket()
        if rc != COMM_SUCCESS:
            raise IOError(f"sync write goal: {self.packet.getTxRxResult(rc)}")

    # ---------------- gentle startup ----------------
    def gentle_torque_on(
        self,
        slow_vel: int = 30,
        slow_acc: int = 10,
        run_vel: int = 100,
        run_acc: int = 30,
        settle_s: float = 0.4,
    ) -> Dict[int, float]:
        """Safely bring up torque so the arm doesn't snap.

        Procedure:
          1. Ensure torque OFF, set Position mode.
          2. Read present positions (= where the arm physically is right now).
          3. Pre-load Goal Position = Present Position (no commanded motion).
          4. Set Profile Velocity / Accel to *very low* values.
          5. Enable torque. Motors lock to their current pose with a slow profile.
          6. After `settle_s`, ramp profile to operating values.

        Returns the joint angles (rad) read at startup so the caller can decide
        when/where to move next.
        """
        self._log_info("Gentle torque-on: disabling torque & forcing position mode")
        self.set_operating_mode_position()  # also writes torque=0

        # 1) read where we are
        present = self.read_positions_rad()
        self._log_info(f"Present joints (rad): {present}")

        # 2) preload goal = present so the first thing the controller sees is "no motion"
        self.write_goal_positions_rad(present)

        # 3) slow profile so even a small position error is moved through gently
        self._log_info(f"Setting slow profile vel={slow_vel} acc={slow_acc}")
        self.set_profile(slow_vel, slow_acc)

        # 4) torque on
        self._log_info("Torque ON")
        self.set_torque(True)

        time.sleep(settle_s)

        # 5) re-check for hardware errors
        errs = self.read_hw_errors()
        bad = {i: e for i, e in errs.items() if e and e > 0}
        if bad:
            self._log_warn(f"Hardware errors after torque-on: {bad}")

        # 6) ramp profile to running values
        self._log_info(f"Ramping profile to vel={run_vel} acc={run_acc}")
        self.set_profile(run_vel, run_acc)

        return present

    def safe_shutdown(self) -> None:
        """Lower profile, leave torque enabled (or off if caller wants). Caller
        decides whether to move to a safe pose first."""
        try:
            self.set_profile(30, 10)
        except Exception:
            pass

    # ---------------- logging shim ----------------
    def _log_info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warn(msg)


# ----- Default OpenManipulator-X configuration (ROBOTIS) -----
def default_om_x_motors(include_gripper: bool = True) -> List[MotorConfig]:
    """OM-X joint conventions:
    - joint1 (id 11): base yaw, +z = CCW from above. direction +1.
    - joint2 (id 12): shoulder pitch. Sign chosen so + lifts arm UP. The motor's
      raw sign is inverted relative to our convention, so direction = -1.
    - joint3 (id 13): elbow. direction +1 (folds arm).
    - joint4 (id 14): wrist pitch. direction +1.
    - id 15: gripper (linear-ish via parallel jaw). Treat as position joint.

    Limits are conservative software limits. They do NOT replace the EEPROM
    Min/Max position limit registers but provide a runtime safety net.
    """
    pi = math.pi
    motors = [
        MotorConfig("joint1", 11, min_rad=-pi,        max_rad=pi,        direction=+1),
        MotorConfig("joint2", 12, min_rad=-pi / 2,    max_rad=+pi / 2,   direction=-1),
        MotorConfig("joint3", 13, min_rad=-pi / 2,    max_rad=+pi / 2,   direction=+1),
        MotorConfig("joint4", 14, min_rad=-pi / 2,    max_rad=+pi / 2,   direction=+1),
    ]
    if include_gripper:
        # Gripper: allow full single-turn motor travel (~-π..+π). The GUI's
        # calibration picks actual open/close targets within this range; the
        # EEPROM Min/Max Position Limit registers on the Dynamixel still apply
        # as the ultimate hardware safety net.
        motors.append(MotorConfig("gripper", 15, min_rad=-pi, max_rad=+pi, direction=+1))
    return motors
