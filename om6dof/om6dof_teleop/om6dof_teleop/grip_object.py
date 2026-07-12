"""Dedicated grasp routine for the OpenManipulator Chain gripper (ID 37).

Slowly closes the gripper toward the CLOSE target while watching the motor's
Present Current. As soon as the current stays above a threshold for a few
consecutive reads — i.e. the jaws pressed onto something — motion stops and the
gripper HOLDS at that position (a soft, current-limited grasp). If the target
is reached with no current spike, the jaws are empty and simply closed fully.

This talks to the Dynamixel bus directly (dynamixel_sdk, Protocol 2.0), the
same way teleop_node does. It therefore needs EXCLUSIVE access to the serial
port: stop om6dof_teleop teleop before running this, or they will fight over
/dev/ttyUSB0.

Run standalone:
    python3 -m om6dof_teleop.grip_object
or via the installed console script:
    ros2 run om6dof_teleop grip_object -- --current-threshold 120

Detection is torque-based (Present Current, addr 126). No extra sensor needed —
an empty jaw reaches CLOSE freely (low current); a jaw pressing on an object
stalls before CLOSE and current rises.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


# ---------- XM430 control table (Protocol 2.0) ----------
ADDR_OPERATING_MODE = 11        # 1 byte
ADDR_TORQUE_ENABLE = 64         # 1 byte
ADDR_PROFILE_ACCEL = 108        # 4 bytes
ADDR_PROFILE_VELOCITY = 112     # 4 bytes
ADDR_GOAL_POSITION = 116        # 4 bytes
ADDR_PRESENT_CURRENT = 126      # 2 bytes, signed
ADDR_PRESENT_POSITION = 132     # 4 bytes, signed

OP_MODE_POSITION = 3
PROTOCOL_VERSION = 2.0
TICKS_PER_REV = 4096
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV
# XM430-W350 current unit: 2.69 mA per raw tick.
CURRENT_UNIT_MA = 2.69


def rad_to_ticks(r: float) -> int:
    return int(round(2048 + r / TICK_RAD))


def ticks_to_rad(t: int) -> float:
    return (t - 2048) * TICK_RAD


def _s16(v: int) -> int:
    return v - (1 << 16) if v >= (1 << 15) else v


def _s32(v: int) -> int:
    return v - (1 << 32) if v >= (1 << 31) else v


class GripError(RuntimeError):
    pass


class Gripper:
    def __init__(self, device: str, baud: int, motor_id: int) -> None:
        self.motor_id = motor_id
        self.port = PortHandler(device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise GripError(f"failed to open {device}")
        if not self.port.setBaudRate(baud):
            raise GripError(f"failed to set baud {baud}")
        _, rc, err = self.packet.ping(self.port, motor_id)
        if rc != COMM_SUCCESS or err != 0:
            raise GripError(
                f"gripper ID {motor_id} did not respond "
                f"(rc={rc} err={err}); check 12V power / U2D2 / daisy chain"
            )

    # ---- low-level ----
    def _w1(self, addr: int, val: int) -> None:
        rc, err = self.packet.write1ByteTxRx(self.port, self.motor_id, addr, val & 0xFF)
        if rc != COMM_SUCCESS or err != 0:
            raise GripError(f"write1 addr={addr}: rc={rc} err={err}")

    def _w4(self, addr: int, val: int) -> None:
        rc, err = self.packet.write4ByteTxRx(
            self.port, self.motor_id, addr, val & 0xFFFFFFFF
        )
        if rc != COMM_SUCCESS or err != 0:
            raise GripError(f"write4 addr={addr}: rc={rc} err={err}")

    def present_position_rad(self) -> float:
        raw, rc, err = self.packet.read4ByteTxRx(
            self.port, self.motor_id, ADDR_PRESENT_POSITION
        )
        if rc != COMM_SUCCESS or err != 0:
            raise GripError(f"read position: rc={rc} err={err}")
        return ticks_to_rad(_s32(raw))

    def present_current_ma(self) -> float:
        raw, rc, err = self.packet.read2ByteTxRx(
            self.port, self.motor_id, ADDR_PRESENT_CURRENT
        )
        if rc != COMM_SUCCESS or err != 0:
            raise GripError(f"read current: rc={rc} err={err}")
        return _s16(raw) * CURRENT_UNIT_MA

    def write_goal_rad(self, r: float) -> None:
        self._w4(ADDR_GOAL_POSITION, rad_to_ticks(r) & 0xFFFFFFFF)

    # ---- setup ----
    def prepare(self, profile_vel: int, profile_acc: int) -> None:
        """Position mode, slow profile, torque on, goal preloaded to present so
        the jaw does not snap when torque engages."""
        self._w1(ADDR_TORQUE_ENABLE, 0)
        self._w1(ADDR_OPERATING_MODE, OP_MODE_POSITION)
        self._w4(ADDR_PROFILE_VELOCITY, profile_vel)
        self._w4(ADDR_PROFILE_ACCEL, profile_acc)
        self.write_goal_rad(self.present_position_rad())
        self._w1(ADDR_TORQUE_ENABLE, 1)

    def close(self) -> None:
        try:
            self.port.closePort()
        except Exception:
            pass


def grasp(
    g: Gripper,
    close_target: float,
    step_rad: float,
    rate_hz: float,
    current_threshold_ma: float,
    consecutive: int,
    settle_ma: float,
    warmup_ticks: int,
    log,
) -> dict:
    """Step the gripper toward `close_target`, monitoring current.

    Returns a dict describing the outcome:
        {"object": bool, "position": rad, "current": mA, "reason": str}
    """
    dt = 1.0 / rate_hz
    start = g.present_position_rad()
    # closing direction: sign that moves `start` toward `close_target`
    direction = 1.0 if close_target >= start else -1.0
    cmd = start
    over = 0
    peak = 0.0
    tick = 0

    log(
        f"grasp start: pos={start:+.3f} rad → target={close_target:+.3f} rad "
        f"(dir={'+' if direction > 0 else '-'}, "
        f"threshold={current_threshold_ma:.0f} mA)"
    )

    while True:
        # advance commanded goal one step toward the target, without overshoot
        remaining = close_target - cmd
        if abs(remaining) <= step_rad:
            cmd = close_target
            reached = True
        else:
            cmd += direction * step_rad
            reached = False
        g.write_goal_rad(cmd)

        time.sleep(dt)
        tick += 1

        pos = g.present_position_rad()
        cur = abs(g.present_current_ma())
        peak = max(peak, cur)

        # ignore the initial acceleration current spike
        if tick > warmup_ticks and cur > current_threshold_ma:
            over += 1
        else:
            over = 0

        log(
            f"  t={tick:3d} cmd={cmd:+.3f} pos={pos:+.3f} "
            f"cur={cur:6.1f} mA over={over}",
            debug=True,
        )

        if over >= consecutive:
            # object is resisting the jaws — hold right here
            g.write_goal_rad(pos)
            return {
                "object": True,
                "position": pos,
                "current": cur,
                "reason": f"current {cur:.0f} mA > {current_threshold_ma:.0f} mA "
                          f"for {consecutive} reads",
            }

        if reached:
            # commanded goal is at target; wait for the jaw to settle, then
            # decide: low current + at target = empty (fully closed).
            time.sleep(0.15)
            cur = abs(g.present_current_ma())
            pos = g.present_position_rad()
            if cur > settle_ma:
                g.write_goal_rad(pos)
                return {
                    "object": True,
                    "position": pos,
                    "current": cur,
                    "reason": f"reached target but holding {cur:.0f} mA "
                              f"(> {settle_ma:.0f} mA settle) — object at full close",
                }
            return {
                "object": False,
                "position": pos,
                "current": cur,
                "reason": "reached CLOSE target with low current — jaw empty",
            }


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--id", type=int, default=37, help="gripper Dynamixel ID")
    p.add_argument("--close-target", type=float, default=0.0,
                   help="fully-closed jaw position (rad)")
    p.add_argument("--step", type=float, default=0.004,
                   help="rad advanced per control tick (smaller = slower/gentler)")
    p.add_argument("--rate", type=float, default=50.0, help="control loop Hz")
    p.add_argument("--current-threshold", type=float, default=120.0,
                   help="present current (mA) that counts as 'holding an object'")
    p.add_argument("--consecutive", type=int, default=3,
                   help="consecutive over-threshold reads before declaring a grasp")
    p.add_argument("--settle-current", type=float, default=60.0,
                   help="mA at full close that still counts as an object")
    p.add_argument("--profile-vel", type=int, default=30,
                   help="Dynamixel Profile Velocity (caps jaw speed)")
    p.add_argument("--profile-acc", type=int, default=10)
    p.add_argument("--warmup-ticks", type=int, default=5,
                   help="ticks to ignore before current is trusted")
    p.add_argument("--release-torque", action="store_true",
                   help="disable torque on exit instead of holding the grasp")
    p.add_argument("--verbose", "-v", action="store_true",
                   help="print every control tick")
    args = p.parse_args(argv)

    def log(msg: str, debug: bool = False) -> None:
        if debug and not args.verbose:
            return
        print(msg, flush=True)

    g: Optional[Gripper] = None
    try:
        g = Gripper(args.device, args.baud, args.id)
        g.prepare(args.profile_vel, args.profile_acc)
        result = grasp(
            g,
            close_target=args.close_target,
            step_rad=args.step,
            rate_hz=args.rate,
            current_threshold_ma=args.current_threshold,
            consecutive=args.consecutive,
            settle_ma=args.settle_current,
            warmup_ticks=args.warmup_ticks,
            log=log,
        )
        if result["object"]:
            print(
                f"\n✅ OBJECT GRASPED — holding at {result['position']:+.3f} rad, "
                f"{result['current']:.0f} mA\n   ({result['reason']})",
                flush=True,
            )
        else:
            print(
                f"\n⚪ NO OBJECT — jaw closed to {result['position']:+.3f} rad, "
                f"{result['current']:.0f} mA\n   ({result['reason']})",
                flush=True,
            )

        if args.release_torque:
            g._w1(ADDR_TORQUE_ENABLE, 0)
            print("torque released.", flush=True)
        else:
            print("holding grasp (torque stays ON). Ctrl-C to exit.", flush=True)

        return 0 if result["object"] else 2
    except GripError as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if g is not None:
            g.close()


if __name__ == "__main__":
    raise SystemExit(main())
