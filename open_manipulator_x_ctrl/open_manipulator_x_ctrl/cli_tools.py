"""
Small standalone CLI tools that DON'T require the running node.

They talk directly to the U2D2. Useful for emergency release/recovery.
"""

from __future__ import annotations

import argparse
import sys
import time

from .dxl_driver import DxlBus, default_om_x_motors
from .kinematics import POSE_HOME_UP


def _make_bus(args) -> DxlBus:
    motors = default_om_x_motors(include_gripper=not args.no_gripper)
    bus = DxlBus(args.device, args.baudrate, motors, logger=None)
    bus.open()
    return bus


def _common_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--device", default="/dev/ttyUSB0")
    p.add_argument("--baudrate", type=int, default=1000000)
    p.add_argument("--no-gripper", action="store_true")
    return p


def disable_torque(argv=None) -> int:
    """`ros2 run open_manipulator_x_ctrl disable_torque` — drops the arm gently
    BY YOU first: this just turns off the torque, hold the arm if it's elevated.
    """
    parser = _common_parser()
    args = parser.parse_args(argv)
    bus = _make_bus(args)
    try:
        print("Pinging...")
        print(bus.ping_all())
        print("WARNING: torque will turn OFF. Support the arm if it is raised. "
              "Continuing in 3 s — Ctrl-C to abort.")
        time.sleep(3.0)
        bus.set_torque(False)
        print("Torque OFF.")
    finally:
        bus.close()
    return 0


def go_home(argv=None) -> int:
    """`ros2 run open_manipulator_x_ctrl go_home` — gentle torque-on then move
    slowly to POSE_HOME_UP. Useful as a one-shot before launching the node.
    """
    parser = _common_parser()
    parser.add_argument("--slow-vel", type=int, default=20)
    parser.add_argument("--slow-acc", type=int, default=8)
    args = parser.parse_args(argv)
    bus = _make_bus(args)
    try:
        print("Gentle torque-on...")
        bus.gentle_torque_on(slow_vel=args.slow_vel, slow_acc=args.slow_acc,
                             run_vel=args.slow_vel, run_acc=args.slow_acc,
                             settle_s=0.6)
        ids = {m.name: m.dxl_id for m in bus.motors}
        goals = {
            ids["joint1"]: POSE_HOME_UP[0],
            ids["joint2"]: POSE_HOME_UP[1],
            ids["joint3"]: POSE_HOME_UP[2],
            ids["joint4"]: POSE_HOME_UP[3],
        }
        print(f"Commanding HOME_UP joints (rad): {POSE_HOME_UP}")
        bus.write_goal_positions_rad(goals)
        print("Goal written. Watching motion for ~6 s...")
        time.sleep(6.0)
        positions = bus.read_positions_rad()
        print(f"Final positions (rad): {positions}")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "go_home"
    rest = sys.argv[2:]
    if cmd == "go_home":
        sys.exit(go_home(rest))
    elif cmd == "disable_torque":
        sys.exit(disable_torque(rest))
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(2)
