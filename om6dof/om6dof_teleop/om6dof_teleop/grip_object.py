"""Close the OM6DOF gripper and detect an object from motor current.

The Dynamixel bus is owned exclusively by ``om6dof_bringup``.  This utility
uses only standard ROS 2 interfaces:

* ``/gripper_controller/gripper_cmd`` (``control_msgs/GripperCommand``) for
  position commands; and
* ``/joint_states`` for the measured gripper position and Present Current.

The bringup maps Dynamixel ``Present Current`` to the joint's ``effort`` state
and overrides the gripper scale so that value is already expressed in mA.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


@dataclass
class GripperSample:
    position: float
    current_effort: float
    stamp_monotonic: float
    sequence: int

    @property
    def valid(self) -> bool:
        return math.isfinite(self.position) and math.isfinite(self.current_effort)


class GripError(RuntimeError):
    """Raised when the ROS gripper interface cannot complete a grasp."""


class GripperClient(Node):
    def __init__(
        self,
        joint_name: str,
        joint_states_topic: str,
        action_name: str,
        current_unit_ma: float,
        verbose: bool,
    ) -> None:
        super().__init__("om6dof_grip_object")
        self.joint_name = joint_name
        self.current_unit_ma = current_unit_ma
        self.verbose = verbose
        self.sample: Optional[GripperSample] = None
        self._sample_sequence = 0

        self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(self, GripperCommand, action_name)

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return

        # A current measurement is mandatory for this utility.  Do not silently
        # treat a missing effort array as zero because that would report an
        # obstructed or disconnected gripper as an empty grasp.
        if index >= len(msg.position) or index >= len(msg.effort):
            return

        self._sample_sequence += 1
        self.sample = GripperSample(
            position=float(msg.position[index]),
            current_effort=float(msg.effort[index]),
            stamp_monotonic=time.monotonic(),
            sequence=self._sample_sequence,
        )

    def current_ma(self, sample: GripperSample) -> float:
        return abs(sample.current_effort) * self.current_unit_ma

    def wait_for_interfaces(self, timeout: float) -> GripperSample:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.action.server_is_ready() and self.sample is not None and self.sample.valid:
                return self.sample
            rclpy.spin_once(self, timeout_sec=0.05)

        missing = []
        if not self.action.server_is_ready():
            missing.append("GripperCommand action server")
        if self.sample is None or not self.sample.valid:
            missing.append(f"{self.joint_name} position/effort in /joint_states")
        raise GripError("timed out waiting for " + " and ".join(missing))

    def wait_future(self, future: Future, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise GripError(f"timed out waiting for {description}")
        exception = future.exception()
        if exception is not None:
            raise GripError(f"{description} failed: {exception}")
        return future.result()

    def send_goal(self, position: float, max_effort: float, timeout: float):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        handle = self.wait_future(
            self.action.send_goal_async(goal), timeout, "gripper goal acceptance"
        )
        if handle is None or not handle.accepted:
            raise GripError("gripper controller rejected the goal")
        return handle

    def hold_position(self, position: float, max_effort: float, timeout: float) -> None:
        # Sending the hold goal also preempts a still-running close goal.  The
        # returned goal handle only confirms acceptance; controller_manager
        # remains alive after this short-lived utility exits and keeps holding.
        self.send_goal(position, max_effort, timeout)

    def log_sample(self, sample: GripperSample, over: int) -> None:
        if self.verbose:
            self.get_logger().info(
                f"position={sample.position:+.5f} m, "
                f"current={self.current_ma(sample):6.1f} mA, over={over}"
            )


def grasp(
    node: GripperClient,
    close_target: float,
    current_threshold_ma: float,
    consecutive: int,
    settle_current_ma: float,
    warmup_seconds: float,
    settle_seconds: float,
    motion_timeout: float,
    state_timeout: float,
    max_effort: float,
) -> dict:
    """Command one close motion and stop it when sustained current is detected."""
    initial = node.wait_for_interfaces(state_timeout)
    node.get_logger().info(
        f"grasp start: {initial.position:+.5f} m -> {close_target:+.5f} m; "
        f"threshold={current_threshold_ma:.0f} mA"
    )
    close_handle = node.send_goal(close_target, max_effort, state_timeout)
    result_future = close_handle.get_result_async()

    started = time.monotonic()
    deadline = started + motion_timeout
    last_sequence = initial.sequence
    over = 0
    peak_ma = 0.0

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        sample = node.sample
        if sample is None or sample.sequence == last_sequence:
            continue
        last_sequence = sample.sequence
        if time.monotonic() - sample.stamp_monotonic > state_timeout:
            raise GripError("gripper joint state became stale")

        current_ma = node.current_ma(sample)
        peak_ma = max(peak_ma, current_ma)
        if time.monotonic() - started >= warmup_seconds and current_ma > current_threshold_ma:
            over += 1
        else:
            over = 0
        node.log_sample(sample, over)

        if over >= consecutive:
            cancel = close_handle.cancel_goal_async()
            # Cancellation is best effort.  The subsequent accepted hold goal
            # is what deterministically preempts the close command.
            try:
                node.wait_future(cancel, state_timeout, "close-goal cancellation")
            except GripError as exc:
                node.get_logger().warning(str(exc))
            node.hold_position(sample.position, max_effort, state_timeout)
            return {
                "object": True,
                "position": sample.position,
                "current": current_ma,
                "peak": peak_ma,
                "reason": (
                    f"current {current_ma:.0f} mA > {current_threshold_ma:.0f} mA "
                    f"for {consecutive} samples"
                ),
            }

        if result_future.done():
            break

    if not result_future.done():
        close_handle.cancel_goal_async()
        sample = node.sample
        if sample is not None:
            node.hold_position(sample.position, max_effort, state_timeout)
        raise GripError(f"gripper did not finish within {motion_timeout:.1f} s")

    result = node.wait_future(result_future, state_timeout, "gripper result")
    if result is None:
        raise GripError("gripper action returned no result")

    # Continue sampling briefly after the action reaches its target.  Current
    # at rest distinguishes an empty fully-closed jaw from an object that sits
    # at the mechanical close position.
    settle_deadline = time.monotonic() + settle_seconds
    final = node.sample
    while rclpy.ok() and time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, settle_deadline - time.monotonic()))
        if node.sample is not None:
            final = node.sample
            peak_ma = max(peak_ma, node.current_ma(final))
    if final is None or time.monotonic() - final.stamp_monotonic > state_timeout:
        raise GripError("no fresh gripper state after action completion")

    current_ma = node.current_ma(final)
    object_detected = current_ma > settle_current_ma
    if object_detected:
        node.hold_position(final.position, max_effort, state_timeout)
    return {
        "object": object_detected,
        "position": final.position,
        "current": current_ma,
        "peak": peak_ma,
        "reason": (
            f"settled current {current_ma:.0f} mA > {settle_current_ma:.0f} mA"
            if object_detected
            else "reached close target with low current; jaw is empty"
        ),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", default="gripper_left_joint")
    parser.add_argument("--joint-states", default="/joint_states")
    parser.add_argument("--action", default="/gripper_controller/gripper_cmd")
    parser.add_argument("--close-target", type=float, default=-0.010,
                        help="fully closed prismatic position in metres")
    parser.add_argument("--current-threshold", type=float, default=120.0,
                        help="sustained Present Current threshold in mA")
    parser.add_argument("--settle-current", type=float, default=60.0,
                        help="current at the close target that still means object")
    parser.add_argument("--current-unit-ma", type=float, default=1.0,
                        help="mA per effort unit (default 1: bringup exports mA)")
    parser.add_argument("--consecutive", type=int, default=3)
    parser.add_argument("--warmup-seconds", type=float, default=0.10)
    parser.add_argument("--settle-seconds", type=float, default=0.15)
    parser.add_argument("--motion-timeout", type=float, default=10.0)
    parser.add_argument("--state-timeout", type=float, default=2.0)
    parser.add_argument("--max-effort", type=float, default=0.0,
                        help="GripperCommand max_effort (0 lets the controller use its limit)")
    parser.add_argument("--verbose", "-v", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    ros_argv = list(sys.argv if argv is None else argv)
    cli_argv = remove_ros_args(args=ros_argv)[1:] if argv is None else list(argv)
    args = _parser().parse_args(cli_argv)
    if args.current_unit_ma <= 0.0 or args.consecutive < 1:
        _parser().error("current-unit-ma must be positive and consecutive must be >= 1")

    rclpy.init(args=ros_argv if argv is None else None)
    node = GripperClient(
        args.joint,
        args.joint_states,
        args.action,
        args.current_unit_ma,
        args.verbose,
    )
    try:
        result = grasp(
            node,
            close_target=args.close_target,
            current_threshold_ma=args.current_threshold,
            consecutive=args.consecutive,
            settle_current_ma=args.settle_current,
            warmup_seconds=args.warmup_seconds,
            settle_seconds=args.settle_seconds,
            motion_timeout=args.motion_timeout,
            state_timeout=args.state_timeout,
            max_effort=args.max_effort,
        )
        label = "OBJECT GRASPED" if result["object"] else "NO OBJECT"
        print(
            f"{label}: position={result['position']:+.5f} m, "
            f"current={result['current']:.0f} mA, peak={result['peak']:.0f} mA "
            f"({result['reason']})",
            flush=True,
        )
        return 0 if result["object"] else 2
    except GripError as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
