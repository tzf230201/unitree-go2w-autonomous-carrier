"""Pick-and-place state machine.

Default behavior:
  1. Move to `start_pose` (named SRDF state, e.g. "rest")
  2. Open gripper
  3. Move above the pick location (`pick_above`)
  4. Lower to pick location (`pick`)
  5. Close gripper (grasp)
  6. Lift back up (`pick_above`)
  7. Move above the place location (`place_above`)
  8. Lower (`place`)
  9. Open gripper (release)
 10. Lift back up (`place_above`)
 11. Return to `start_pose`

Each waypoint is defined in `waypoints.yaml` as either:
  * `joint: [j1, j2, j3, j4, j5, j6]`   — joint values, OR
  * `xyz: [x, y, z]` (+ optional `rpy: [r, p, y]`) — Cartesian pose

A waypoint named "start_pose" can be either a named pose or one of the above.

The node triggers a single run on startup and then idles. Re-trigger via the
`/run_pick_place` service (std_srvs/Trigger).
"""

from __future__ import annotations

import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .moveit_client import MoveItClient


ARM_JOINT_ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class PickPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("om_chain_pick_place")

        # ---- parameters ----
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("auto_run", True)
        self.declare_parameter("arm_group", "arm")
        self.declare_parameter("gripper_group", "gripper")
        self.declare_parameter("ee_link", "end_effector_link")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("vel_scale", 0.3)
        self.declare_parameter("acc_scale", 0.3)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("planning_attempts", 20)
        # gripper positions (prismatic, metres). Defaults match SRDF group_states.
        self.declare_parameter("gripper_open_pos", 0.019)
        self.declare_parameter("gripper_close_pos", -0.010)
        # SRDF path — defaults to the file shipped in om_chain_moveit_config.
        self.declare_parameter("srdf_file", "")

        # ---- waypoints ----
        wp_path = str(self.get_parameter("waypoints_file").value)
        if not wp_path:
            self.get_logger().error("Parameter 'waypoints_file' is required")
            raise RuntimeError("waypoints_file parameter not set")
        self.waypoints: Dict[str, Any] = self._load_waypoints(wp_path)
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {wp_path}: "
            f"{list(self.waypoints.keys())}"
        )

        # ---- moveit client ----
        self.client = MoveItClient(
            self,
            arm_group=str(self.get_parameter("arm_group").value),
            gripper_group=str(self.get_parameter("gripper_group").value),
            ee_link=str(self.get_parameter("ee_link").value),
            reference_frame=str(self.get_parameter("reference_frame").value),
            max_velocity_scaling=float(self.get_parameter("vel_scale").value),
            max_acceleration_scaling=float(self.get_parameter("acc_scale").value),
            planning_time=float(self.get_parameter("planning_time").value),
            num_planning_attempts=int(self.get_parameter("planning_attempts").value),
        )
        self.grip_open = float(self.get_parameter("gripper_open_pos").value)
        self.grip_close = float(self.get_parameter("gripper_close_pos").value)

        # ---- SRDF named-pose dict, loaded from file ----
        self.named_poses: Dict[str, Dict[str, float]] = {}
        srdf_path = str(self.get_parameter("srdf_file").value)
        if not srdf_path:
            # canonical SRDF = the open_manipulator_6dof one (tip at the
            # gripper). om_chain_moveit_config's SRDF (tip at link7) is legacy.
            try:
                srdf_path = str(
                    Path(get_package_share_directory("open_manipulator_6dof_moveit"))
                    / "config" / "open_manipulator_6dof.srdf"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"could not auto-locate SRDF: {exc} — named poses disabled"
                )
                srdf_path = ""
        if srdf_path:
            self._load_srdf(srdf_path)

        # ---- threading ----
        # We service the sequence on a worker thread because each MoveGroup
        # round-trip blocks waiting for action results. Running it on the
        # executor thread would deadlock the action future callbacks.
        self._cb_group = ReentrantCallbackGroup()
        self._run_lock = threading.Lock()
        self._worker: Optional[threading.Thread] = None
        self._sequence_busy = False
        self._auto_run_fired = False

        # ---- /joint_states cache (for snapshot helper) ----
        self._latest_joints: Optional[Dict[str, float]] = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self._cb_group,
        )

        # ---- services ----
        self.create_service(
            Trigger, "/run_pick_place",
            self._on_trigger, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/snapshot_waypoint",
            self._on_snapshot, callback_group=self._cb_group,
        )

        if bool(self.get_parameter("auto_run").value):
            self.get_logger().info("auto_run=true → starting sequence")
            self.create_timer(
                2.0, self._auto_run_once, callback_group=self._cb_group,
            )

    # ---------------- waypoints I/O ----------------
    @staticmethod
    def _load_waypoints(path: str) -> Dict[str, Any]:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("waypoints", data)  # accept nested or flat

    def _load_srdf(self, path: str) -> None:
        """Parse SRDF file → {state_name: {joint_name: value}}."""
        try:
            with open(path) as f:
                root = ET.fromstring(f.read())
        except (FileNotFoundError, ET.ParseError) as exc:
            self.get_logger().error(f"SRDF read/parse error ({path}): {exc}")
            return
        new_states: Dict[str, Dict[str, float]] = {}
        for gs in root.findall("group_state"):
            name = gs.get("name")
            if not name:
                continue
            joints: Dict[str, float] = {}
            for j in gs.findall("joint"):
                jn = j.get("name")
                jv = j.get("value")
                if jn is None or jv is None:
                    continue
                joints[jn] = float(jv)
            if joints:
                new_states[name] = joints
        self.named_poses = new_states
        self.get_logger().info(
            f"SRDF loaded from {path}: named poses {list(self.named_poses.keys())}"
        )

    def _go_waypoint(self, name: str) -> bool:
        if name not in self.waypoints:
            self.get_logger().error(f"Unknown waypoint '{name}'")
            return False
        wp = self.waypoints[name]
        if isinstance(wp, str):
            # named SRDF group_state shorthand
            return self.client.move_to_named_pose(wp, named_poses=self.named_poses)
        if not isinstance(wp, dict):
            self.get_logger().error(f"Waypoint '{name}' has invalid format: {wp}")
            return False
        if "named" in wp:
            return self.client.move_to_named_pose(
                str(wp["named"]), named_poses=self.named_poses,
            )
        if "joint" in wp:
            return self.client.move_to_joint_values([float(x) for x in wp["joint"]])
        if "xyz" in wp:
            xyz = [float(x) for x in wp["xyz"]]
            rpy = [float(x) for x in wp.get("rpy", [0.0, 0.0, 0.0])]
            return self.client.move_to_xyz_rpy(*xyz, *rpy)
        self.get_logger().error(f"Waypoint '{name}' missing 'joint'/'xyz'/'named'")
        return False

    # ---------------- sequence ----------------
    def _run_sequence(self) -> bool:
        steps: List[tuple] = [
            ("move", "start_pose"),
            ("gripper", self.grip_open, "OPEN"),
            ("move", "pick_above"),
            ("move", "pick"),
            ("gripper", self.grip_close, "CLOSE (grasp)"),
            ("move", "pick_above"),
            ("move", "place_above"),
            ("move", "place"),
            ("gripper", self.grip_open, "OPEN (release)"),
            ("move", "place_above"),
            ("move", "start_pose"),
        ]
        for i, step in enumerate(steps, start=1):
            kind = step[0]
            label = f"[{i}/{len(steps)}] {kind}"
            if kind == "move":
                name = step[1]
                self.get_logger().info(f"{label} → waypoint '{name}'")
                if not self._go_waypoint(name):
                    self.get_logger().error(f"{label} FAILED at '{name}' — aborting")
                    return False
            elif kind == "gripper":
                pos, comment = step[1], step[2]
                self.get_logger().info(f"{label} {comment} ({pos:+.4f})")
                if not self.client.set_gripper(pos):
                    self.get_logger().error(f"{label} FAILED — aborting")
                    return False
        self.get_logger().info("Pick-and-place sequence COMPLETE")
        return True

    # ---------------- worker thread ----------------
    def _start_worker(self) -> bool:
        """Spawn the sequence runner on a background thread. Returns False if
        a sequence is already in progress."""
        with self._run_lock:
            if self._sequence_busy:
                return False
            self._sequence_busy = True
        self._worker = threading.Thread(target=self._worker_body, daemon=True)
        self._worker.start()
        return True

    def _worker_body(self) -> None:
        try:
            if not self.client.wait_for_servers(timeout_sec=30.0):
                self.get_logger().error("Required action servers not available")
                return
            needs_named = any(
                isinstance(w, str)
                or (isinstance(w, dict) and "named" in w)
                for w in self.waypoints.values()
            )
            if needs_named and not self.named_poses:
                self.get_logger().error(
                    "Waypoints reference named SRDF states but no "
                    "named poses were loaded — set parameter `srdf_file` "
                    "or switch waypoints to `joint:` / `xyz:` form."
                )
                return
            self._run_sequence()
        finally:
            with self._run_lock:
                self._sequence_busy = False

    # ---------------- callbacks ----------------
    def _auto_run_once(self) -> None:
        # one-shot
        if self._auto_run_fired:
            return
        self._auto_run_fired = True
        self._start_worker()

    def _on_trigger(self, req: Trigger.Request, res: Trigger.Response):
        if not self._start_worker():
            res.success = False
            res.message = "sequence already running"
            return res
        # Return immediately. Sequence runs on the worker thread; client can
        # follow logs (or `ros2 topic echo /joint_states`) to watch progress.
        res.success = True
        res.message = "started"
        return res

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joints = dict(zip(msg.name, msg.position))

    def _on_snapshot(self, req: Trigger.Request, res: Trigger.Response):
        """Return current arm joint values as a YAML-ready block — perfect
        for pasting into waypoints.yaml after positioning the arm via RViz."""
        if not self._latest_joints:
            res.success = False
            res.message = "no /joint_states received yet"
            return res
        try:
            vals = [round(float(self._latest_joints[n]), 4) for n in ARM_JOINT_ORDER]
        except KeyError as exc:
            res.success = False
            res.message = f"joint not in /joint_states: {exc}"
            return res
        snippet = (
            "  # paste under `waypoints:` and pick a name\n"
            "  YOUR_NAME:\n"
            f"    joint: {vals}\n"
        )
        self.get_logger().info(f"snapshot:\n{snippet}")
        res.success = True
        res.message = snippet
        return res


def main(args=None):
    rclpy.init(args=args)
    node: Optional[PickPlaceNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = PickPlaceNode()
        # MultiThreadedExecutor so action client futures get serviced on a
        # different thread than the one waiting on them.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
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
