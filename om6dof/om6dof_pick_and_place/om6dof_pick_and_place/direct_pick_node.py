"""Direct vision pick-and-place sequencer through ordinary MoveGroup.

MoveGroup plans Cartesian and joint targets and executes them through
``arm_controller``. ``om6dof_bringup`` remains the only hardware owner. Remote
joint teleop must be OFF while this sequence runs.

Kinematics here loads IKSolver only for forward-kinematics reachability and
camera calculations. The wrist-camera extrinsic (camera_xyz/camera_rpy,
parent = end_effector_link) then places the tag in the arm base frame:

    world <- FK(joint_states) <- end_effector_link <- camera <- AprilTag

Sequence (front pick, matches the ALOHA-style approach):
   1. → ready pose (joint goal)
   2. ↓ gripper OPEN
   3. → standoff:  front_standoff in front of the object, tilted up
   4. → advance:   straight in to grasp depth
   5. ↓ gripper CLOSE
   6. → retreat:   back to standoff + lift
   7. → place pose (joint goal), gripper OPEN
   8. → ready pose

Services:
   /run_direct_pick   std_srvs/Trigger  — run the full sequence
   /direct_approach    std_srvs/Trigger  — steps 1-3 only (approach, no grasp)
   /direct_pick_status std_srvs/Trigger  — report current step / object pose
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .tag_pick_place_node import (R_BODY_OPTICAL, quat_to_matrix,
                                  rpy_to_matrix)
from .moveit_client import MoveItClient


def _matrix_to_quat(R):
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    idx = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if idx == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if idx == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def rpy_pose(pitch: float, yaw: float, roll: float = 0.0) -> np.ndarray:
    """Tool orientation matrix from roll/pitch/yaw (base frame ZYX)."""
    return rpy_to_matrix(roll, pitch, yaw)


class DirectPickNode(Node):
    def __init__(self) -> None:
        super().__init__("direct_pick")

        # ---- kinematics (forward only) ----
        self.declare_parameter("ik_urdf_pkg", "om6dof_description")
        self.declare_parameter("ik_base_link", "world")
        self.declare_parameter("ik_tip_link", "end_effector_link")
        self.declare_parameter("arm_joint_names",
                               ["joint1", "joint2", "joint3",
                                "joint4", "joint5", "joint6"])
        from om6dof_controller.ik_solver import IKSolver
        self.ik = IKSolver(
            base_link=str(self.get_parameter("ik_base_link").value),
            tip_link=str(self.get_parameter("ik_tip_link").value),
            urdf_pkg=str(self.get_parameter("ik_urdf_pkg").value),
            xacro_rel="urdf/om6dof.urdf.xacro",
        )
        self.arm_joints = [str(x) for x in
                           self.get_parameter("arm_joint_names").value]

        # ---- camera extrinsic (parent = end_effector_link) ----
        self.declare_parameter("camera_xyz", [-0.08247, 0.0, -0.0096])
        self.declare_parameter("camera_rpy", [0.0, -1.1345, 0.0])
        self.declare_parameter("object_center_from_tag", [0.0, 0.0, -0.015])
        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("max_sample_age", 1.5)
        self._t_ec = np.array([float(v) for v in
                               self.get_parameter("camera_xyz").value])
        self._R_eo = rpy_to_matrix(
            *[float(v) for v in self.get_parameter("camera_rpy").value]) \
            @ R_BODY_OPTICAL

        # ---- front-pick geometry ----
        self.declare_parameter("front_standoff", 0.05)   # advance ~5 cm
        self.declare_parameter("front_approach_z_offset", 0.0)
        self.declare_parameter("front_grasp_back", 0.005)   # (legacy, unused)
        # approach offset [standoff, up] (m): where the arm hovers before the
        # final advance. standoff = distance in front of the object; up =
        # height above the object centre. Calibrate live from the GUI. If
        # left at [0,0] the legacy front_standoff/front_approach_z_offset
        # scalars are used instead (back-compat).
        self.declare_parameter("approach_offset", [0.0, 0.0])  # [standoff, up]; up=0 → approach sejajar tinggi objek (z≈0)
        # pick offset [forward, up] (m): where the fingers actually grasp
        # vs the estimated object centre (end_effector_link != fingertip).
        # forward+ = deeper toward/past the object; up+ = higher. Calibrate
        # live with the GUI steppers.
        self.declare_parameter("pick_offset", [0.0, 0.0])
        self.declare_parameter("front_pitch", 1.5708)     # pi/2 horizontal
        self.declare_parameter("lift_after_grasp", 0.05)
        self.declare_parameter("min_z", 0.01)
        self.declare_parameter("yaw_offset", 0.0)
        # named joint poses (rad, joint1..6)
        self.declare_parameter("ready_pose",
                               [0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0])
        self.declare_parameter("place_pose",
                               [1.2, -0.5, 1.2, 0.0, 0.9, 0.0])
        self.declare_parameter("gripper_open", 0.019)
        self.declare_parameter("gripper_close", -0.010)
        self.declare_parameter("gripper_settle_s", 1.2)
        self.declare_parameter("goal_timeout_s", 25.0)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("vel_scale", 0.2)
        self.declare_parameter("acc_scale", 0.2)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("orientation_tolerance", 0.25)

        # once localised, a wrist camera loses the tag as it moves in — so we
        # keep the last good object pose and reuse it when detection is stale.
        self.declare_parameter("object_cache_max_age", 20.0)   # s
        # ---- state ----
        self._samples: deque = deque(maxlen=60)     # (t, p_opt, q_opt)
        self._last_obj = None       # (tag_w, obj_w, R_wt)
        self._last_obj_t = 0.0
        self._joints: Optional[dict] = None
        self._status_line = "ready"
        self._run_lock = threading.Lock()
        self._busy = False
        self._cancel = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        cb = ReentrantCallbackGroup()

        # ---- IO ----
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_topic").value),
            self._on_tag, 10, callback_group=cb)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10, callback_group=cb)
        self.client = MoveItClient(
            self,
            arm_joint_names=self.arm_joints,
            planning_time=float(self.get_parameter("planning_time").value),
            num_planning_attempts=int(
                self.get_parameter("planning_attempts").value),
            max_velocity_scaling=float(self.get_parameter("vel_scale").value),
            max_acceleration_scaling=float(self.get_parameter("acc_scale").value),
            position_tolerance=float(
                self.get_parameter("position_tolerance").value),
            orientation_tolerance=float(
                self.get_parameter("orientation_tolerance").value),
        )
        # continuous reachability check for the live object
        self._reach_pub = self.create_publisher(
            String, "/direct_reach",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_timer(0.5, self._reach_tick, callback_group=cb)

        self.create_service(Trigger, "/run_direct_pick", self._on_run,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_approach", self._on_approach,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_pick_status", self._on_status,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_go_origin", self._on_go_origin,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_go_ready", self._on_go_ready,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_grip_open",
                            lambda rq, rs: self._grip_srv(rq, rs, "open"),
                            callback_group=cb)
        self.create_service(Trigger, "/direct_grip_close",
                            lambda rq, rs: self._grip_srv(rq, rs, "close"),
                            callback_group=cb)
        self.create_service(Trigger, "/direct_track", self._on_track,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_stop", self._on_stop,
                            callback_group=cb)
        self.create_service(Trigger, "/direct_reachable", self._on_reachable,
                            callback_group=cb)
        self.get_logger().info(
            "direct pick ready — MoveGroup executes through arm_controller. "
            "Ensure F3 remote mode is OFF, then call /run_direct_pick.")

    # ---------------- callbacks ----------------
    def _on_tag(self, msg: PoseStamped) -> None:
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        self._samples.append((time.monotonic(), p, q))

    def _on_joints(self, msg: JointState) -> None:
        self._joints = dict(zip(msg.name, msg.position))

    # ---------------- geometry ----------------
    def _arm_q(self) -> Optional[np.ndarray]:
        if not self._joints:
            return None
        try:
            return np.array([float(self._joints[n]) for n in self.arm_joints])
        except KeyError:
            return None

    def _tag_object_world(self, allow_stale: bool = False):
        """(tag_world, object_world, R_wt) using self-computed FK — no TF.

        With a fresh detection, compute and CACHE the result. If detection is
        stale/missing and `allow_stale` is set, return the last cached pose
        (until object_cache_max_age) instead of None — so the pick keeps its
        heading after the wrist camera loses the tag on the way in."""
        now = time.monotonic()
        max_age = float(self.get_parameter("max_sample_age").value)
        samples = [s for s in self._samples if now - s[0] <= max_age]
        q = self._arm_q()
        if len(samples) >= int(self.get_parameter("min_samples").value) \
                and q is not None:
            # forward kinematics: world <- end_effector_link
            p_we, R_we = self.ik.fk_pose(q)
            t_wo = R_we @ self._t_ec + p_we
            R_wo = R_we @ self._R_eo
            p_opt = np.median(np.stack([s[1] for s in samples]), axis=0)
            R_ot = quat_to_matrix(*[float(v) for v in samples[-1][2]])
            tag_w = R_wo @ p_opt + t_wo
            R_wt = R_wo @ R_ot
            offs = np.array([float(v) for v in
                             self.get_parameter("object_center_from_tag").value])
            obj_w = tag_w + R_wt @ offs
            self._last_obj = (tag_w, obj_w, R_wt)
            self._last_obj_t = now
            return self._last_obj
        # no fresh detection
        if allow_stale and self._last_obj is not None:
            age = now - self._last_obj_t
            if age <= float(self.get_parameter("object_cache_max_age").value):
                self.get_logger().info(
                    f"tag lost — using last known object ({age:.1f}s old)",
                    throttle_duration_sec=1.0)
                return self._last_obj
        return None

    def _approach_geom(self):
        """Effective (standoff, up) — from approach_offset if set, else the
        legacy front_standoff / front_approach_z_offset scalars."""
        ao = [float(v) for v in self.get_parameter("approach_offset").value]
        ao = (ao + [0.0, 0.0])[:2]
        if abs(ao[0]) > 1e-6 or abs(ao[1]) > 1e-6:
            return ao[0], ao[1]
        return (float(self.get_parameter("front_standoff").value),
                float(self.get_parameter("front_approach_z_offset").value))

    def _reachable(self, obj_w: np.ndarray):
        """Check whether the object is in the arm's workspace: does IK have a
        solution for BOTH the standoff hover pose AND the grasp pose (within
        joint limits)? Returns a dict with the verdict + details, or None if
        no joint state yet."""
        q = self._arm_q()
        if q is None:
            return None
        standoff, zoff = self._approach_geom()
        pitch = float(self.get_parameter("front_pitch").value)
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        min_z = float(self.get_parameter("min_z").value)
        bearing = self._bearing(obj_w)
        R = rpy_pose(pitch, yaw)

        stand = obj_w - standoff * bearing
        stand[2] = max(obj_w[2] + zoff, min_z)
        grasp = self._ee_for_gripper(obj_w.copy(), bearing)
        grasp[2] = max(grasp[2], min_z)

        def ik_ok(pos):
            q_sol, conv = self.ik.solve_pose_ik(q, pos, R)
            p2, _ = self.ik.fk_pose(np.array(q_sol))
            return bool(conv) and float(np.linalg.norm(p2 - pos)) < 0.02

        r = float(np.hypot(obj_w[0], obj_w[1]))
        stand_ok = ik_ok(stand)
        grasp_ok = ik_ok(grasp)
        return {
            "reachable": stand_ok and grasp_ok,
            "standoff_ok": stand_ok,
            "grasp_ok": grasp_ok,
            "radius": round(r, 3),
            "obj": [round(float(v), 3) for v in obj_w],
        }

    def _bearing(self, obj_w: np.ndarray) -> np.ndarray:
        """Unit horizontal direction base→object = the approach direction."""
        d = np.array([obj_w[0], obj_w[1], 0.0])
        n = np.linalg.norm(d)
        return d / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])

    def _front_target(self, obj_w: np.ndarray, back: float) -> np.ndarray:
        d = self._bearing(obj_w)
        return np.array([obj_w[0] - back * d[0],
                         obj_w[1] - back * d[1], obj_w[2]])

    def _ee_for_gripper(self, grip_pos: np.ndarray,
                        bearing: np.ndarray) -> np.ndarray:
        """Convert a desired GRIPPER (fingertip) world position to the
        end_effector_link command, subtracting the fingertip offset
        (pick_offset = fingertip position relative to end_effector_link,
        [forward, up]). This is what makes standoff/grasp distances measured
        to the GRIPPER, not to end_effector_link or the camera."""
        po = [float(v) for v in self.get_parameter("pick_offset").value]
        fwd, up = (po + [0.0, 0.0])[:2]
        return np.array([grip_pos[0] - fwd * bearing[0],
                         grip_pos[1] - fwd * bearing[1],
                         grip_pos[2] - up])

    # ---------------- MoveGroup commands ----------------
    def _send_cart(self, pos: np.ndarray, R: np.ndarray, label: str) -> bool:
        """Plan and execute one Cartesian target through MoveGroup."""
        if self._cancel.is_set():
            return False
        if not self.client.wait_for_move_server(timeout_sec=5.0):
            self.get_logger().error(
                f"{label} FAILED: MoveGroup unavailable or remote mode still owns the arm")
            return False
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = \
            float(pos[0]), float(pos[1]), float(pos[2])
        qx, qy, qz, qw = _matrix_to_quat(R)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self._status_line = f"{label} → ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
        self.get_logger().info(self._status_line)
        ok = self.client.move_to_pose(pose)
        if self._cancel.is_set():
            self.get_logger().warn(f"{label} cancelled after current MoveGroup goal")
            return False
        if not ok:
            self.get_logger().error(f"{label} FAILED in MoveGroup")
        return ok

    def _send_joint_pose(self, pose, label: str) -> bool:
        """Plan and execute a six-joint target through MoveGroup."""
        if self._cancel.is_set():
            return False
        if not self.client.wait_for_move_server(timeout_sec=5.0):
            return False
        target = [float(v) for v in pose]
        self._status_line = f"{label} (MoveGroup)"
        self.get_logger().info(self._status_line)
        ok = self.client.move_to_joint_values(target)
        return bool(ok and not self._cancel.is_set())

    def _gripper(self, which: str, label: str = "") -> bool:
        """Open/close through the always-active GripperActionController."""
        which = which.strip().lower()
        self._status_line = f"gripper {label or which.upper()}"
        self.get_logger().info(self._status_line)
        position = float(self.get_parameter(
            "gripper_open" if which == "open" else "gripper_close").value)
        return self.client.set_gripper(position, max_effort=0.0)

    # ---------------- sequence ----------------
    def _approach_only(self) -> bool:
        found = self._tag_object_world(allow_stale=True)
        if found is None:
            self.get_logger().error("no tag ever seen — cannot pick")
            self._status_line = "no tag"
            return False
        _tw, obj_w, _R = found
        standoff, zoff = self._approach_geom()
        pitch = float(self.get_parameter("front_pitch").value)
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        min_z = float(self.get_parameter("min_z").value)
        bearing = self._bearing(obj_w)

        self._send_joint_pose(self.get_parameter("ready_pose").value, "ready")
        self._gripper("open", "OPEN")

        # HOVER = end_effector_link `standoff` in front of the object and
        # `zoff` above it. This is EE-referenced and simple: approach_offset
        # directly sets where the arm's tool point hovers. pick_offset is NOT
        # applied here — it only fine-tunes the final grasp, so the two
        # offsets no longer interact.
        stand = obj_w - standoff * bearing
        stand[2] = max(obj_w[2] + zoff, min_z)
        self.get_logger().info(
            f"standoff: gripper {standoff*100:.0f} cm in front of object, "
            f"{zoff*100:.0f} cm above")
        R = rpy_pose(pitch, yaw)
        return self._send_cart(stand, R, "standoff"), obj_w, yaw, pitch, R

    def _run_sequence(self) -> bool:
        res = self._approach_only()
        if isinstance(res, bool):     # approach failed early
            return False
        ok, obj_w, yaw, pitch, R = res
        if not ok:
            return False

        min_z = float(self.get_parameter("min_z").value)
        lift = float(self.get_parameter("lift_after_grasp").value)
        standoff, zoff = self._approach_geom()
        bearing = self._bearing(obj_w)

        # ADVANCE (grasp): here — and ONLY here — apply pick_offset so the
        # fingertip lands on the object. _ee_for_gripper puts the GRIPPER at
        # the object centre by offsetting the EE command by pick_offset.
        grasp_pt = self._ee_for_gripper(obj_w.copy(), bearing)
        grasp_pt[2] = max(grasp_pt[2], min_z)
        if not self._send_cart(grasp_pt, R, "advance"):
            return False
        self._gripper("close", "CLOSE")

        # retreat: EE back to the hover point (standoff, zoff), then lift.
        # NON-FATAL: if the Cartesian retreat is infeasible from the grasp
        # pose, don't freeze holding the object — fall through to the
        # joint-space place pose, which is always reachable.
        if self._cancel.is_set():
            return False
        back = obj_w - standoff * bearing
        back[2] = max(obj_w[2] + zoff + lift, min_z)
        if not self._send_cart(back, R, "retreat+lift"):
            self.get_logger().warn(
                "retreat infeasible — going straight to place (still holding)")

        # place + release + return
        self._send_joint_pose(self.get_parameter("place_pose").value, "place")
        self._gripper("open", "OPEN")
        self._send_joint_pose(self.get_parameter("ready_pose").value, "ready")
        self._status_line = "COMPLETE"
        self.get_logger().info("direct pick-and-place COMPLETE")
        return True

    def _track_worker(self) -> None:
        """Continuous Servo tracking was intentionally removed."""
        with self._run_lock:
            self._busy = False

    def _publish_pose(self, pub, pos, R) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
            float(pos[0]), float(pos[1]), float(pos[2])
        qx, qy, qz, qw = _matrix_to_quat(R)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)

    def _preempt(self) -> bool:
        """Cancel any running sequence and wait for its worker to exit, so a
        new command can start immediately (e.g. re-running APPROACH after
        changing an offset — no 'sequence already running' rejection)."""
        with self._run_lock:
            busy = self._busy
            th = self._worker_thread
        if busy:
            self.get_logger().info("preempting running sequence …")
            self._cancel.set()
            if th is not None and th.is_alive():
                th.join(timeout=3.0)
            if th is not None and th.is_alive():
                self.get_logger().warn(
                    "current MoveGroup goal is still finishing; new command rejected")
                return False
        self._cancel.clear()
        return True

    def _worker(self, approach_only: bool) -> None:
        try:
            if approach_only:
                res = self._approach_only()
                if not isinstance(res, bool) and res[0]:
                    self._status_line = "approach done — arm at standoff"
                    self.get_logger().info(self._status_line)
            else:
                self._run_sequence()
        finally:
            with self._run_lock:
                self._busy = False

    def _start(self, approach_only: bool) -> bool:
        if not self._preempt():
            return False
        with self._run_lock:
            if self._busy:
                return False
            self._busy = True
            self._cancel.clear()
        self._worker_thread = threading.Thread(
            target=self._worker, args=(approach_only,), daemon=True)
        self._worker_thread.start()
        return True

    # ---------------- services ----------------
    def _on_run(self, req, res):
        if not self._start(False):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "direct pick started"
        return res

    def _on_approach(self, req, res):
        if not self._start(True):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "approach started"
        return res

    def _on_track(self, req, res):
        res.success = False
        res.message = (
            "continuous tracking was a MoveIt Servo feature and is disabled; "
            "use /direct_approach for a planned snapshot target")
        return res

    def _on_stop(self, req, res):
        stopped = self._preempt()
        self._status_line = "stopped"
        res.success = stopped
        res.message = (
            "stopped" if stopped
            else "cancel requested; current MoveGroup goal is still finishing")
        return res

    def _reach_tick(self) -> None:
        """Publish live reachability of the current object (allow_stale so it
        keeps a verdict briefly after the tag flickers)."""
        found = self._tag_object_world(allow_stale=True)
        if found is None:
            self._reach_pub.publish(String(data="no_object"))
            return
        rr = self._reachable(found[1])
        if rr is None:
            self._reach_pub.publish(String(data="no_joints"))
            return
        if rr["reachable"]:
            msg = f"reachable r={rr['radius']:.2f}m"
        elif not rr["standoff_ok"] and not rr["grasp_ok"]:
            msg = f"OUT of reach r={rr['radius']:.2f}m"
        elif not rr["grasp_ok"]:
            msg = f"grasp unreachable r={rr['radius']:.2f}m (standoff ok)"
        else:
            msg = f"standoff unreachable r={rr['radius']:.2f}m"
        self._reach_pub.publish(String(data=msg))

    def _on_reachable(self, req, res):
        found = self._tag_object_world(allow_stale=True)
        if found is None:
            res.success = False
            res.message = "no object"
            return res
        rr = self._reachable(found[1])
        if rr is None:
            res.success = False
            res.message = "no joint state"
            return res
        res.success = bool(rr["reachable"])
        res.message = (
            f"{'DALAM jangkauan' if rr['reachable'] else 'DI LUAR jangkauan'} — "
            f"radius {rr['radius']:.3f} m, standoff "
            f"{'ok' if rr['standoff_ok'] else 'GAGAL'}, grasp "
            f"{'ok' if rr['grasp_ok'] else 'GAGAL'} "
            f"(obj {rr['obj']})")
        return res

    def _go_pose_worker(self, pose, label: str) -> None:
        try:
            self._send_joint_pose(pose, label)
            self._status_line = f"at {label}"
        finally:
            with self._run_lock:
                self._busy = False

    def _start_go_pose(self, pose, label: str) -> bool:
        if not self._preempt():
            return False
        with self._run_lock:
            if self._busy:
                return False
            self._busy = True
            self._cancel.clear()
        self._worker_thread = threading.Thread(
            target=self._go_pose_worker, args=(pose, label), daemon=True)
        self._worker_thread.start()
        return True

    def _on_go_origin(self, req, res):
        if not self._start_go_pose([0.0] * 6, "origin (0,0,0,0,0,0)"):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "moving to origin (all joints 0)"
        return res

    def _on_go_ready(self, req, res):
        pose = [float(v) for v in self.get_parameter("ready_pose").value]
        if not self._start_go_pose(pose, "ready pose"):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "moving to ready pose"
        return res

    def _grip_srv(self, req, res, which: str):
        self._gripper(which, which.upper() + " (test)")
        res.success = True
        res.message = f"gripper {which} via GripperActionController"
        return res

    def _on_status(self, req, res):
        found = self._tag_object_world()
        obj = "no tag" if found is None else \
            f"object=({found[1][0]:+.3f},{found[1][1]:+.3f},{found[1][2]:+.3f})"
        res.success = True
        res.message = f"step: {self._status_line} | {obj}"
        return res


def main(args=None):
    rclpy.init(args=args)
    node: Optional[DirectPickNode] = None
    try:
        node = DirectPickNode()
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.spin()
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
