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
   3. → pre-position: move in front while preserving tool orientation
   4. ↻ align: rotate the gripper while holding that position
   5. → advance: move straight in through short waypoints
   6. ↓ gripper CLOSE
   7. → retreat: back to standoff + lift
   8. → place pose (joint goal), gripper OPEN, then ready

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
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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


def optical_point_to_world(p_opt, p_we, R_we, t_ec, R_eo):
    """Transform one camera-optical point through the wrist extrinsic."""
    return R_we @ t_ec + p_we + (R_we @ R_eo) @ p_opt


def stable_point_median(samples, max_spread: float):
    """Median point if all samples form a sufficiently tight 3D cluster."""
    if not samples:
        return None
    points = np.stack(samples)
    median = np.median(points, axis=0)
    spread = float(np.max(np.linalg.norm(points - median, axis=1)))
    return median if spread <= max_spread else None


def linear_waypoints(start, end, max_step: float):
    """Evenly-spaced Cartesian points from ``start`` (excluded) to end."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    distance = float(np.linalg.norm(end - start))
    step = max(float(max_step), 1e-4)
    count = max(1, int(math.ceil(distance / step)))
    return [start + (end - start) * (index / count)
            for index in range(1, count + 1)]


def approach_standoff_distances(coarse: float, final: float, step: float):
    """Descending visual-approach distances, always ending at ``final``."""
    coarse = max(float(coarse), float(final))
    final = max(0.0, float(final))
    step = max(0.005, abs(float(step)))
    distances = []
    current = coarse
    while current > final + 1e-6:
        distances.append(current)
        current -= step
    if not distances or abs(distances[-1] - final) > 1e-6:
        distances.append(final)
    return distances


def safe_approach_standoff_distances(object_radius: float, distances,
                                      minimum_target_radius: float):
    """Drop standoffs that fold the arm too close to its base.

    For a frontal approach the EE radius is approximately object radius minus
    standoff. Low targets below about 0.28 m radius cause link7/link4
    self-collision on this arm.
    """
    radius = float(object_radius)
    minimum = max(0.0, float(minimum_target_radius))
    return [float(distance) for distance in distances
            if radius - float(distance) >= minimum - 1e-9]


def image_axis_tracking_target(current_joint: float, optical_axis: float,
                               optical_z: float, gain: float,
                               direction: float, deadband_rad: float,
                               max_step_rad: float, joint_min: float,
                               joint_max: float):
    """Map one optical-image angular error to one bounded joint target."""
    if not all(math.isfinite(value) for value in
               (current_joint, optical_axis, optical_z)) or optical_z <= 0.0:
        return current_joint, 0.0, False
    error = math.atan2(optical_axis, optical_z)
    if abs(error) <= max(0.0, deadband_rad):
        return current_joint, error, False
    correction = direction * gain * error
    correction = float(np.clip(correction, -abs(max_step_rad), abs(max_step_rad)))
    target = float(np.clip(current_joint + correction, joint_min, joint_max))
    return target, error, abs(target - current_joint) > 1e-5


def yaw_tracking_target(current_yaw: float, optical_x: float,
                        optical_z: float, gain: float, direction: float,
                        deadband_rad: float, max_step_rad: float,
                        yaw_min: float, yaw_max: float):
    """Backward-compatible horizontal tracking helper."""
    return image_axis_tracking_target(
        current_yaw, optical_x, optical_z, gain, direction,
        deadband_rad, max_step_rad, yaw_min, yaw_max)


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
        self.declare_parameter("object_source", "apriltag")
        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter(
            "perception_topic", "/om6dof_perception/target_point"
        )
        self.declare_parameter("perception_max_spread", 0.025)
        self.declare_parameter("perception_world_min", [0.08, -0.35, -0.08])
        self.declare_parameter("perception_world_max", [0.50, 0.35, 0.45])
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
        self.declare_parameter("front_pitch_fallbacks",
                               [1.5708, 1.65, 1.72, 1.8, 1.9, 2.0,
                                2.1, 2.2, 2.3, 2.4, 1.4, 1.2])
        # During visual approach keep the tool on the upright/front-facing
        # side. Higher pitch fallbacks remain available only for the final IK.
        self.declare_parameter("approach_upright_pitch_max", 2.0)
        self.declare_parameter("advance_step_m", 0.015)
        self.declare_parameter("align_position_tolerance", 0.006)
        self.declare_parameter("align_orientation_tolerance", 0.10)
        self.declare_parameter("advance_position_tolerance", 0.004)
        self.declare_parameter("advance_orientation_tolerance", 0.10)
        # Horizontal object tracking: only joint1 is changed. A positive X in
        # the optical frame is right in the image; this mount needs negative
        # joint1 motion to turn the camera right.
        self.declare_parameter("tracking_rate_hz", 5.0)
        self.declare_parameter("tracking_gain", 0.45)
        self.declare_parameter("tracking_yaw_sign", -1.0)
        self.declare_parameter("tracking_deadband_rad", 0.045)
        self.declare_parameter("tracking_max_step_rad", 0.05)
        self.declare_parameter("tracking_trajectory_s", 0.30)
        self.declare_parameter("tracking_target_timeout_s", 0.6)
        self.declare_parameter("tracking_joint1_min", -2.60)
        self.declare_parameter("tracking_joint1_max", 2.60)
        # Optical Y is positive downward. FK of this camera mount shows that
        # positive joint5 reduces positive image-Y error, hence sign +1.
        self.declare_parameter("tracking_vertical_gain", 0.35)
        self.declare_parameter("tracking_pitch_sign", 1.0)
        self.declare_parameter("tracking_vertical_deadband_rad", 0.045)
        self.declare_parameter("tracking_vertical_max_step_rad", 0.04)
        self.declare_parameter("tracking_joint5_min", -1.80)
        self.declare_parameter("tracking_joint5_max", 1.80)
        # Pickup interleaves visual centering and short approach moves. It
        # starts farther away, refreshes the 3D target after each step, then
        # performs a final horizontal lock before align/advance/grasp.
        self.declare_parameter("pickup_tracking_enabled", True)
        self.declare_parameter("pickup_tracking_timeout_s", 8.0)
        self.declare_parameter("pickup_tracking_stable_cycles", 3)
        self.declare_parameter("pickup_object_refresh_timeout_s", 2.0)
        self.declare_parameter("pickup_coarse_standoff_m", 0.16)
        self.declare_parameter("pickup_approach_step_m", 0.04)
        self.declare_parameter("pickup_min_approach_radius_m", 0.28)
        self.declare_parameter("pickup_optical_depth_min_m", 0.05)
        self.declare_parameter("pickup_optical_depth_max_m", 1.00)
        self.declare_parameter("lift_after_grasp", 0.05)
        # The arm base/world origin is above the supporting surface on this
        # rig. A detected object centre around z=-0.04 m is valid; clamp the
        # commanded EE/fingertip height to -0.02 m so it grasps the upper body
        # without descending to the table (bbox bottom is about -0.09 m).
        self.declare_parameter("min_z", -0.02)
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
        self._perception_samples: deque = deque(maxlen=60)  # (t, p_opt)
        self.object_source = str(
            self.get_parameter("object_source").value
        ).strip().lower()
        self._last_obj = None       # (tag_w, obj_w, R_wt)
        self._last_obj_t = 0.0
        self._joints: Optional[dict] = None
        self._status_line = "ready"
        self._run_lock = threading.Lock()
        self._busy = False
        self._tracking_mode = False
        self._cancel = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        cb = ReentrantCallbackGroup()

        # ---- IO ----
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_topic").value),
            self._on_tag, 10, callback_group=cb)
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("perception_topic").value),
            self._on_perception_target,
            10,
            callback_group=cb,
        )
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
        self._trajectory_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self._tracking_status_pub = self.create_publisher(
            String, "/direct_tracking_status",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        # continuous reachability check for the live object
        self._reach_pub = self.create_publisher(
            String, "/direct_reach",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_timer(0.5, self._reach_tick, callback_group=cb)

        self.create_service(Trigger, "/run_direct_pick", self._on_run,
                            callback_group=cb)
        if self.object_source == "perception":
            self.create_service(
                Trigger, "/run_perception_pick", self._on_run,
                callback_group=cb,
            )
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
            f"direct pick ready — source={self.object_source}; "
            "MoveGroup executes through arm_controller. "
            "Ensure F3 remote mode is OFF, then call /run_direct_pick.")

    # ---------------- callbacks ----------------
    def _on_tag(self, msg: PoseStamped) -> None:
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        self._samples.append((time.monotonic(), p, q))

    def _on_perception_target(self, msg: PointStamped) -> None:
        p = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        if np.all(np.isfinite(p)) and p[2] > 0.0:
            self._perception_samples.append((time.monotonic(), p))

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

    def _perception_object_world(self, allow_stale: bool = False):
        """Transform a stable YOLO/depth target from optical to arm world."""
        now = time.monotonic()
        max_age = float(self.get_parameter("max_sample_age").value)
        samples = [
            sample[1] for sample in self._perception_samples
            if now - sample[0] <= max_age
        ]
        q = self._arm_q()
        if len(samples) >= int(self.get_parameter("min_samples").value) \
                and q is not None:
            p_opt = stable_point_median(
                samples,
                float(self.get_parameter("perception_max_spread").value),
            )
            if p_opt is not None:
                p_we, R_we = self.ik.fk_pose(q)
                obj_w = optical_point_to_world(
                    p_opt, p_we, R_we, self._t_ec, self._R_eo
                )
                self._last_obj = (obj_w.copy(), obj_w, np.eye(3))
                self._last_obj_t = now
                return self._last_obj
        if allow_stale and self._last_obj is not None:
            age = now - self._last_obj_t
            if age <= float(self.get_parameter("object_cache_max_age").value):
                self.get_logger().info(
                    f"perception lost — using snapshot ({age:.1f}s old)",
                    throttle_duration_sec=1.0,
                )
                return self._last_obj
        return None

    def _object_world(self, allow_stale: bool = False):
        if self.object_source == "perception":
            return self._perception_object_world(allow_stale)
        return self._tag_object_world(allow_stale)

    def _approach_geom(self):
        """Effective (standoff, up) — from approach_offset if set, else the
        legacy front_standoff / front_approach_z_offset scalars."""
        ao = [float(v) for v in self.get_parameter("approach_offset").value]
        ao = (ao + [0.0, 0.0])[:2]
        if abs(ao[0]) > 1e-6 or abs(ao[1]) > 1e-6:
            return ao[0], ao[1]
        return (float(self.get_parameter("front_standoff").value),
                float(self.get_parameter("front_approach_z_offset").value))

    def _reachable(self, obj_w: np.ndarray, seed_from_current: bool = False):
        """Check whether the object is in the arm's workspace: does IK have a
        solution for BOTH the standoff hover pose AND the grasp pose (within
        joint limits)? Returns a dict with the verdict + details, or None if
        no joint state yet."""
        q = self._arm_q()
        if q is None:
            return None
        standoff, zoff = self._approach_geom()
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        min_z = float(self.get_parameter("min_z").value)
        bearing = self._bearing(obj_w)
        stand = obj_w - standoff * bearing
        stand[2] = max(obj_w[2] + zoff, min_z)
        grasp = self._ee_for_gripper(obj_w.copy(), bearing)
        grasp[2] = max(grasp[2], min_z)

        def ik_solve(seed, pos, R):
            q_sol, conv = self.ik.solve_pose_ik(seed, pos, R)
            p2, R2 = self.ik.fk_pose(np.array(q_sol))
            position_error = float(np.linalg.norm(p2 - pos))
            trace = float(np.trace(R2.T @ R))
            orientation_error = math.acos(float(np.clip(
                (trace - 1.0) * 0.5, -1.0, 1.0)))
            # The numerical solver's convergence threshold is 1e-12, much
            # tighter than hardware accuracy. Use the measured FK residual;
            # a 3 mm / 1 degree solution is valid even when conv=False.
            ok = position_error < 0.02 and orientation_error < 0.20
            return np.array(q_sol), ok

        # The sequence always starts from ready. Try the requested pitch and
        # its calibrated fallbacks at the alignment point, then solve grasp
        # from that aligned solution. This mirrors the physical order instead
        # of testing two unrelated IK targets from the current arm pose.
        ready = np.asarray(self.get_parameter("ready_pose").value, dtype=float)
        seed = np.asarray(q) if seed_from_current else ready
        if len(seed) != len(self.arm_joints):
            seed = np.asarray(q)
        pitches = [float(self.get_parameter("front_pitch").value)]
        pitches.extend(float(value) for value in
                       self.get_parameter("front_pitch_fallbacks").value)
        pitches = list(dict.fromkeys(round(value, 6) for value in pitches))
        stand_ok = False
        grasp_ok = False
        selected_pitch = pitches[0]
        selected_stand_q = None
        selected_grasp_q = None
        for candidate in pitches:
            R = rpy_pose(candidate, yaw)
            q_stand, candidate_stand_ok = ik_solve(seed, stand, R)
            if not candidate_stand_ok:
                continue
            stand_ok = True
            _q_grasp, candidate_grasp_ok = ik_solve(q_stand, grasp, R)
            if candidate_grasp_ok:
                grasp_ok = True
                selected_pitch = candidate
                selected_stand_q = q_stand
                selected_grasp_q = _q_grasp
                break

        r = float(np.hypot(obj_w[0], obj_w[1]))
        return {
            "reachable": stand_ok and grasp_ok,
            "standoff_ok": stand_ok,
            "grasp_ok": grasp_ok,
            "radius": round(r, 3),
            "obj": [round(float(v), 3) for v in obj_w],
            "pitch": selected_pitch,
            # Keep the exact, current-state-seeded IK branch.  Re-solving the
            # same Cartesian pose inside MoveIt can select an equivalent but
            # inverted wrist branch.
            "stand_q": selected_stand_q,
            "grasp_q": selected_grasp_q,
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
    def _send_cart(self, pos: np.ndarray, R: np.ndarray, label: str,
                   position_tolerance=None,
                   orientation_tolerance=None) -> bool:
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
        ok = self.client.move_to_pose(
            pose,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
        if self._cancel.is_set():
            self.get_logger().warn(f"{label} cancelled after current MoveGroup goal")
            return False
        if not ok:
            self.get_logger().error(f"{label} FAILED in MoveGroup")
        return ok

    def _send_position(self, pos: np.ndarray, label: str) -> bool:
        """Plan position only; orientation is deliberately unconstrained."""
        if self._cancel.is_set():
            return False
        if not self.client.wait_for_move_server(timeout_sec=5.0):
            return False
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = \
            float(pos[0]), float(pos[1]), float(pos[2])
        pose.orientation.w = 1.0
        self._status_line = (
            f"{label} → ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
        )
        self.get_logger().info(self._status_line)
        ok = self.client.move_to_position(pose)
        if not ok:
            self.get_logger().error(f"{label} FAILED in MoveGroup")
        return bool(ok and not self._cancel.is_set())

    def _send_continuous_pose(self, pos: np.ndarray, R: np.ndarray,
                              label: str,
                              max_joint_delta: Optional[float] = None,
                              max_wrist_roll_delta: float = 0.75) -> bool:
        """Execute the IK branch nearest the current joints.

        A raw Cartesian MoveGroup goal may be satisfied by several IK
        branches.  On this arm one of those branches rolls the wrist by
        roughly pi, which looks like the gripper turning upside down.  Solve
        from the measured joints and send that explicit joint target so the
        branch cannot change between visual-approach waypoints.
        """
        current = self._arm_q()
        if current is None:
            self._status_line = f"{label} FAILED — joint state unavailable"
            return False
        solution, converged = self.ik.solve_pose_ik(current, pos, R)
        reached_pos, reached_R = self.ik.fk_pose(np.asarray(solution))
        position_error = float(np.linalg.norm(reached_pos - pos))
        trace = float(np.trace(reached_R.T @ R))
        orientation_error = math.acos(float(np.clip(
            (trace - 1.0) * 0.5, -1.0, 1.0)))
        joint_delta = np.abs(np.asarray(solution) - current)
        delta = float(np.max(joint_delta))
        # joint2/joint3/joint5 can legitimately counter-rotate by a large
        # amount while the *tool* orientation stays fixed (e.g. reaching from
        # ready z=0.19 m to a low bottle at z=0.00 m).  A branch flip instead
        # appears primarily as a near-pi roll in joint4/joint6.
        wrist_roll_delta = float(max(joint_delta[3], joint_delta[5]))
        excessive_step = max_joint_delta is not None and \
            delta > abs(float(max_joint_delta))
        if (position_error >= 0.02
                or orientation_error >= 0.20
                or excessive_step
                or wrist_roll_delta > abs(float(max_wrist_roll_delta))):
            self._status_line = (
                f"{label} BLOCKED — orientation continuity "
                f"(Δjoint={math.degrees(delta):.1f}°, "
                f"Δroll={math.degrees(wrist_roll_delta):.1f}°, "
                f"pose error={position_error*1000:.0f} mm/"
                f"{math.degrees(orientation_error):.1f}°)"
            )
            self.get_logger().error(self._status_line)
            return False
        if not converged:
            self.get_logger().info(
                f"{label}: accepting bounded IK residual "
                f"{position_error*1000:.1f} mm/"
                f"{math.degrees(orientation_error):.1f}°")
        return self._send_joint_pose(solution, label + " (continuous IK)")

    def _send_position_preserve_orientation(self, pos: np.ndarray,
                                            label: str) -> bool:
        current = self._arm_q()
        if current is None:
            self._status_line = f"{label} FAILED — joint state unavailable"
            return False
        _current_pos, current_R = self.ik.fk_pose(current)
        return self._send_continuous_pose(pos, current_R, label)

    def _select_locked_approach_orientation(self, obj_w: np.ndarray,
                                            distances):
        """Choose one feasible upright orientation for the whole approach.

        Camera-ready orientation is not necessarily reachable near a low
        object. Evaluate each calibrated front pitch across every standoff,
        chaining IK from the current joints, then keep the best orientation
        fixed while moving closer.
        """
        seed = self._arm_q()
        if seed is None:
            return None
        _standoff, zoff = self._approach_geom()
        min_z = float(self.get_parameter("min_z").value)
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        requested = float(self.get_parameter("front_pitch").value)
        pitches = [requested]
        pitches.extend(float(value) for value in
                       self.get_parameter("front_pitch_fallbacks").value)
        pitch_max = float(self.get_parameter(
            "approach_upright_pitch_max").value)
        pitches = list(dict.fromkeys(
            round(value, 6) for value in pitches if 0.0 <= value <= pitch_max
        ))
        bearing = self._bearing(obj_w)
        candidates = []
        for pitch in pitches:
            R = rpy_pose(pitch, yaw)
            q_stage = np.asarray(seed, dtype=float)
            worst_pos = 0.0
            worst_ori = 0.0
            valid = True
            for distance in distances:
                target = obj_w - float(distance) * bearing
                target[2] = max(obj_w[2] + zoff, min_z)
                solution, _conv = self.ik.solve_pose_ik(q_stage, target, R)
                reached_pos, reached_R = self.ik.fk_pose(
                    np.asarray(solution))
                pos_error = float(np.linalg.norm(reached_pos - target))
                trace = float(np.trace(reached_R.T @ R))
                ori_error = math.acos(float(np.clip(
                    (trace - 1.0) * 0.5, -1.0, 1.0)))
                if pos_error >= 0.02 or ori_error >= 0.20:
                    valid = False
                    break
                worst_pos = max(worst_pos, pos_error)
                worst_ori = max(worst_ori, ori_error)
                q_stage = np.asarray(solution)
            if valid:
                # Prefer low residual first, with a small bias toward the
                # requested horizontal pitch so the gripper remains upright.
                score = (10.0 * worst_pos + worst_ori
                         + 0.05 * abs(pitch - requested))
                candidates.append((score, pitch, R, worst_pos, worst_ori))
        if not candidates:
            return None
        _score, pitch, R, pos_error, ori_error = min(
            candidates, key=lambda item: item[0])
        self.get_logger().info(
            f"locked approach orientation pitch={math.degrees(pitch):.1f}° "
            f"(worst residual {pos_error*1000:.1f} mm/"
            f"{math.degrees(ori_error):.1f}°)")
        return pitch, R

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

    def _wait_fresh_object(self, timeout_s: float):
        """Wait for a new stable perception cluster after an arm movement."""
        if self.object_source != "perception":
            return self._object_world(allow_stale=False)
        self._perception_samples.clear()
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while rclpy.ok() and not self._cancel.is_set():
            found = self._object_world(allow_stale=False)
            if found is not None:
                return found
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        return None

    def _visual_center(self, horizontal_only: bool, label: str) -> bool:
        """Centre the live object before the next approach movement.

        This runs only between MoveGroup goals. Pan uses joint1; tilt uses
        joint5 unless ``horizontal_only`` is requested for the final lock.
        """
        timeout = float(
            self.get_parameter("pickup_tracking_timeout_s").value)
        stable_needed = max(1, int(self.get_parameter(
            "pickup_tracking_stable_cycles").value))
        target_timeout = float(
            self.get_parameter("tracking_target_timeout_s").value)
        rate = max(1.0, float(self.get_parameter("tracking_rate_hz").value))
        # Do not replace a trajectory before the controller has had time to
        # reach it. The old 5 Hz loop sent a new 0.30 s goal every 0.20 s, so
        # joint5 only moved a few milliradians per iteration and timed out.
        trajectory_s = float(
            self.get_parameter("tracking_trajectory_s").value)
        period = max(1.0 / rate, trajectory_s + 0.05)
        deadline = time.monotonic() + timeout
        stable = 0
        last_error_x = None
        last_error_y = None
        while rclpy.ok() and not self._cancel.is_set() \
                and time.monotonic() < deadline:
            latest = self._perception_samples[-1] \
                if self._perception_samples else None
            q = self._arm_q()
            if latest is None or time.monotonic() - latest[0] > target_timeout \
                    or q is None:
                stable = 0
                self._status_line = f"{label} — waiting object"
                time.sleep(period)
                continue
            point = latest[1]
            pan_target, error_x, pan_move = image_axis_tracking_target(
                float(q[0]), float(point[0]), float(point[2]),
                float(self.get_parameter("tracking_gain").value),
                float(self.get_parameter("tracking_yaw_sign").value),
                float(self.get_parameter("tracking_deadband_rad").value),
                float(self.get_parameter("tracking_max_step_rad").value),
                float(self.get_parameter("tracking_joint1_min").value),
                float(self.get_parameter("tracking_joint1_max").value),
            )
            tilt_target, error_y, tilt_move = image_axis_tracking_target(
                float(q[4]), float(point[1]), float(point[2]),
                float(self.get_parameter("tracking_vertical_gain").value),
                float(self.get_parameter("tracking_pitch_sign").value),
                float(self.get_parameter(
                    "tracking_vertical_deadband_rad").value),
                float(self.get_parameter(
                    "tracking_vertical_max_step_rad").value),
                float(self.get_parameter("tracking_joint5_min").value),
                float(self.get_parameter("tracking_joint5_max").value),
            )
            last_error_x = error_x
            last_error_y = error_y
            centered = not pan_move and (horizontal_only or not tilt_move)
            stable = stable + 1 if centered else 0
            self._status_line = (
                f"{label} — x={math.degrees(error_x):+.1f}°, "
                f"y={math.degrees(error_y):+.1f}°, stable={stable}/{stable_needed}"
            )
            if stable >= stable_needed:
                self.get_logger().info(self._status_line + " — LOCKED")
                return True
            command = np.asarray(q, dtype=float)
            command[0] = pan_target
            if not horizontal_only:
                command[4] = tilt_target
            if pan_move or (tilt_move and not horizontal_only):
                self._publish_arm_trajectory(command)
            time.sleep(period)
        error_detail = ""
        if last_error_x is not None and last_error_y is not None:
            error_detail = (
                f" (last x={math.degrees(last_error_x):+.1f}°, "
                f"y={math.degrees(last_error_y):+.1f}°)"
            )
        self._status_line = (
            f"{label} FAILED — object not centered{error_detail}")
        self.get_logger().error(self._status_line)
        return False

    def _object_within_safety_bounds(self, obj_w: np.ndarray) -> bool:
        lower = np.asarray(self.get_parameter(
            "perception_world_min").value, dtype=float)
        upper = np.asarray(self.get_parameter(
            "perception_world_max").value, dtype=float)
        return bool(np.all(obj_w >= lower) and np.all(obj_w <= upper))

    # ---------------- sequence ----------------
    def _approach_only(self) -> bool:
        standoff, zoff = self._approach_geom()
        min_z = float(self.get_parameter("min_z").value)
        tracked_pick = bool(self.get_parameter(
            "pickup_tracking_enabled").value) \
            and self.object_source == "perception"

        ready_pose = np.asarray(
            self.get_parameter("ready_pose").value, dtype=float)
        ready_label = "ready"
        if tracked_pick:
            # Preserve the pan/tilt lock from continuous tracking. Resetting
            # joint1/joint5 here made every pickup throw away centering and
            # spend the entire lock timeout trying to recover it.
            current_q = self._arm_q()
            if current_q is None:
                self._status_line = "pickup aborted — joint state unavailable"
                return False
            ready_pose = ready_pose.copy()
            ready_pose[0] = current_q[0]
            tilt_min = float(self.get_parameter(
                "tracking_joint5_min").value)
            tilt_max = float(self.get_parameter(
                "tracking_joint5_max").value)
            # Never carry a previously flipped wrist into another pickup.
            # Preserve a valid camera tilt, but recover to the calibrated
            # ready tilt when the measured joint is outside its safe tracking
            # envelope (observed failure: joint5=2.08 rad while max=1.80).
            if tilt_min <= float(current_q[4]) <= tilt_max:
                ready_pose[4] = current_q[4]
                ready_label = "tracking-ready (preserve safe pan/tilt)"
            else:
                ready_label = "tracking-ready (recover upright tilt)"
                self.get_logger().warn(
                    f"joint5 {current_q[4]:+.3f} outside safe tracking "
                    f"range [{tilt_min:+.3f}, {tilt_max:+.3f}]; "
                    f"recovering to {ready_pose[4]:+.3f}")
        if not self._send_joint_pose(ready_pose, ready_label):
            return False
        if not self._gripper("open", "OPEN"):
            return False

        if tracked_pick and not self._visual_center(
                horizontal_only=False, label="initial pan-tilt lock"):
            return False

        refresh_timeout = float(self.get_parameter(
            "pickup_object_refresh_timeout_s").value)
        found = self._wait_fresh_object(refresh_timeout) \
            if tracked_pick else self._object_world(allow_stale=True)
        if found is None:
            self.get_logger().error("no fresh stable object after tracking")
            self._status_line = "pickup aborted — object lost"
            return False
        obj_w = found[1]
        if not self._object_within_safety_bounds(obj_w):
            self._status_line = (
                "pickup aborted — tracked object outside safety bounds "
                f"{[round(float(value), 3) for value in obj_w]}"
            )
            self.get_logger().error(self._status_line)
            return False

        # Alternate approach and visual correction. Each movement uses the
        # latest 3D object pose; after the movement the target is centered and
        # transformed again before the next, closer standoff.
        if tracked_pick:
            distances = approach_standoff_distances(
                float(self.get_parameter("pickup_coarse_standoff_m").value),
                standoff,
                float(self.get_parameter("pickup_approach_step_m").value),
            )
            raw_distances = list(distances)
            distances = safe_approach_standoff_distances(
                float(np.hypot(obj_w[0], obj_w[1])), distances,
                float(self.get_parameter(
                    "pickup_min_approach_radius_m").value),
            )
            if not distances:
                self._status_line = (
                    "pickup aborted — no collision-safe approach radius")
                self.get_logger().error(self._status_line)
                return False
            skipped = len(raw_distances) - len(distances)
            if skipped:
                self.get_logger().info(
                    f"adaptive approach skipped {skipped} folded waypoint(s); "
                    f"using {[round(value, 3) for value in distances]} m")
            locked_orientation = self._select_locked_approach_orientation(
                obj_w, distances)
            if locked_orientation is None:
                self._status_line = (
                    "pickup aborted — no upright orientation for approach")
                self.get_logger().error(self._status_line)
                return False
            approach_pitch, approach_R = locked_orientation
            self._status_line = (
                f"approach orientation locked at "
                f"{math.degrees(approach_pitch):.1f}°")
            for index, distance in enumerate(distances, start=1):
                bearing = self._bearing(obj_w)
                target = obj_w - distance * bearing
                target[2] = max(obj_w[2] + zoff, min_z)
                if not self._send_continuous_pose(
                        target, approach_R,
                        f"tracked approach {index}/{len(distances)} "
                        f"({distance*100:.0f} cm)"):
                    return False
                final_stage = index == len(distances)
                if not self._visual_center(
                        horizontal_only=final_stage,
                        label=("final left-right lock" if final_stage
                               else f"re-track {index}/{len(distances)}")):
                    return False
                found = self._wait_fresh_object(refresh_timeout)
                if found is None:
                    self._status_line = (
                        "pickup aborted — object lost during approach")
                    self.get_logger().error(self._status_line)
                    return False
                obj_w = found[1]
                if not self._object_within_safety_bounds(obj_w):
                    self._status_line = (
                        "pickup aborted — refreshed object outside safety "
                        f"bounds {[round(float(value), 3) for value in obj_w]}"
                    )
                    self.get_logger().error(self._status_line)
                    return False

        # Final pose is calculated only after the close-range left-right lock.
        reach = self._reachable(obj_w, seed_from_current=tracked_pick)
        if reach is None or not reach["reachable"]:
            detail = "no joint state" if reach is None else str(reach)
            self.get_logger().error(
                f"pick rejected after visual approach: {detail}")
            self._status_line = "pickup aborted — final pose unreachable"
            return False
        pitch = float(reach["pitch"])
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        bearing = self._bearing(obj_w)
        stand = obj_w - standoff * bearing
        stand[2] = max(obj_w[2] + zoff, min_z)
        if not self._send_position_preserve_orientation(
                stand, "final pre-position"):
            return False

        # Hold XYZ while rotating into the selected feasible grasp orientation.
        R = rpy_pose(pitch, yaw)
        self.get_logger().info(
            f"align orientation at pre-position: pitch={math.degrees(pitch):.1f}°")
        stand_q = reach.get("stand_q")
        if stand_q is None:
            self._status_line = "align orientation FAILED — no continuous IK"
            return False
        current_q = self._arm_q()
        if current_q is None:
            self._status_line = "align orientation FAILED — joint state unavailable"
            return False
        align_delta = np.abs(np.asarray(stand_q) - current_q)
        if float(max(align_delta[3], align_delta[5])) > 0.90:
            self._status_line = (
                "align orientation BLOCKED — inverted wrist branch")
            self.get_logger().error(self._status_line)
            return False
        return self._send_joint_pose(
            stand_q, "align orientation (locked IK branch)"
        ), obj_w, yaw, pitch, R

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

        # Stage 3 — ADVANCE (grasp): here — and ONLY here — apply pick_offset so the
        # fingertip lands on the object. _ee_for_gripper puts the GRIPPER at
        # the object centre by offsetting the EE command by pick_offset.
        grasp_pt = self._ee_for_gripper(obj_w.copy(), bearing)
        grasp_pt[2] = max(grasp_pt[2], min_z)
        stand = obj_w - standoff * bearing
        stand[2] = max(obj_w[2] + zoff, min_z)
        step = float(self.get_parameter("advance_step_m").value)
        waypoints = linear_waypoints(stand, grasp_pt, step)
        for index, waypoint in enumerate(waypoints, start=1):
            if not self._send_continuous_pose(
                    waypoint, R,
                    f"advance straight {index}/{len(waypoints)}",
                    max_joint_delta=0.55):
                return False
        if not self._gripper("close", "CLOSE — gripping object"):
            self._status_line = "pickup aborted — gripper failed to close"
            self.get_logger().error(self._status_line)
            return False

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

        # Transfer to the configured drop location while keeping the object
        # clamped.  Never release unless that motion has actually succeeded.
        if not self._send_joint_pose(
                self.get_parameter("place_pose").value,
                "carry object to drop location"):
            self._status_line = (
                "place FAILED — still holding object; release blocked")
            self.get_logger().error(self._status_line)
            return False
        if not self._gripper("open", "RELEASE at drop location"):
            self._status_line = "place FAILED — gripper did not release"
            self.get_logger().error(self._status_line)
            return False
        if not self._send_joint_pose(
                self.get_parameter("ready_pose").value,
                "return ready after release"):
            self._status_line = "object released — return to ready failed"
            self.get_logger().error(self._status_line)
            return False
        self._status_line = "COMPLETE"
        self.get_logger().info("direct pick-and-place COMPLETE")
        return True

    def _track_worker(self) -> None:
        """Centre the target in image X/Y using joint1 pan and joint5 tilt."""
        rate = max(1.0, float(self.get_parameter("tracking_rate_hz").value))
        period = 1.0 / rate
        timeout = float(
            self.get_parameter("tracking_target_timeout_s").value)
        self.get_logger().info(
            "pan-tilt object tracking started (joint1 + joint5 only)")
        try:
            while rclpy.ok() and not self._cancel.is_set():
                now = time.monotonic()
                latest = self._perception_samples[-1] \
                    if self._perception_samples else None
                q = self._arm_q()
                if latest is None or now - latest[0] > timeout:
                    self._status_line = "tracking pan-tilt — object lost, holding"
                    self._tracking_status_pub.publish(String(
                        data="active: object lost, holding"))
                    time.sleep(period)
                    continue
                if q is None:
                    self._status_line = "tracking pan-tilt — waiting joint states"
                    self._tracking_status_pub.publish(String(
                        data="active: waiting joint states"))
                    time.sleep(period)
                    continue

                point = latest[1]
                pan_target, error_x, pan_move = image_axis_tracking_target(
                    current_joint=float(q[0]),
                    optical_axis=float(point[0]),
                    optical_z=float(point[2]),
                    gain=float(self.get_parameter("tracking_gain").value),
                    direction=float(self.get_parameter(
                        "tracking_yaw_sign").value),
                    deadband_rad=float(self.get_parameter(
                        "tracking_deadband_rad").value),
                    max_step_rad=float(self.get_parameter(
                        "tracking_max_step_rad").value),
                    joint_min=float(self.get_parameter(
                        "tracking_joint1_min").value),
                    joint_max=float(self.get_parameter(
                        "tracking_joint1_max").value),
                )
                tilt_target, error_y, tilt_move = image_axis_tracking_target(
                    current_joint=float(q[4]),
                    optical_axis=float(point[1]),
                    optical_z=float(point[2]),
                    gain=float(self.get_parameter(
                        "tracking_vertical_gain").value),
                    direction=float(self.get_parameter(
                        "tracking_pitch_sign").value),
                    deadband_rad=float(self.get_parameter(
                        "tracking_vertical_deadband_rad").value),
                    max_step_rad=float(self.get_parameter(
                        "tracking_vertical_max_step_rad").value),
                    joint_min=float(self.get_parameter(
                        "tracking_joint5_min").value),
                    joint_max=float(self.get_parameter(
                        "tracking_joint5_max").value),
                )
                self._status_line = (
                    f"tracking pan-tilt — error_x={math.degrees(error_x):+.1f}°, "
                    f"error_y={math.degrees(error_y):+.1f}°, "
                    f"j1={q[0]:+.3f}→{pan_target:+.3f}, "
                    f"j5={q[4]:+.3f}→{tilt_target:+.3f}"
                )
                self._tracking_status_pub.publish(String(
                    data="active: " + self._status_line))
                if pan_move or tilt_move:
                    command = np.asarray(q, dtype=float)
                    command[0] = pan_target
                    command[4] = tilt_target
                    self._publish_arm_trajectory(command)
                time.sleep(period)
        finally:
            # Replace any still-running correction with a short hold at the
            # latest measured position. No search motion is issued when lost.
            q = self._arm_q()
            if q is not None:
                self._publish_arm_trajectory(np.asarray(q, dtype=float), 0.10)
            self._status_line = "tracking pan-tilt stopped"
            self._tracking_status_pub.publish(String(data="inactive: stopped"))
            self.get_logger().info(self._status_line)
            with self._run_lock:
                self._busy = False
                self._tracking_mode = False

    def _publish_arm_trajectory(self, positions: np.ndarray,
                                duration_s: Optional[float] = None) -> None:
        """Publish a full six-joint target while the trajectory controller owns the arm."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.arm_joints)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        duration = max(0.05, float(
            self.get_parameter("tracking_trajectory_s").value
            if duration_s is None else duration_s))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self._trajectory_pub.publish(msg)

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
            self._tracking_mode = False
            self._cancel.clear()
        self._worker_thread = threading.Thread(
            target=self._worker, args=(approach_only,), daemon=True)
        self._worker_thread.start()
        return True

    # ---------------- services ----------------
    def _pick_preflight(self):
        with self._run_lock:
            if self._busy and not getattr(self, "_tracking_mode", False):
                return False, "pickup sequence already running"
        tracked_pick = bool(self.get_parameter(
            "pickup_tracking_enabled").value) \
            and getattr(self, "object_source", "apriltag") == "perception"
        if tracked_pick:
            latest = self._perception_samples[-1] \
                if self._perception_samples else None
            max_age = float(self.get_parameter(
                "tracking_target_timeout_s").value)
            if latest is None or time.monotonic() - latest[0] > max_age:
                return False, "no fresh perception target for visual approach"
            depth = float(latest[1][2])
            depth_min = float(self.get_parameter(
                "pickup_optical_depth_min_m").value)
            depth_max = float(self.get_parameter(
                "pickup_optical_depth_max_m").value)
            if not math.isfinite(depth) or not depth_min <= depth <= depth_max:
                return False, f"unsafe optical target depth: {depth:.3f} m"
            if self._arm_q() is None:
                return False, "joint state unavailable"
            return True, (
                f"visual target ready at depth {depth:.3f} m; "
                "final workspace check deferred until centered")
        found = self._object_world(allow_stale=False)
        if found is None:
            return False, "no fresh, stable perception target"
        obj_w = found[1]
        lower = np.array([
            float(value) for value in
            self.get_parameter("perception_world_min").value
        ])
        upper = np.array([
            float(value) for value in
            self.get_parameter("perception_world_max").value
        ])
        if np.any(obj_w < lower) or np.any(obj_w > upper):
            return False, (
                "transformed target outside configured safety bounds: "
                f"point={[round(float(value), 3) for value in obj_w]}"
            )
        reach = self._reachable(obj_w)
        if reach is None:
            return False, "joint state unavailable"
        if not reach["reachable"]:
            return False, (
                f"object outside arm workspace: r={reach['radius']:.3f} m, "
                f"standoff={'ok' if reach['standoff_ok'] else 'failed'}, "
                f"grasp={'ok' if reach['grasp_ok'] else 'failed'}"
            )
        return True, f"target reachable at {reach['obj']}"

    def _on_run(self, req, res):
        ready, detail = self._pick_preflight()
        if not ready:
            res.success = False
            res.message = f"pickup rejected: {detail}"
            return res
        if not self._start(False):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "direct pick started"
        return res

    def _on_approach(self, req, res):
        ready, detail = self._pick_preflight()
        if not ready:
            res.success = False
            res.message = f"approach rejected: {detail}"
            return res
        if not self._start(True):
            res.success = False
            res.message = "sequence already running"
        else:
            res.success = True
            res.message = "approach started"
        return res

    def _on_track(self, req, res):
        if self.object_source != "perception":
            res.success = False
            res.message = "tracking requires object_source=perception"
            return res
        with self._run_lock:
            if self._busy:
                res.success = False
                res.message = "arm busy; stop pickup/tracking first"
                return res
            self._busy = True
            self._tracking_mode = True
            self._cancel.clear()
        self._worker_thread = threading.Thread(
            target=self._track_worker, daemon=True)
        self._worker_thread.start()
        res.success = True
        res.message = "pan-tilt object tracking started (joint1 + joint5)"
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
        found = self._object_world(allow_stale=True)
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
        found = self._object_world(allow_stale=True)
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
        found = self._object_world()
        obj = "no object" if found is None else \
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
