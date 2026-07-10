"""Vision-driven pick-and-place: pick where the AprilTag is.

Level-1 proof of concept on top of the waypoint state machine:

  * `apriltag_detector` publishes the tag pose in the camera optical frame
  * this node transforms it into the arm `world` frame through a camera
    extrinsic (`camera_xyz` / `camera_rpy` params)
  * the object centre is generated from the tag pose + side/top-face offset
  * the PICK poses are generated from that object centre (Cartesian goals)
  * the PLACE poses stay static joint-space waypoints from waypoints.yaml
  * if the tag is not visible yet, the arm sweeps through configured search
    waypoints before giving up

Sequence (same shape as pick_place_node, pick side replaced by vision):

   1. → start_pose            (waypoint)
   2. ↓ gripper OPEN
   3. → above object          (object centre + approach_offset, Cartesian)
   4. → onto object           (object centre + grasp_offset,   Cartesian)
   5. ↓ gripper CLOSE
   6. → above tag             (lift)
   7. → place_above           (waypoint)
   8. → place                 (waypoint)
   9. ↓ gripper OPEN
  10. → place_above
  11. → start_pose
  12. → optional small "circle" gesture

Camera extrinsic convention: by default `camera_xyz`/`camera_rpy` locate the
camera BODY frame (ROS convention: x out of the lens, y left, z up) in
`world` (arm base). If `camera_parent_frame` is set, the same transform is
relative to that moving frame, e.g. `end_effector_link` for a wrist-mounted
RealSense. The fixed body→optical rotation is applied internally. Verify with:

    ros2 service call /tag_world std_srvs/srv/Trigger

which reports where the arm thinks the tag is — put the tag at a spot you
can measure from the arm base and compare before trusting a pick.
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .moveit_client import MoveItClient


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# camera BODY (x fwd, y left, z up) → OPTICAL (x right, y down, z fwd):
# columns are the optical axes expressed in body coordinates.
R_BODY_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


class TagPickPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tag_pick_place")

        # ---- parameters: MoveIt plumbing (mirrors pick_place_node) ----
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("srdf_file", "")
        self.declare_parameter("arm_group", "arm")
        self.declare_parameter("gripper_group", "gripper")
        self.declare_parameter("ee_link", "end_effector_link")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("vel_scale", 0.2)
        self.declare_parameter("acc_scale", 0.2)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("planning_attempts", 20)
        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("orientation_tolerance", 0.5)
        self.declare_parameter("gripper_open_pos", 0.019)
        self.declare_parameter("gripper_close_pos", -0.010)
        self.declare_parameter("auto_run", False)
        self.declare_parameter("auto_run_delay", 8.0)

        # ---- parameters: camera extrinsic ----
        self.declare_parameter("camera_parent_frame", "")
        self.declare_parameter("camera_xyz", [0.25, 0.0, 0.30])
        self.declare_parameter("camera_rpy", [0.0, 0.6, 3.1416])

        # ---- parameters: grasp generation ----
        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("approach_offset", 0.10)   # m above object centre
        self.declare_parameter("cube_size", 0.03)         # 30 mm cube
        # Vector from tag centre to cube centre in the tag frame. With the tag
        # printed face visible, +Z points out of the paper, so the cube centre
        # is 15 mm behind the tag for both side-mounted and top-mounted tags.
        self.declare_parameter("object_center_from_tag", [0.0, 0.0, -0.015])
        self.declare_parameter("grasp_offset", 0.0)       # m above object centre
        self.declare_parameter("grasp_pitch", 3.1416)     # pi = straight down
        self.declare_parameter("grasp_pitch_fallbacks", [3.1416, 2.7, 2.4])
        # hover/approach orientation — separate from the grasp orientation:
        # top-down is only IK-feasible at LOW z (<~0.06 m), the hover height
        # is not, so the approach must be tilted.
        self.declare_parameter("approach_pitch", 2.4)
        self.declare_parameter("approach_pitch_fallbacks", [2.4, 2.2, 2.6, 2.0, 1.8])
        # ---- FRONT pick mode: approach horizontally, refine, then grasp ----
        # IK map 2026-07-09: frontal (pitch pi/2, roll 0) is feasible at
        # object height ~0.20 m for r 0.14-0.30; lower objects need r~0.30
        # or a slightly nose-down pitch. Fingers-vertical (roll ±90°) has NO
        # solutions anywhere, so roll stays 0.
        self.declare_parameter("pick_mode", "front")      # "front" | "top"
        self.declare_parameter("front_standoff", 0.10)    # m in front of object
        self.declare_parameter("front_approach_z_offset", 0.04)  # m above object z
        self.declare_parameter("front_grasp_back", 0.005) # m EE stops short of centre
        self.declare_parameter("front_pitch", 1.5708)     # pi/2 = horizontal
        self.declare_parameter("front_pitch_fallbacks", [1.5708, 1.72, 1.87, 2.0])
        self.declare_parameter("front_refine_max_iters", 4)
        self.declare_parameter("front_refine_tolerance", 0.008)  # stop when move < this
        self.declare_parameter("front_refine_pos_tol", 0.006)    # MoveIt goal sphere
        self.declare_parameter("front_refine_settle", 0.6)       # s wait for fresh tag
        self.declare_parameter("approach_extra_heights", [0.0, 0.04, 0.08])
        self.declare_parameter("approach_orientation_tolerance", 0.9)
        self.declare_parameter("grasp_orientation_tolerance", 0.7)
        self.declare_parameter("yaw_offset", 0.0)         # added to radial yaw
        self.declare_parameter("min_z", 0.015)            # floor clamp (m)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("max_sample_age", 1.5)     # s
        self.declare_parameter("stabilize_before_pick", True)
        self.declare_parameter("stabilize_timeout", 4.0)
        self.declare_parameter("stabilize_duration", 0.8)
        self.declare_parameter("stabilize_required_samples", 10)
        self.declare_parameter("stabilize_position_tolerance", 0.02)

        # ---- parameters: search / feedback gestures ----
        self.declare_parameter("prepare_before_search", True)
        self.declare_parameter("prepare_waypoint", "start_pose")
        self.declare_parameter("open_gripper_before_search", True)
        self.declare_parameter("search_enabled", True)
        self.declare_parameter("search_timeout", 18.0)
        self.declare_parameter("search_hold_sec", 0.6)
        self.declare_parameter("search_waypoints", [
            "search_center",
            "search_left",
            "search_right",
            "search_high",
            "search_low",
            "search_center",
        ])
        self.declare_parameter("not_found_nod_enabled", True)
        self.declare_parameter("not_found_nod_waypoints", [
            "nod_down",
            "nod_up",
            "nod_down",
            "nod_up",
            "search_center",
        ])
        self.declare_parameter("success_circle_enabled", True)
        self.declare_parameter("success_circle_waypoints", [
            "circle_1",
            "circle_2",
            "circle_3",
            "circle_4",
            "circle_5",
        ])

        # ---- waypoints + SRDF named poses (for start/place side) ----
        wp_path = str(self.get_parameter("waypoints_file").value)
        if not wp_path:
            wp_path = str(
                Path(get_package_share_directory("om_chain_pick_place"))
                / "config" / "waypoints.yaml"
            )
        with open(wp_path) as f:
            data = yaml.safe_load(f) or {}
        self.waypoints: Dict[str, Any] = data.get("waypoints", data)
        self.get_logger().info(f"waypoints from {wp_path}: {list(self.waypoints)}")

        self.named_poses: Dict[str, Dict[str, float]] = {}
        srdf_path = str(self.get_parameter("srdf_file").value)
        if not srdf_path:
            # Canonical SRDF = open_manipulator_6dof, whose planning tip is the
            # gripper end-effector link.
            try:
                srdf_path = str(
                    Path(get_package_share_directory("open_manipulator_6dof_moveit"))
                    / "config" / "open_manipulator_6dof.srdf"
                )
            except Exception:
                srdf_path = ""
        if srdf_path:
            self._load_srdf(srdf_path)

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
            position_tolerance=float(self.get_parameter("position_tolerance").value),
            orientation_tolerance=float(self.get_parameter("orientation_tolerance").value),
        )
        self.grip_open = float(self.get_parameter("gripper_open_pos").value)
        self.grip_close = float(self.get_parameter("gripper_close_pos").value)

        # ---- camera extrinsic ----
        cam_xyz = [float(v) for v in self.get_parameter("camera_xyz").value]
        cam_rpy = [float(v) for v in self.get_parameter("camera_rpy").value]
        self.camera_parent_frame = str(self.get_parameter("camera_parent_frame").value)
        self._t_wc = np.array(cam_xyz)
        self._R_parent_body = rpy_to_matrix(*cam_rpy)
        self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if self.camera_parent_frame:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            frame_note = f"relative to {self.camera_parent_frame}"
        else:
            frame_note = "in world"
        self.get_logger().info(
            f"camera body frame @ xyz={cam_xyz} rpy={cam_rpy} ({frame_note})"
        )
        # live extrinsic updates (e.g. from calib_gui via `ros2 param set`)
        self.add_on_set_parameters_callback(self._on_extrinsic_params)

        # ---- tag sample buffer ----
        self._samples: deque = deque(maxlen=60)   # (time, p_optical, q_optical_tag)
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_topic").value),
            self._on_tag, 10, callback_group=self._cb_group,
        )

        # ---- worker plumbing ----
        self._run_lock = threading.Lock()
        self._sequence_busy = False
        self._auto_run_fired = False

        self.create_service(
            Trigger, "/run_tag_pick", self._on_run, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/tag_world", self._on_tag_world, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/tag_pick_status", self._on_status, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/run_search", self._on_run_search,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/run_front_approach", self._on_run_front_approach,
            callback_group=self._cb_group,
        )

        # ---- RViz markers: where the arm thinks the tag / pick poses are ----
        self._marker_pub = self.create_publisher(MarkerArray, "/tag_markers", 2)
        self.create_timer(0.5, self._publish_markers, callback_group=self._cb_group)

        if bool(self.get_parameter("auto_run").value):
            delay = float(self.get_parameter("auto_run_delay").value)
            self.create_timer(delay, self._auto_run_once, callback_group=self._cb_group)
            self.get_logger().info(
                f"auto_run=true — sequence will start in {delay:.1f} s"
            )

        self.get_logger().info(
            "ready — check extrinsic with /tag_world, then /run_tag_pick"
        )

    # ---------------- live extrinsic updates ----------------
    def _on_extrinsic_params(self, params) -> SetParametersResult:
        for p in params:
            try:
                if p.name == "camera_xyz":
                    self._t_wc = np.array([float(v) for v in p.value])
                elif p.name == "camera_rpy":
                    self._R_parent_body = rpy_to_matrix(*[float(v) for v in p.value])
                    self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
                else:
                    continue
                self.get_logger().info(
                    f"extrinsic updated: {p.name} = "
                    f"{[round(float(v), 5) for v in p.value]}")
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=f"{p.name}: {exc}")
        return SetParametersResult(successful=True)

    # ---------------- RViz markers ----------------
    def _make_marker(self, mid: int, mtype: int, pos, scale, rgba) -> Marker:
        m = Marker()
        m.header.frame_id = str(self.get_parameter("reference_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "tag_pick"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(pos[0]), float(pos[1]), float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime = Duration(seconds=1.5).to_msg()
        return m

    def _publish_markers(self) -> None:
        pose = self._tag_object_world()
        if pose is None:
            return
        tag, obj, _R_wt = pose
        approach = float(self.get_parameter("approach_offset").value)
        grasp = float(self.get_parameter("grasp_offset").value)
        cube = float(self.get_parameter("cube_size").value)
        arr = MarkerArray()
        # the cube centre is derived from the measured tag pose.
        arr.markers.append(self._make_marker(
            0, Marker.CUBE, obj,
            (cube, cube, cube), (0.1, 0.9, 0.1, 0.8)))
        # grasp target for the EE
        arr.markers.append(self._make_marker(
            1, Marker.SPHERE, obj + [0, 0, grasp],
            (0.015, 0.015, 0.015), (0.9, 0.2, 0.2, 0.9)))
        # approach (hover) point
        arr.markers.append(self._make_marker(
            2, Marker.SPHERE, obj + [0, 0, approach],
            (0.02, 0.02, 0.02), (0.2, 0.4, 1.0, 0.9)))
        # label with coordinates
        label = self._make_marker(
            3, Marker.TEXT_VIEW_FACING, obj + [0, 0, approach + 0.05],
            (0.0, 0.0, 0.025), (1.0, 1.0, 1.0, 1.0))
        label.text = (
            f"tag ({tag[0]:+.2f}, {tag[1]:+.2f}, {tag[2]:+.2f}) "
            f"obj ({obj[0]:+.2f}, {obj[1]:+.2f}, {obj[2]:+.2f})"
        )
        arr.markers.append(label)
        self._marker_pub.publish(arr)

    # ---------------- SRDF ----------------
    def _load_srdf(self, path: str) -> None:
        try:
            with open(path) as f:
                root = ET.fromstring(f.read())
        except (FileNotFoundError, ET.ParseError) as exc:
            self.get_logger().error(f"SRDF read/parse error ({path}): {exc}")
            return
        for gs in root.findall("group_state"):
            name = gs.get("name")
            if not name:
                continue
            joints = {
                j.get("name"): float(j.get("value"))
                for j in gs.findall("joint")
                if j.get("name") is not None and j.get("value") is not None
            }
            if joints:
                self.named_poses[name] = joints

    # ---------------- tag geometry ----------------
    def _on_tag(self, msg: PoseStamped) -> None:
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        self._samples.append((time.monotonic(), p, q))

    def _camera_reference_optical(self):
        if not self.camera_parent_frame:
            return self._t_wc, self._R_parent_optical, None
        if self._tf_buffer is None:
            return None, None, "camera_parent_frame needs TF, but TF is disabled"
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter("reference_frame").value),
                self.camera_parent_frame,
                Time(),
            )
        except TransformException as exc:
            return None, None, str(exc)
        t = tf.transform.translation
        q = tf.transform.rotation
        t_ref_parent = np.array([t.x, t.y, t.z])
        R_ref_parent = quat_to_matrix(q.x, q.y, q.z, q.w)
        t_ref_optical = t_ref_parent + R_ref_parent @ self._t_wc
        R_ref_optical = R_ref_parent @ self._R_parent_optical
        return t_ref_optical, R_ref_optical, None

    def _tag_object_world(self):
        """Return (tag centre, object centre, tag rotation) in world frame."""
        now = time.monotonic()
        max_age = float(self.get_parameter("max_sample_age").value)
        samples = [s for s in self._samples if now - s[0] <= max_age]
        if len(samples) < int(self.get_parameter("min_samples").value):
            return None
        pts = [p for (_t, p, _q) in samples]
        p_opt = np.median(np.stack(pts), axis=0)
        q_latest = samples[-1][2]
        R_ot = quat_to_matrix(
            float(q_latest[0]), float(q_latest[1]),
            float(q_latest[2]), float(q_latest[3]),
        )
        t_ref_optical, R_ref_optical, tf_error = self._camera_reference_optical()
        if tf_error is not None:
            self.get_logger().warn(
                f"cannot transform tag to world: {tf_error}",
                throttle_duration_sec=2.0,
            )
            return None
        R_wt = R_ref_optical @ R_ot
        tag_w = R_ref_optical @ p_opt + t_ref_optical
        tag_to_obj = np.array(
            [float(v) for v in self.get_parameter("object_center_from_tag").value]
        )
        obj_w = tag_w + R_wt @ tag_to_obj
        return tag_w, obj_w, R_wt

    def _status_report(self) -> str:
        now = time.monotonic()
        max_age = float(self.get_parameter("max_sample_age").value)
        recent = [s for s in self._samples if now - s[0] <= max_age]
        lines = [
            f"busy: {self._sequence_busy}",
            f"auto_run_fired: {self._auto_run_fired}",
            f"camera_parent_frame: {self.camera_parent_frame or '(static world)'}",
            f"recent tag samples: {len(recent)} / required {int(self.get_parameter('min_samples').value)}",
        ]
        if self._samples:
            age = now - self._samples[-1][0]
            p = self._samples[-1][1]
            lines.append(
                f"last detector tag_xyz camera: x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} m age={age:.2f}s"
            )
        else:
            lines.append("last detector tag_xyz camera: none")

        _t_ref_optical, _R_ref_optical, tf_error = self._camera_reference_optical()
        if tf_error is None:
            lines.append("camera TF: ok")
        else:
            lines.append(f"camera TF: unavailable: {tf_error}")

        pose = self._tag_object_world()
        if pose is None:
            lines.append("world object: unavailable")
        else:
            tag_w, obj_w, _R_wt = pose
            grasp = float(self.get_parameter("grasp_offset").value)
            min_z = float(self.get_parameter("min_z").value)
            pick_z = max(float(obj_w[2] + grasp), min_z)
            lines.append(
                f"world tag:    x={tag_w[0]:+.3f} y={tag_w[1]:+.3f} z={tag_w[2]:+.3f} m"
            )
            lines.append(
                f"world object: x={obj_w[0]:+.3f} y={obj_w[1]:+.3f} z={obj_w[2]:+.3f} m"
            )
            lines.append(
                f"pickup target: z={pick_z:+.3f} m "
                f"(object_z {obj_w[2]:+.3f} + grasp_offset {grasp:+.3f}, min_z {min_z:+.3f})"
            )
        return "\n".join(lines)

    # ---------------- waypoint helper (start/place side) ----------------
    def _go_waypoint(self, name: str) -> bool:
        if name not in self.waypoints:
            self.get_logger().error(f"Unknown waypoint '{name}' in waypoints.yaml")
            return False
        wp = self.waypoints[name]
        if isinstance(wp, str):
            return self.client.move_to_named_pose(wp, named_poses=self.named_poses)
        if isinstance(wp, dict):
            if "named" in wp:
                return self.client.move_to_named_pose(
                    str(wp["named"]), named_poses=self.named_poses)
            if "joint" in wp:
                return self.client.move_to_joint_values(
                    [float(x) for x in wp["joint"]])
            if "xyz" in wp:
                xyz = [float(x) for x in wp["xyz"]]
                rpy = [float(x) for x in wp.get("rpy", [0.0, 0.0, 0.0])]
                return self.client.move_to_xyz_rpy(*xyz, *rpy)
        self.get_logger().error(f"Waypoint '{name}' has invalid format: {wp}")
        return False

    def _string_list_param(self, name: str) -> list:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        return [str(v) for v in value]

    def _wait_for_tag_pose(self, timeout_sec: float):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok():
            pose = self._tag_object_world()
            if pose is not None:
                return pose
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

    def _wait_for_stable_tag_pose(self):
        if not bool(self.get_parameter("stabilize_before_pick").value):
            return self._tag_object_world()

        timeout = float(self.get_parameter("stabilize_timeout").value)
        duration = float(self.get_parameter("stabilize_duration").value)
        required = int(self.get_parameter("stabilize_required_samples").value)
        tolerance = float(self.get_parameter("stabilize_position_tolerance").value)
        deadline = time.monotonic() + max(0.0, timeout)
        window = []
        last_report = 0.0

        self.get_logger().info(
            f"stabilizing tag pose for pickup: {duration:.1f}s window, "
            f"{required} samples, tolerance {tolerance * 1000:.0f} mm"
        )
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            pose = self._tag_object_world()
            if pose is None:
                window.clear()
                if now - last_report > 1.0:
                    last_report = now
                    self.get_logger().info("stabilizing: waiting for recent tag samples")
                time.sleep(0.05)
                continue

            window.append((now, pose))
            window = [(t, p) for (t, p) in window if now - t <= duration]
            if len(window) >= required and window[-1][0] - window[0][0] >= duration * 0.8:
                tags = np.stack([p[0] for (_t, p) in window])
                objs = np.stack([p[1] for (_t, p) in window])
                spread = np.ptp(objs, axis=0)
                max_spread = float(np.max(np.abs(spread)))
                if max_spread <= tolerance:
                    stable_tag = np.median(tags, axis=0)
                    stable_obj = np.median(objs, axis=0)
                    stable_R = window[-1][1][2]
                    self.get_logger().info(
                        "stable tag pose locked: "
                        f"object=({stable_obj[0]:+.3f}, {stable_obj[1]:+.3f}, {stable_obj[2]:+.3f}) "
                        f"spread={max_spread * 1000:.0f} mm"
                    )
                    return stable_tag, stable_obj, stable_R
                if now - last_report > 1.0:
                    last_report = now
                    self.get_logger().info(
                        f"stabilizing: object spread {max_spread * 1000:.0f} mm"
                    )
            time.sleep(0.05)

        self.get_logger().error("tag pose did not stabilize enough for pickup")
        return None

    def _run_waypoint_gesture(self, param_name: str, label: str) -> bool:
        names = self._string_list_param(param_name)
        if not names:
            self.get_logger().warn(f"{label}: no waypoints configured")
            return False
        ok = True
        for name in names:
            self.get_logger().info(f"{label}: waypoint '{name}'")
            if not self._go_waypoint(name):
                self.get_logger().warn(f"{label}: waypoint '{name}' failed")
                ok = False
        return ok

    def _find_tag_with_search(self):
        pose = self._wait_for_tag_pose(0.5)
        if pose is not None:
            return pose
        if not bool(self.get_parameter("search_enabled").value):
            return None

        timeout = float(self.get_parameter("search_timeout").value)
        hold = float(self.get_parameter("search_hold_sec").value)
        search_names = self._string_list_param("search_waypoints")
        deadline = time.monotonic() + timeout
        self.get_logger().info(
            f"tag not visible yet — searching for up to {timeout:.1f} s"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            for name in search_names:
                pose = self._wait_for_tag_pose(hold)
                if pose is not None:
                    self.get_logger().info("tag found during search")
                    return pose
                self.get_logger().info(f"search sweep → '{name}'")
                self._go_waypoint(name)
                pose = self._wait_for_tag_pose(hold)
                if pose is not None:
                    self.get_logger().info("tag found during search")
                    return pose
                if time.monotonic() >= deadline:
                    break
        return None

    def _prepare_for_search(self) -> bool:
        if not bool(self.get_parameter("prepare_before_search").value):
            return True
        waypoint = str(self.get_parameter("prepare_waypoint").value)
        self.get_logger().info(
            f"prepare: moving to initial waypoint '{waypoint}'"
        )
        if not self._go_waypoint(waypoint):
            self.get_logger().error("prepare: failed to reach initial waypoint")
            return False
        if bool(self.get_parameter("open_gripper_before_search").value):
            self.get_logger().info("prepare: opening gripper")
            if not self.client.set_gripper(self.grip_open):
                self.get_logger().error("prepare: failed to open gripper")
                return False
        return True

    def _float_list_param(self, name: str) -> list:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, (float, int)):
            return [float(value)]
        return [float(v) for v in value]

    def _pitch_candidates(
        self, first_pitch: float,
        fallback_param: str = "grasp_pitch_fallbacks",
    ) -> list:
        candidates = [float(first_pitch)]
        for value in self._float_list_param(fallback_param):
            if all(abs(value - existing) > 1e-4 for existing in candidates):
                candidates.append(value)
        return candidates

    def _move_xyz_with_fallback(
        self,
        target: np.ndarray,
        comment: str,
        yaw: float,
        first_pitch: float,
        extra_heights=None,
        orientation_tolerance: Optional[float] = None,
        fallback_param: str = "grasp_pitch_fallbacks",
    ):
        extras = [0.0] if extra_heights is None else [float(v) for v in extra_heights]
        ori_tol = (
            float(self.get_parameter("orientation_tolerance").value)
            if orientation_tolerance is None
            else float(orientation_tolerance)
        )
        pos_tol = float(self.get_parameter("position_tolerance").value)
        for extra in extras:
            p = target + np.array([0.0, 0.0, extra])
            for candidate_pitch in self._pitch_candidates(
                    first_pitch, fallback_param):
                self.get_logger().info(
                    f"{comment} try ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) "
                    f"pitch={math.degrees(candidate_pitch):.0f} deg "
                    f"ori_tol={ori_tol:.2f}"
                )
                if self.client.move_to_xyz_rpy(
                    float(p[0]), float(p[1]), float(p[2]),
                    0.0, candidate_pitch, yaw,
                    position_tolerance=pos_tol,
                    orientation_tolerance=ori_tol,
                ):
                    return True, p, candidate_pitch
        return False, target, first_pitch

    # ---------------- FRONT pick sequence ----------------
    def _front_target(self, obj_w: np.ndarray, back: float) -> np.ndarray:
        """Point `back` metres behind the object centre along the horizontal
        base→object bearing (i.e. between the arm and the object)."""
        d = np.array([obj_w[0], obj_w[1], 0.0])
        n = np.linalg.norm(d)
        if n < 1e-6:
            return np.array([obj_w[0], obj_w[1], obj_w[2]])
        d /= n
        return np.array([obj_w[0] - back * d[0], obj_w[1] - back * d[1], obj_w[2]])

    def _front_approach(self, obj_w: np.ndarray):
        """Steps 1-3 of the front pick: start pose, open gripper, move to
        the standoff point in front of the object. Returns
        (ok, approach_point, used_pitch, yaw)."""
        standoff = float(self.get_parameter("front_standoff").value)
        pitch = float(self.get_parameter("front_pitch").value)
        min_z = float(self.get_parameter("min_z").value)
        approach_z_offset = float(
            self.get_parameter("front_approach_z_offset").value)

        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        approach = self._front_target(obj_w, standoff)
        approach[2] = max(approach[2] + approach_z_offset, min_z)

        self.get_logger().info(
            f"FRONT approach: object=({obj_w[0]:+.3f}, {obj_w[1]:+.3f}, "
            f"{obj_w[2]:+.3f}) standoff {standoff*100:.0f} cm, "
            f"z_offset {approach_z_offset*100:.0f} cm, "
            f"yaw={math.degrees(yaw):.0f}°"
        )

        if not self._go_waypoint("start_pose"):
            return False, approach, pitch, yaw
        if not self.client.set_gripper(self.grip_open):
            return False, approach, pitch, yaw

        ok, _p, used_pitch = self._move_xyz_with_fallback(
            approach, "[3] front standoff", yaw, pitch,
            extra_heights=[0.0, 0.03, 0.06],
            orientation_tolerance=float(
                self.get_parameter("approach_orientation_tolerance").value),
            fallback_param="front_pitch_fallbacks",
        )
        if not ok:
            self.get_logger().error("front standoff FAILED")
        return ok, approach, used_pitch, yaw

    def _run_sequence_front(self, tag_w: np.ndarray, obj_w: np.ndarray) -> bool:
        standoff = float(self.get_parameter("front_standoff").value)
        grasp_back = float(self.get_parameter("front_grasp_back").value)
        refine_iters = int(self.get_parameter("front_refine_max_iters").value)
        refine_tol = float(self.get_parameter("front_refine_tolerance").value)
        refine_pos_tol = float(self.get_parameter("front_refine_pos_tol").value)
        settle = float(self.get_parameter("front_refine_settle").value)
        min_z = float(self.get_parameter("min_z").value)
        approach_z_offset = float(
            self.get_parameter("front_approach_z_offset").value)

        ok, approach, used_pitch, yaw = self._front_approach(obj_w)
        if not ok:
            return False

        # 4: PRECISION REFINE — remeasure the object with the wrist camera
        # (it looks straight at it now) and correct the standoff pose with
        # tight tolerances until the correction is below refine_tol.
        for i in range(refine_iters):
            time.sleep(settle)  # let fresh detections accumulate
            fresh = self._tag_object_world()
            if fresh is None:
                self.get_logger().warn(
                    f"[4] refine {i+1}: tag hilang dari kamera — pakai "
                    "estimasi terakhir")
                break
            _tw, obj_w, _R = fresh
            yaw = math.atan2(obj_w[1], obj_w[0]) + \
                float(self.get_parameter("yaw_offset").value)
            target = self._front_target(obj_w, standoff)
            target[2] = max(target[2] + approach_z_offset, min_z)
            delta = float(np.linalg.norm(target - approach))
            self.get_logger().info(
                f"[4] refine {i+1}/{refine_iters}: object="
                f"({obj_w[0]:+.3f}, {obj_w[1]:+.3f}, {obj_w[2]:+.3f}) "
                f"koreksi {delta*1000:.1f} mm"
            )
            if delta < refine_tol:
                self.get_logger().info("[4] sudah sejajar — refine selesai")
                break
            if not self.client.move_to_xyz_rpy(
                float(target[0]), float(target[1]), float(target[2]),
                0.0, used_pitch, yaw,
                position_tolerance=refine_pos_tol,
                orientation_tolerance=0.3,
            ):
                self.get_logger().warn(
                    f"[4] refine {i+1}: gerakan koreksi gagal — lanjut "
                    "dengan pose sekarang")
                break
            approach = target

        # 5: advance straight in to grasp depth
        grasp_pt = self._front_target(obj_w, grasp_back)
        grasp_pt[2] = max(grasp_pt[2], min_z)
        self.get_logger().info(
            f"[5] maju ke objek ({grasp_pt[0]:+.3f}, {grasp_pt[1]:+.3f}, "
            f"{grasp_pt[2]:+.3f})")
        if not self.client.move_to_xyz_rpy(
            float(grasp_pt[0]), float(grasp_pt[1]), float(grasp_pt[2]),
            0.0, used_pitch, yaw,
            position_tolerance=refine_pos_tol,
            orientation_tolerance=float(
                self.get_parameter("grasp_orientation_tolerance").value),
        ):
            self.get_logger().error("[5] advance FAILED — aborting")
            return False

        # 6: grasp
        if not self.client.set_gripper(self.grip_close):
            return False

        # 7: retreat back to standoff, then lift a bit
        retreat = self._front_target(obj_w, standoff)
        retreat[2] = max(retreat[2] + 0.05, min_z)
        self.get_logger().info("[7] mundur + angkat")
        if not self.client.move_to_xyz_rpy(
            float(retreat[0]), float(retreat[1]), float(retreat[2]),
            0.0, used_pitch, yaw,
            position_tolerance=float(
                self.get_parameter("position_tolerance").value),
            orientation_tolerance=float(
                self.get_parameter("approach_orientation_tolerance").value),
        ):
            self.get_logger().error("[7] retreat FAILED — aborting")
            return False

        # 8-11: place via static waypoints, release, return
        for wp in ["place_above", "place"]:
            if not self._go_waypoint(wp):
                self.get_logger().error(f"waypoint '{wp}' FAILED — aborting")
                return False
        if not self.client.set_gripper(self.grip_open):
            return False
        for wp in ["place_above", "start_pose"]:
            if not self._go_waypoint(wp):
                return False
        self.get_logger().info("FRONT pick-and-place COMPLETE")
        return True

    # ---------------- sequence ----------------
    def _run_sequence(self, tag_w: np.ndarray, obj_w: np.ndarray) -> bool:
        if str(self.get_parameter("pick_mode").value).lower() == "front":
            return self._run_sequence_front(tag_w, obj_w)
        approach = float(self.get_parameter("approach_offset").value)
        grasp = float(self.get_parameter("grasp_offset").value)
        # hover (tinggi) hanya feasible dengan pitch miring; grasp (rendah,
        # z<~0.06) justru bisa tegak lurus — lihat peta IK di tag_pick.yaml
        hover_pitch = float(self.get_parameter("approach_pitch").value)
        grasp_pitch = float(self.get_parameter("grasp_pitch").value)
        yaw = math.atan2(obj_w[1], obj_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        min_z = float(self.get_parameter("min_z").value)

        above = obj_w + np.array([0.0, 0.0, approach])
        onto = obj_w + np.array([0.0, 0.0, grasp])
        onto[2] = max(onto[2], min_z)

        self.get_logger().info(
            f"tag@world=({tag_w[0]:+.3f}, {tag_w[1]:+.3f}, {tag_w[2]:+.3f})  "
            f"object=({obj_w[0]:+.3f}, {obj_w[1]:+.3f}, {obj_w[2]:+.3f})  "
            f"yaw={math.degrees(yaw):.0f}° hover pitch="
            f"{math.degrees(hover_pitch):.0f}° grasp pitch="
            f"{math.degrees(grasp_pitch):.0f}°"
        )
        self.get_logger().info(
            f"pickup z target={onto[2]:+.3f} "
            f"(object_z={obj_w[2]:+.3f} + grasp_offset={grasp:+.3f}, min_z={min_z:+.3f})"
        )

        steps = [
            ("wp", "start_pose"),
            ("grip", self.grip_open, "OPEN"),
            ("xyz", above, "above object"),
            ("xyz", onto, "onto object"),
            ("grip", self.grip_close, "CLOSE (grasp)"),
            ("xyz", above, "lift"),
            ("wp", "place_above"),
            ("wp", "place"),
            ("grip", self.grip_open, "OPEN (release)"),
            ("wp", "place_above"),
            ("wp", "start_pose"),
        ]
        for i, step in enumerate(steps, start=1):
            kind = step[0]
            label = f"[{i}/{len(steps)}]"
            if kind == "wp":
                self.get_logger().info(f"{label} waypoint '{step[1]}'")
                if not self._go_waypoint(step[1]):
                    self.get_logger().error(f"{label} FAILED — aborting")
                    return False
            elif kind == "xyz":
                p, comment = step[1], step[2]
                if comment == "lift":
                    p = above
                self.get_logger().info(
                    f"{label} {comment} ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"
                )
                if comment == "above object":
                    ok, used_point, used_pitch = self._move_xyz_with_fallback(
                        p,
                        f"{label} {comment}",
                        yaw,
                        hover_pitch,
                        extra_heights=self._float_list_param("approach_extra_heights"),
                        orientation_tolerance=float(
                            self.get_parameter("approach_orientation_tolerance").value
                        ),
                        fallback_param="approach_pitch_fallbacks",
                    )
                    if ok:
                        above = used_point
                        hover_pitch = used_pitch
                elif comment == "onto object":
                    ok, _used_point, used_pitch = self._move_xyz_with_fallback(
                        p,
                        f"{label} {comment}",
                        yaw,
                        grasp_pitch,
                        extra_heights=[0.0],
                        orientation_tolerance=float(
                            self.get_parameter("grasp_orientation_tolerance").value
                        ),
                        fallback_param="grasp_pitch_fallbacks",
                    )
                    if ok:
                        grasp_pitch = used_pitch
                else:
                    ok, _used_point, used_pitch = self._move_xyz_with_fallback(
                        p,
                        f"{label} {comment}",
                        yaw,
                        hover_pitch,
                        extra_heights=[0.0],
                        orientation_tolerance=float(
                            self.get_parameter("approach_orientation_tolerance").value
                        ),
                        fallback_param="approach_pitch_fallbacks",
                    )
                    if ok:
                        hover_pitch = used_pitch
                if not ok:
                    self.get_logger().error(f"{label} FAILED — aborting")
                    return False
            elif kind == "grip":
                pos, comment = step[1], step[2]
                self.get_logger().info(f"{label} gripper {comment} ({pos:+.4f})")
                if not self.client.set_gripper(pos):
                    self.get_logger().error(f"{label} FAILED — aborting")
                    return False
        self.get_logger().info("Tag pick-and-place COMPLETE")
        return True

    def _worker_body(self, mode: str = "full") -> None:
        search_only = (mode == "search")
        try:
            if not self.client.wait_for_servers(timeout_sec=30.0):
                self.get_logger().error("Required action servers not available")
                return

            pose = self._wait_for_tag_pose(0.8)
            if pose is not None:
                self.get_logger().info("tag already visible — using current global object pose")
            else:
                if not self._prepare_for_search():
                    return
                pose = self._find_tag_with_search()
            if pose is None:
                self.get_logger().error("tag not found after search")
                if bool(self.get_parameter("not_found_nod_enabled").value):
                    self._run_waypoint_gesture(
                        "not_found_nod_waypoints", "not-found nod"
                    )
                return

            if search_only:
                tag_w, obj_w, _R_wt = pose
                self.get_logger().info(
                    "SEARCH-ONLY selesai: tag ditemukan, object@world="
                    f"({obj_w[0]:+.3f}, {obj_w[1]:+.3f}, {obj_w[2]:+.3f}) "
                    "— arm berhenti di pose search ini"
                )
                return

            pose = self._wait_for_stable_tag_pose()
            if pose is None:
                if bool(self.get_parameter("not_found_nod_enabled").value):
                    self._run_waypoint_gesture(
                        "not_found_nod_waypoints", "unstable-tag nod"
                    )
                return

            tag_w, obj_w, _R_wt = pose
            if mode == "front_approach":
                ok = self._front_approach(obj_w)[0]
                if ok:
                    self.get_logger().info(
                        "FRONT APPROACH selesai — arm berhenti di depan "
                        "objek (tanpa refine/grasp). Cek posisi lalu "
                        "/run_tag_pick untuk sekuens penuh.")
                return
            ok = self._run_sequence(tag_w, obj_w)
            if ok and bool(self.get_parameter("success_circle_enabled").value):
                self._run_waypoint_gesture(
                    "success_circle_waypoints", "success circle"
                )
        finally:
            with self._run_lock:
                self._sequence_busy = False

    def _start_worker(self, mode: str = "full") -> bool:
        with self._run_lock:
            if self._sequence_busy:
                return False
            self._sequence_busy = True
        threading.Thread(
            target=self._worker_body, args=(mode,), daemon=True,
        ).start()
        return True

    def _auto_run_once(self) -> None:
        if self._auto_run_fired:
            return
        self._auto_run_fired = True
        if self._start_worker():
            self.get_logger().info("auto_run started")
        else:
            self.get_logger().warn("auto_run skipped: sequence already running")

    # ---------------- services ----------------
    def _on_run(self, req: Trigger.Request, res: Trigger.Response):
        if not self._start_worker():
            res.success = False
            res.message = "sequence already running"
            return res
        res.success = True
        res.message = (
            "started — moving to initial pose, opening gripper, then searching"
        )
        return res

    def _on_run_search(self, req: Trigger.Request, res: Trigger.Response):
        if not self._start_worker(mode="search"):
            res.success = False
            res.message = "sequence already running"
            return res
        res.success = True
        res.message = (
            "search started — arm menyapu waypoint search sampai tag terlihat "
            "(tanpa pick); hasil di log / /tag_world"
        )
        return res

    def _on_run_front_approach(self, req: Trigger.Request, res: Trigger.Response):
        if not self._start_worker(mode="front_approach"):
            res.success = False
            res.message = "sequence already running"
            return res
        res.success = True
        res.message = (
            "front approach started — arm menuju titik "
            f"{float(self.get_parameter('front_standoff').value)*100:.0f} cm "
            "di depan objek, lalu berhenti (tanpa refine/grasp)"
        )
        return res

    def _on_tag_world(self, req: Trigger.Request, res: Trigger.Response):
        pose = self._tag_object_world()
        if pose is None:
            res.success = False
            res.message = "no recent tag detections — is the tag in view?"
            return res
        tag_w, obj_w, _R_wt = pose
        res.success = True
        res.message = (
            f"tag in world frame: x={tag_w[0]:+.3f} y={tag_w[1]:+.3f} "
            f"z={tag_w[2]:+.3f}; object centre: x={obj_w[0]:+.3f} "
            f"y={obj_w[1]:+.3f} z={obj_w[2]:+.3f} "
            "(measure from arm base and compare)"
        )
        return res

    def _on_status(self, req: Trigger.Request, res: Trigger.Response):
        res.success = True
        res.message = self._status_report()
        return res


def main(args=None):
    rclpy.init(args=args)
    node: Optional[TagPickPlaceNode] = None
    try:
        node = TagPickPlaceNode()
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
