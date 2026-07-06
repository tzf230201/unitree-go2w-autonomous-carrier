"""Vision-driven pick-and-place: pick where the AprilTag is.

Level-1 proof of concept on top of the waypoint state machine:

  * `apriltag_detector` publishes the tag pose in the camera optical frame
  * this node transforms it into the arm `world` frame through a static,
    hand-measured camera extrinsic (`camera_xyz` / `camera_rpy` params)
  * the PICK poses are generated from the tag position (Cartesian goals)
  * the PLACE poses stay static joint-space waypoints from waypoints.yaml

Sequence (same shape as pick_place_node, pick side replaced by vision):

   1. → start_pose            (waypoint)
   2. ↓ gripper OPEN
   3. → above tag             (tag + approach_offset, Cartesian)
   4. → onto tag              (tag + grasp_offset,   Cartesian)
   5. ↓ gripper CLOSE
   6. → above tag             (lift)
   7. → place_above           (waypoint)
   8. → place                 (waypoint)
   9. ↓ gripper OPEN
  10. → place_above
  11. → start_pose

Camera extrinsic convention: `camera_xyz`/`camera_rpy` locate the camera
BODY frame (ROS convention: x out of the lens, y left, z up) in `world`
(arm base). The fixed body→optical rotation is applied internally, so you
can measure the mount with a ruler. Verify with:

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .moveit_client import MoveItClient


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


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
        self.declare_parameter("gripper_open_pos", 0.019)
        self.declare_parameter("gripper_close_pos", -0.010)

        # ---- parameters: camera extrinsic (world → camera body frame) ----
        self.declare_parameter("camera_xyz", [0.25, 0.0, 0.30])
        self.declare_parameter("camera_rpy", [0.0, 0.6, 3.1416])

        # ---- parameters: grasp generation ----
        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("approach_offset", 0.10)   # m above tag
        self.declare_parameter("grasp_offset", 0.01)      # m above tag at grasp
        self.declare_parameter("grasp_pitch", 3.1416)     # pi = straight down
        self.declare_parameter("yaw_offset", 0.0)         # added to radial yaw
        self.declare_parameter("min_z", 0.015)            # floor clamp (m)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("max_sample_age", 1.5)     # s

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
            try:
                srdf_path = str(
                    Path(get_package_share_directory("om_chain_moveit_config"))
                    / "srdf" / "om_chain.srdf"
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
        )
        self.grip_open = float(self.get_parameter("gripper_open_pos").value)
        self.grip_close = float(self.get_parameter("gripper_close_pos").value)

        # ---- camera extrinsic ----
        cam_xyz = [float(v) for v in self.get_parameter("camera_xyz").value]
        cam_rpy = [float(v) for v in self.get_parameter("camera_rpy").value]
        self._t_wc = np.array(cam_xyz)
        self._R_wo = rpy_to_matrix(*cam_rpy) @ R_BODY_OPTICAL
        self.get_logger().info(
            f"camera body frame @ xyz={cam_xyz} rpy={cam_rpy} (in world)"
        )

        # ---- tag sample buffer ----
        self._samples: deque = deque(maxlen=60)   # (monotonic_time, p_optical)
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_topic").value),
            self._on_tag, 10, callback_group=self._cb_group,
        )

        # ---- worker plumbing ----
        self._run_lock = threading.Lock()
        self._sequence_busy = False

        self.create_service(
            Trigger, "/run_tag_pick", self._on_run, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/tag_world", self._on_tag_world, callback_group=self._cb_group,
        )
        self.get_logger().info(
            "ready — check extrinsic with /tag_world, then /run_tag_pick"
        )

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
        self._samples.append((time.monotonic(), p))

    def _tag_world(self) -> Optional[np.ndarray]:
        """Median of recent detections, transformed into the world frame."""
        now = time.monotonic()
        max_age = float(self.get_parameter("max_sample_age").value)
        pts = [p for (t, p) in self._samples if now - t <= max_age]
        if len(pts) < int(self.get_parameter("min_samples").value):
            return None
        p_opt = np.median(np.stack(pts), axis=0)
        return self._R_wo @ p_opt + self._t_wc

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

    # ---------------- sequence ----------------
    def _run_sequence(self, tag_w: np.ndarray) -> bool:
        approach = float(self.get_parameter("approach_offset").value)
        grasp = float(self.get_parameter("grasp_offset").value)
        pitch = float(self.get_parameter("grasp_pitch").value)
        yaw = math.atan2(tag_w[1], tag_w[0]) + \
            float(self.get_parameter("yaw_offset").value)
        min_z = float(self.get_parameter("min_z").value)

        above = tag_w + np.array([0.0, 0.0, approach])
        onto = tag_w + np.array([0.0, 0.0, grasp])
        onto[2] = max(onto[2], min_z)

        self.get_logger().info(
            f"tag@world=({tag_w[0]:+.3f}, {tag_w[1]:+.3f}, {tag_w[2]:+.3f})  "
            f"grasp yaw={math.degrees(yaw):.0f}° pitch={math.degrees(pitch):.0f}°"
        )

        steps = [
            ("wp", "start_pose"),
            ("grip", self.grip_open, "OPEN"),
            ("xyz", above, "above tag"),
            ("xyz", onto, "onto tag"),
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
                self.get_logger().info(
                    f"{label} {comment} ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"
                )
                if not self.client.move_to_xyz_rpy(
                    float(p[0]), float(p[1]), float(p[2]), 0.0, pitch, yaw
                ):
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

    def _worker_body(self, tag_w: np.ndarray) -> None:
        try:
            if not self.client.wait_for_servers(timeout_sec=30.0):
                self.get_logger().error("Required action servers not available")
                return
            self._run_sequence(tag_w)
        finally:
            with self._run_lock:
                self._sequence_busy = False

    # ---------------- services ----------------
    def _on_run(self, req: Trigger.Request, res: Trigger.Response):
        tag_w = self._tag_world()
        if tag_w is None:
            res.success = False
            res.message = "no recent tag detections — is the tag in view?"
            return res
        with self._run_lock:
            if self._sequence_busy:
                res.success = False
                res.message = "sequence already running"
                return res
            self._sequence_busy = True
        threading.Thread(
            target=self._worker_body, args=(tag_w,), daemon=True
        ).start()
        res.success = True
        res.message = (
            f"started — tag at world ({tag_w[0]:+.3f}, {tag_w[1]:+.3f}, "
            f"{tag_w[2]:+.3f})"
        )
        return res

    def _on_tag_world(self, req: Trigger.Request, res: Trigger.Response):
        tag_w = self._tag_world()
        if tag_w is None:
            res.success = False
            res.message = "no recent tag detections — is the tag in view?"
            return res
        res.success = True
        res.message = (
            f"tag in world frame: x={tag_w[0]:+.3f} y={tag_w[1]:+.3f} "
            f"z={tag_w[2]:+.3f}  (measure from arm base and compare)"
        )
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
