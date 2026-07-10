"""QR/AprilTag follower for the OpenManipulator chain.

The detector publishes the QR/tag pose in the camera optical frame. This node
transforms that pose into the arm reference frame, then commands the end
effector to stay `follow_distance` metres in front of the tag while preserving
the tag orientation.

Default orientation convention:
  * target position = tag origin + tag +Z normal * follow_distance
  * EE +Z axis points back toward the tag
  * EE +X axis follows the tag +X axis

That convention is encoded by `ee_from_tag_rpy = [pi, 0, 0]`. Tune that offset
if the robot's end_effector_link needs a different tool-frame alignment.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
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


def matrix_to_quat(R: np.ndarray) -> Quaternion:
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    q = Quaternion()
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q.w = 0.25 * s
        q.x = (R[2, 1] - R[1, 2]) / s
        q.y = (R[0, 2] - R[2, 0]) / s
        q.z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q.w = (R[2, 1] - R[1, 2]) / s
        q.x = 0.25 * s
        q.y = (R[0, 1] + R[1, 0]) / s
        q.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q.w = (R[0, 2] - R[2, 0]) / s
        q.x = (R[0, 1] + R[1, 0]) / s
        q.y = 0.25 * s
        q.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q.w = (R[1, 0] - R[0, 1]) / s
        q.x = (R[0, 2] + R[2, 0]) / s
        q.y = (R[1, 2] + R[2, 1]) / s
        q.z = 0.25 * s
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if n < 1e-9:
        q.w = 1.0
        q.x = q.y = q.z = 0.0
    else:
        q.x, q.y, q.z, q.w = q.x / n, q.y / n, q.z / n, q.w / n
    return q


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    R = a.T @ b
    c = (float(np.trace(R)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, c)))


# camera BODY (x fwd, y left, z up) -> OPTICAL (x right, y down, z fwd)
R_BODY_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


class QRFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_follower")

        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("ee_link", "end_effector_link")
        self.declare_parameter("arm_group", "arm")
        self.declare_parameter("gripper_group", "gripper")
        self.declare_parameter("camera_parent_frame", "end_effector_link")
        self.declare_parameter("camera_xyz", [-0.08247, 0.0, -0.00960])
        self.declare_parameter("camera_rpy", [0.0, -1.13450, 0.0])
        self.declare_parameter("follow_distance", 0.10)
        self.declare_parameter("ee_from_tag_rpy", [3.1416, 0.0, 0.0])
        self.declare_parameter("auto_start", False)
        self.declare_parameter("command_period", 0.8)
        self.declare_parameter("min_samples", 4)
        self.declare_parameter("max_sample_age", 1.0)
        self.declare_parameter("position_deadband", 0.012)
        self.declare_parameter("orientation_deadband", 0.12)
        self.declare_parameter("min_z", 0.015)
        self.declare_parameter("max_z", 0.45)
        self.declare_parameter("vel_scale", 0.15)
        self.declare_parameter("acc_scale", 0.15)
        self.declare_parameter("planning_time", 3.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("orientation_tolerance", 0.35)
        self.declare_parameter("marker_period", 0.2)

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.camera_parent_frame = str(self.get_parameter("camera_parent_frame").value)
        cam_xyz = [float(v) for v in self.get_parameter("camera_xyz").value]
        cam_rpy = [float(v) for v in self.get_parameter("camera_rpy").value]
        self._t_parent_body = np.array(cam_xyz)
        self._R_parent_body = rpy_to_matrix(*cam_rpy)
        self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
        self._R_tag_ee = rpy_to_matrix(
            *[float(v) for v in self.get_parameter("ee_from_tag_rpy").value]
        )

        self.client = MoveItClient(
            self,
            arm_group=str(self.get_parameter("arm_group").value),
            gripper_group=str(self.get_parameter("gripper_group").value),
            ee_link=self.ee_link,
            reference_frame=self.reference_frame,
            max_velocity_scaling=float(self.get_parameter("vel_scale").value),
            max_acceleration_scaling=float(self.get_parameter("acc_scale").value),
            planning_time=float(self.get_parameter("planning_time").value),
            num_planning_attempts=int(self.get_parameter("planning_attempts").value),
            position_tolerance=float(self.get_parameter("position_tolerance").value),
            orientation_tolerance=float(self.get_parameter("orientation_tolerance").value),
        )

        self._samples: deque = deque(maxlen=40)
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("tag_topic").value),
            self._on_tag,
            10,
            callback_group=self._cb_group,
        )

        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if self.camera_parent_frame:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, "/qr_follow/markers", 2)
        self.create_timer(
            float(self.get_parameter("marker_period").value),
            self._publish_markers,
            callback_group=self._cb_group,
        )

        self._run_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._active = False
        self._auto_start_done = False
        self._last_target_pos: Optional[np.ndarray] = None
        self._last_target_rot: Optional[np.ndarray] = None
        self._last_move_ok: Optional[bool] = None
        self._last_move_time = 0.0
        self._last_wait_log = 0.0

        self.create_service(
            Trigger, "/qr_follow/start", self._on_start, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/qr_follow/stop", self._on_stop, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/qr_follow/status", self._on_status,
            callback_group=self._cb_group,
        )
        self.add_on_set_parameters_callback(self._on_params)

        if bool(self.get_parameter("auto_start").value):
            self.create_timer(1.0, self._auto_start_once, callback_group=self._cb_group)

        self.get_logger().info(
            "QR follower ready. Start with: "
            "ros2 service call /qr_follow/start std_srvs/srv/Trigger"
        )

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            try:
                if p.name == "camera_xyz":
                    self._t_parent_body = np.array([float(v) for v in p.value])
                elif p.name == "camera_rpy":
                    self._R_parent_body = rpy_to_matrix(*[float(v) for v in p.value])
                    self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
                elif p.name == "ee_from_tag_rpy":
                    self._R_tag_ee = rpy_to_matrix(*[float(v) for v in p.value])
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=f"{p.name}: {exc}")
        return SetParametersResult(successful=True)

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
            return self._t_parent_body, self._R_parent_optical, None
        if self._tf_buffer is None:
            return None, None, "camera_parent_frame needs TF, but TF is disabled"
        try:
            tf = self._tf_buffer.lookup_transform(
                self.reference_frame,
                self.camera_parent_frame,
                Time(),
            )
        except TransformException as exc:
            return None, None, str(exc)
        t = tf.transform.translation
        q = tf.transform.rotation
        t_ref_parent = np.array([t.x, t.y, t.z])
        R_ref_parent = quat_to_matrix(q.x, q.y, q.z, q.w)
        t_ref_optical = t_ref_parent + R_ref_parent @ self._t_parent_body
        R_ref_optical = R_ref_parent @ self._R_parent_optical
        return t_ref_optical, R_ref_optical, None

    def _recent_tag_camera(self):
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
        age = now - samples[-1][0]
        return p_opt, R_ot, age, len(samples)

    def _target_world(self):
        tag = self._recent_tag_camera()
        if tag is None:
            return None, "waiting for recent tag samples"
        p_opt, R_ot, age, sample_count = tag
        t_ref_optical, R_ref_optical, tf_error = self._camera_reference_optical()
        if tf_error is not None:
            return None, tf_error

        R_wt = R_ref_optical @ R_ot
        tag_w = R_ref_optical @ p_opt + t_ref_optical
        distance = float(self.get_parameter("follow_distance").value)
        target_w = tag_w + R_wt @ np.array([0.0, 0.0, distance])
        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)
        target_w[2] = max(min(target_w[2], max_z), min_z)
        R_we = R_wt @ self._R_tag_ee
        return (tag_w, target_w, R_we, age, sample_count), None

    def _pose_from_target(self, target_w: np.ndarray, R_we: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(target_w[0])
        pose.position.y = float(target_w[1])
        pose.position.z = float(target_w[2])
        pose.orientation = matrix_to_quat(R_we)
        return pose

    def _should_skip(self, target_w: np.ndarray, R_we: np.ndarray) -> bool:
        if self._last_target_pos is None or self._last_target_rot is None:
            return False
        pos_delta = float(np.linalg.norm(target_w - self._last_target_pos))
        rot_delta = rotation_angle(self._last_target_rot, R_we)
        return (
            pos_delta < float(self.get_parameter("position_deadband").value)
            and rot_delta < float(self.get_parameter("orientation_deadband").value)
        )

    def _follow_loop(self) -> None:
        try:
            if not self.client.wait_for_move_server(timeout_sec=30.0):
                return
            period = max(0.05, float(self.get_parameter("command_period").value))
            self.get_logger().info(
                f"QR follow active: keeping {float(self.get_parameter('follow_distance').value)*100:.0f} cm "
                "in front of the tag"
            )
            while rclpy.ok() and not self._stop_event.is_set():
                result, error = self._target_world()
                if error is not None:
                    now = time.monotonic()
                    if now - self._last_wait_log > 1.0:
                        self._last_wait_log = now
                        self.get_logger().info(f"follow waiting: {error}")
                    time.sleep(0.1)
                    continue

                tag_w, target_w, R_we, age, sample_count = result
                if self._should_skip(target_w, R_we):
                    time.sleep(period)
                    continue

                pose = self._pose_from_target(target_w, R_we)
                self.get_logger().info(
                    "follow target: "
                    f"tag=({tag_w[0]:+.3f}, {tag_w[1]:+.3f}, {tag_w[2]:+.3f}) "
                    f"ee=({target_w[0]:+.3f}, {target_w[1]:+.3f}, {target_w[2]:+.3f}) "
                    f"samples={sample_count} age={age:.2f}s"
                )
                ok = self.client.move_to_pose(
                    pose,
                    position_tolerance=float(
                        self.get_parameter("position_tolerance").value),
                    orientation_tolerance=float(
                        self.get_parameter("orientation_tolerance").value),
                )
                self._last_move_ok = ok
                self._last_move_time = time.monotonic()
                if ok:
                    self._last_target_pos = target_w
                    self._last_target_rot = R_we
                time.sleep(period)
        finally:
            with self._run_lock:
                self._active = False
                self._stop_event.set()
            self.get_logger().info("QR follow stopped")

    def _start_following(self) -> bool:
        with self._run_lock:
            if self._active:
                return False
            self._stop_event.clear()
            self._active = True
            self._worker = threading.Thread(target=self._follow_loop, daemon=True)
            self._worker.start()
            return True

    def _stop_following(self) -> bool:
        with self._run_lock:
            if not self._active:
                return False
            self._stop_event.set()
            return True

    def _auto_start_once(self) -> None:
        if self._auto_start_done:
            return
        self._auto_start_done = True
        if self._start_following():
            self.get_logger().info("auto_start triggered QR follower")

    def _format_status(self) -> str:
        result, error = self._target_world()
        lines = [
            f"active: {self._active}",
            f"recent samples: {len(self._samples)}",
            f"follow_distance: {float(self.get_parameter('follow_distance').value):.3f} m",
        ]
        if self._last_move_ok is None:
            lines.append("last_move: none")
        else:
            age = time.monotonic() - self._last_move_time
            lines.append(f"last_move: {'ok' if self._last_move_ok else 'failed'} age={age:.1f}s")
        if error is not None:
            lines.append(f"target: unavailable: {error}")
        else:
            tag_w, target_w, _R, sample_age, sample_count = result
            delta = target_w - tag_w
            lines.append(
                f"tag:    x={tag_w[0]:+.3f} y={tag_w[1]:+.3f} z={tag_w[2]:+.3f} m"
            )
            lines.append(
                f"target: x={target_w[0]:+.3f} y={target_w[1]:+.3f} z={target_w[2]:+.3f} m"
            )
            lines.append(
                f"tag->target distance={np.linalg.norm(delta):.3f} m "
                f"samples={sample_count} sample_age={sample_age:.2f}s"
            )
        return "\n".join(lines)

    def _on_start(self, req: Trigger.Request, res: Trigger.Response):
        if self._start_following():
            res.success = True
            res.message = "QR follower started"
        else:
            res.success = False
            res.message = "QR follower already active"
        return res

    def _on_stop(self, req: Trigger.Request, res: Trigger.Response):
        if self._stop_following():
            res.success = True
            res.message = "QR follower stop requested"
        else:
            res.success = False
            res.message = "QR follower is not active"
        return res

    def _on_status(self, req: Trigger.Request, res: Trigger.Response):
        res.success = True
        res.message = self._format_status()
        return res

    def _make_marker(self, mid: int, mtype: int, xyz, scale, rgba, text: str = ""):
        m = Marker()
        m.header.frame_id = self.reference_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "qr_follow"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x = float(xyz[0])
        m.pose.position.y = float(xyz[1])
        m.pose.position.z = float(xyz[2])
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        m.lifetime = Duration(seconds=1.0).to_msg()
        return m

    def _line_marker(self, mid: int, a, b, rgba):
        m = self._make_marker(mid, Marker.LINE_STRIP, [0, 0, 0], (0.004, 0.0, 0.0), rgba)
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = float(a[0]), float(a[1]), float(a[2])
        p2.x, p2.y, p2.z = float(b[0]), float(b[1]), float(b[2])
        m.points = [p1, p2]
        return m

    def _publish_markers(self) -> None:
        result, error = self._target_world()
        if error is not None:
            return
        tag_w, target_w, _R, _age, _sample_count = result
        arr = MarkerArray()
        arr.markers.append(self._make_marker(
            0, Marker.SPHERE, tag_w, (0.025, 0.025, 0.025),
            (1.0, 0.8, 0.1, 0.9), "QR",
        ))
        arr.markers.append(self._make_marker(
            1, Marker.SPHERE, target_w, (0.022, 0.022, 0.022),
            (0.1, 0.7, 1.0, 0.9), "follow target",
        ))
        arr.markers.append(self._line_marker(
            2, tag_w, target_w, (0.1, 0.7, 1.0, 0.8),
        ))
        self._marker_pub.publish(arr)

    def destroy_node(self):
        self._stop_event.set()
        worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[QRFollowerNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = QRFollowerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
