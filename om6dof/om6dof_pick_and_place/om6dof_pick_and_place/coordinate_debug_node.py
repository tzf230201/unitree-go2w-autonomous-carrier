"""Coordinate debugger for AprilTag pick-and-place calibration.

This node does not command the arm. It compares:

  * detector coordinates: /apriltag/pose in the camera optical frame
  * estimated world coordinates: tag/object transformed through camera extrinsic
  * robot coordinates: current end_effector_link pose from TF

It publishes RViz markers on /coord_debug_markers and exposes
/coord_debug_snapshot for a one-shot text report.
"""

from __future__ import annotations

import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


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


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy < 1e-6:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


R_BODY_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


class CoordinateDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("coord_debug")

        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("enable_robot_debug", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("ee_link", "end_effector_link")
        self.declare_parameter("camera_parent_frame", "")
        self.declare_parameter(
            "arm_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "gripper_joint_names",
            ["gripper_left_joint", "gripper_right_joint"],
        )
        self.declare_parameter("camera_xyz", [0.25, 0.0, 0.30])
        self.declare_parameter("camera_rpy", [0.0, 0.6, 3.1416])
        self.declare_parameter("cube_size", 0.03)
        self.declare_parameter("object_center_from_tag", [0.0, 0.0, -0.015])
        self.declare_parameter("log_period", 1.0)
        self.declare_parameter("live_console", True)
        self.declare_parameter("marker_period", 0.2)
        self.declare_parameter("max_tag_age", 2.0)

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.camera_parent_frame = str(self.get_parameter("camera_parent_frame").value)
        self.enable_robot_debug = bool(self.get_parameter("enable_robot_debug").value)
        self.live_console = bool(self.get_parameter("live_console").value)
        cam_xyz = [float(v) for v in self.get_parameter("camera_xyz").value]
        cam_rpy = [float(v) for v in self.get_parameter("camera_rpy").value]
        self._t_wc = np.array(cam_xyz)
        self._R_parent_body = rpy_to_matrix(*cam_rpy)
        self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
        self._last_tag: Optional[PoseStamped] = None
        self._last_tag_time = 0.0
        self._latest_joints: Optional[Dict[str, float]] = None
        self._last_joint_time = 0.0
        self._last_log = 0.0
        self._cursor_hidden = False

        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if self.enable_robot_debug or self.camera_parent_frame:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self._marker_pub = self.create_publisher(MarkerArray, "/coord_debug_markers", 2)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("tag_topic").value),
            self._on_tag,
            10,
        )
        if self.enable_robot_debug:
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._on_joint_state,
                10,
            )
        self.create_service(Trigger, "/coord_debug_snapshot", self._on_snapshot)
        self.create_service(Trigger, "/coord_debug_waypoint", self._on_waypoint_snapshot)
        self.create_timer(float(self.get_parameter("marker_period").value), self._tick)
        # live extrinsic updates (e.g. from calib_gui via `ros2 param set`)
        self.add_on_set_parameters_callback(self._on_extrinsic_params)

        if self.enable_robot_debug:
            self.get_logger().info(
                "coordinate debugger ready: detector=/apriltag/pose, "
                f"robot TF={self.reference_frame}->{self.ee_link}, joints=/joint_states"
            )
        elif self.camera_parent_frame:
            self.get_logger().info(
                "coordinate debugger ready: AprilTag + camera TF mode "
                f"({self.reference_frame}->{self.camera_parent_frame}->camera)"
            )
        else:
            self.get_logger().info(
                "coordinate debugger ready: AprilTag-only mode "
                "(robot TF and /joint_states disabled)"
            )

    def _on_extrinsic_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            try:
                if p.name == "camera_xyz":
                    self._t_wc = np.array([float(v) for v in p.value])
                elif p.name == "camera_rpy":
                    self._R_parent_body = rpy_to_matrix(*[float(v) for v in p.value])
                    self._R_parent_optical = self._R_parent_body @ R_BODY_OPTICAL
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=f"{p.name}: {exc}")
        return SetParametersResult(successful=True)

    def _on_tag(self, msg: PoseStamped) -> None:
        self._last_tag = msg
        self._last_tag_time = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joints = dict(zip(msg.name, msg.position))
        self._last_joint_time = time.monotonic()

    def _camera_reference_optical(self):
        if not self.camera_parent_frame:
            return self._t_wc, self._R_parent_optical, None
        if self._tf_buffer is None:
            return None, None, "camera_parent_frame needs TF, but TF is disabled"
        try:
            tf = self._tf_buffer.lookup_transform(
                self.reference_frame, self.camera_parent_frame, Time()
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

    def _recent_tag_camera(self):
        if self._last_tag is None:
            return None
        age = time.monotonic() - self._last_tag_time
        if age > float(self.get_parameter("max_tag_age").value):
            return None

        pose = self._last_tag.pose
        tag_cam = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = pose.orientation
        R_ot = quat_to_matrix(q.x, q.y, q.z, q.w)
        return tag_cam, R_ot, age

    def _tag_object_world(self):
        tag = self._recent_tag_camera()
        if tag is None:
            return None
        tag_cam, R_ot, age = tag
        t_ref_optical, R_ref_optical, tf_error = self._camera_reference_optical()
        if tf_error is not None:
            return tag_cam, None, None, age, tf_error
        R_wt = R_ref_optical @ R_ot
        tag_world = R_ref_optical @ tag_cam + t_ref_optical
        tag_to_obj = np.array(
            [float(v) for v in self.get_parameter("object_center_from_tag").value]
        )
        obj_world = tag_world + R_wt @ tag_to_obj
        return tag_cam, tag_world, obj_world, age, None

    def _ee_world(self):
        if not self.enable_robot_debug or self._tf_buffer is None:
            return None, "robot debug disabled"
        try:
            tf = self._tf_buffer.lookup_transform(
                self.reference_frame, self.ee_link, Time()
            )
        except TransformException as exc:
            return None, str(exc)
        t = tf.transform.translation
        q = tf.transform.rotation
        rpy = matrix_to_rpy(quat_to_matrix(q.x, q.y, q.z, q.w))
        return (np.array([t.x, t.y, t.z]), np.array(rpy)), None

    def _joint_lines(self) -> List[str]:
        if not self.enable_robot_debug:
            return []
        if not self._latest_joints:
            return ["robot joints: no /joint_states received yet"]

        age = time.monotonic() - self._last_joint_time
        arm_names = [str(v) for v in self.get_parameter("arm_joint_names").value]
        grip_names = [str(v) for v in self.get_parameter("gripper_joint_names").value]
        missing = [name for name in arm_names if name not in self._latest_joints]
        if missing:
            return [f"robot joints: missing from /joint_states: {missing}"]

        arm = [float(self._latest_joints[name]) for name in arm_names]
        deg = [math.degrees(v) for v in arm]
        rad_txt = ", ".join(f"{name}={val:+.4f}" for name, val in zip(arm_names, arm))
        deg_txt = ", ".join(f"{name}={val:+.1f}" for name, val in zip(arm_names, deg))
        waypoint = ", ".join(f"{val:+.4f}" for val in arm)
        lines = [
            f"robot joints age: {age:.2f}s",
            f"arm joints rad:   {rad_txt}",
            f"arm joints deg:   {deg_txt}",
            f"waypoint joint:   [{waypoint}]",
        ]

        present_grip = [
            f"{name}={float(self._latest_joints[name]):+.4f}"
            for name in grip_names
            if name in self._latest_joints
        ]
        if present_grip:
            lines.append(f"gripper joints:  {', '.join(present_grip)}")
        return lines

    def _waypoint_snippet(self) -> Tuple[bool, str]:
        if not self.enable_robot_debug:
            return False, "robot debug disabled; enable coord_robot_debug to snapshot joints"
        if not self._latest_joints:
            return False, "no /joint_states received yet"
        arm_names = [str(v) for v in self.get_parameter("arm_joint_names").value]
        try:
            vals = [round(float(self._latest_joints[name]), 4) for name in arm_names]
        except KeyError as exc:
            return False, f"joint not in /joint_states: {exc}"
        return True, (
            "  # paste under `waypoints:`\n"
            "  manual_pose:\n"
            f"    joint: {vals}\n"
        )

    def _format_report(self) -> str:
        tag = self._tag_object_world()
        ee, tf_error = self._ee_world()
        lines = []
        if self.camera_parent_frame:
            lines.append(
                f"camera TF: {self.reference_frame}->{self.camera_parent_frame}->camera optical"
            )
        else:
            lines.append(f"camera TF: static {self.reference_frame}->camera optical")
        lines.extend(self._joint_lines())
        if tag is None:
            lines.append("tag: no recent /apriltag/pose")
        else:
            tag_cam, tag_world, obj_world, age, tf_error = tag
            lines.append(
                "detector camera optical tag_xyz: "
                f"x={tag_cam[0]:+.3f} y={tag_cam[1]:+.3f} z={tag_cam[2]:+.3f} m "
                f"(age {age:.2f}s)"
            )
            if tf_error is not None:
                lines.append(f"world tag unavailable: {tf_error}")
            else:
                lines.append(
                    f"world tag_xyz:   x={tag_world[0]:+.3f} y={tag_world[1]:+.3f} "
                    f"z={tag_world[2]:+.3f} m"
                )
                lines.append(
                    f"world object_xyz:x={obj_world[0]:+.3f} y={obj_world[1]:+.3f} "
                    f"z={obj_world[2]:+.3f} m"
                )
        if ee is None and self.enable_robot_debug:
            lines.append(f"robot EE TF unavailable: {tf_error}")
        elif ee is not None:
            ee_xyz, ee_rpy = ee
            lines.append(
                f"robot EE world:  x={ee_xyz[0]:+.3f} y={ee_xyz[1]:+.3f} "
                f"z={ee_xyz[2]:+.3f} m"
            )
            lines.append(
                f"robot EE rpy:    r={ee_rpy[0]:+.3f} p={ee_rpy[1]:+.3f} "
                f"y={ee_rpy[2]:+.3f} rad"
            )
        if tag is not None and tag[2] is not None and ee is not None:
            obj_world = tag[2]
            delta = obj_world - ee[0]
            lines.append(
                "object - EE:     "
                f"dx={delta[0]:+.3f} dy={delta[1]:+.3f} dz={delta[2]:+.3f} m "
                f"|dist|={np.linalg.norm(delta):.3f} m"
            )
        return "\n".join(lines)

    def _tick(self) -> None:
        now = time.monotonic()
        self._publish_markers()
        log_period = float(self.get_parameter("log_period").value)
        if log_period <= 0.0:
            return
        if now - self._last_log >= log_period:
            self._last_log = now
            if self.live_console:
                self._draw_live_console()
            else:
                self.get_logger().info("\n" + self._format_report())

    def _draw_live_console(self) -> None:
        if not self._cursor_hidden:
            sys.stdout.write("\033[?25l")
            self._cursor_hidden = True
        if self.enable_robot_debug:
            mode = "Robot + AprilTag"
        elif self.camera_parent_frame:
            mode = "AprilTag + camera TF"
        else:
            mode = "AprilTag-only"
        lines = [
            "OM6DOF Coordinate Debug",
            f"mode: {mode} | update: {time.strftime('%H:%M:%S')}",
            "snapshot: ros2 service call /coord_debug_snapshot std_srvs/srv/Trigger",
            "",
            self._format_report(),
            "",
            "Ctrl-C to stop. Set coord_log_period:=0.0 to disable this live view.",
        ]
        sys.stdout.write("\033[2J\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def _make_marker(self, mid: int, mtype: int, xyz, scale, rgba, text: str = ""):
        m = Marker()
        m.header.frame_id = self.reference_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "coord_debug"
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
        m = self._make_marker(mid, Marker.LINE_STRIP, [0, 0, 0], (0.006, 0.0, 0.0), rgba)
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = float(a[0]), float(a[1]), float(a[2])
        p2.x, p2.y, p2.z = float(b[0]), float(b[1]), float(b[2])
        m.points = [p1, p2]
        return m

    def _publish_markers(self) -> None:
        arr = MarkerArray()
        tag = self._tag_object_world()
        ee, _ = self._ee_world()
        ee_xyz = ee[0] if ee is not None else None
        cube_size = float(self.get_parameter("cube_size").value)
        if tag is not None and tag[2] is not None:
            _tag_cam, tag_world, obj_world, _age, _tf_error = tag
            arr.markers.append(self._make_marker(
                0, Marker.SPHERE, tag_world, (0.025, 0.025, 0.025),
                (1.0, 0.8, 0.1, 0.9),
            ))
            arr.markers.append(self._make_marker(
                1, Marker.CUBE, obj_world, (cube_size, cube_size, cube_size),
                (0.1, 0.9, 0.1, 0.75),
            ))
            arr.markers.append(self._make_marker(
                2, Marker.TEXT_VIEW_FACING, obj_world + np.array([0, 0, 0.06]),
                (0.0, 0.0, 0.03), (1.0, 1.0, 1.0, 1.0),
                f"object {obj_world[0]:+.2f},{obj_world[1]:+.2f},{obj_world[2]:+.2f}",
            ))
        if ee_xyz is not None:
            arr.markers.append(self._make_marker(
                3, Marker.SPHERE, ee_xyz, (0.025, 0.025, 0.025),
                (0.1, 0.8, 1.0, 0.9),
            ))
            arr.markers.append(self._make_marker(
                4, Marker.TEXT_VIEW_FACING, ee_xyz + np.array([0, 0, 0.05]),
                (0.0, 0.0, 0.03), (0.1, 0.8, 1.0, 1.0), "EE",
            ))
        if tag is not None and ee_xyz is not None:
            arr.markers.append(self._line_marker(5, ee_xyz, tag[2], (1.0, 1.0, 1.0, 0.8)))
        self._marker_pub.publish(arr)

    def _on_snapshot(self, req: Trigger.Request, res: Trigger.Response):
        res.success = True
        res.message = self._format_report()
        return res

    def _on_waypoint_snapshot(self, req: Trigger.Request, res: Trigger.Response):
        res.success, res.message = self._waypoint_snippet()
        if res.success:
            self.get_logger().info(f"manual waypoint snapshot:\n{res.message}")
        return res

    def destroy_node(self):
        if self._cursor_hidden:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
            self._cursor_hidden = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[CoordinateDebugNode] = None
    try:
        node = CoordinateDebugNode()
        rclpy.spin(node)
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
