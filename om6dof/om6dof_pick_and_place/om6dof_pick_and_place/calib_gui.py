"""Web GUI for LIVE camera-extrinsic calibration.

Serves a page at http://<robot-ip>:8081 with +/- steppers for
`camera_xyz` / `camera_rpy` / `object_center_from_tag` and a live readout
of the resulting coordinates:

    world -> end_effector_link (TF) -> camera -> AprilTag -> object centre

Every adjustment is applied in three places at once:
  1. this GUI's own math (instant feedback in the readout + RViz markers
     from coord_debug),
  2. `/tag_pick_place` parameters (the picker uses it on the next run),
  3. `/coord_debug` parameters.

The SIMPAN button writes the current values back into tag_pick.yaml (both
the src copy and the installed copy), so they survive a restart.

Typical workflow: put the cube at a spot you can measure from the arm
base, watch "object @ world" while stepping the offsets until it matches
the ruler, at several EE poses (jog the arm between checks) — a
wrist-mounted camera is only calibrated when the object stays put in
world while the ARM moves.

Run (detector must be running; MoveIt stack recommended for TF):

    ros2 run om6dof_pick_and_place calib_gui
"""

from __future__ import annotations

import json
import math
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .tag_pick_place_node import R_BODY_OPTICAL, rpy_to_matrix

SRC_YAML = Path(
    "/home/unitree/ros2_ws/src/unitree-go2w-autonomous-carrier/"
    "om6dof_pick_and_place/config/tag_pick.yaml"
)

PARAM_NODES = ["/tag_pick_place", "/coord_debug", "/direct_pick"]
VEC_PARAMS = ["camera_xyz", "camera_rpy", "object_center_from_tag",
              "approach_offset", "pick_offset"]


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CalibGui(Node):
    def __init__(self) -> None:
        super().__init__("calib_gui")
        self.declare_parameter("port", 8081)
        self.declare_parameter("tag_topic", "/apriltag/pose")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("camera_parent_frame", "end_effector_link")
        self.declare_parameter("camera_xyz", [-0.04775, 0.0175, -0.09979])
        self.declare_parameter("camera_rpy", [0.0, -1.5708, 0.0])
        self.declare_parameter("object_center_from_tag", [0.0, 0.0, -0.015])
        self.declare_parameter("approach_offset", [0.0, 0.0])
        self.declare_parameter("pick_offset", [0.0, 0.0])

        self._lock = threading.Lock()
        self.vals = {
            n: [float(v) for v in self.get_parameter(n).value]
            for n in VEC_PARAMS
        }
        self.ref_frame = str(self.get_parameter("reference_frame").value)
        self.parent_frame = str(self.get_parameter("camera_parent_frame").value)

        self._tag: Optional[PoseStamped] = None
        self._tag_stamp = 0.0
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_topic").value),
            self._on_tag, 10)

        # ---- camera view: /apriltag/debug_image -> MJPEG for the browser ----
        self.declare_parameter("image_topic", "/apriltag/debug_image")
        self.declare_parameter("stream_fps", 10.0)
        self.declare_parameter("jpeg_quality", 70)
        self._bridge = CvBridge()
        self._jpeg: Optional[bytes] = None
        self._jpeg_cond = threading.Condition()
        self._last_encode = 0.0
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, 2)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- self-FK fallback (no MoveIt / no TF: the direct_pick flow) ----
        # If TF world->end_effector_link isn't published, compute it ourselves
        # from /joint_states with the same PyKDL solver, so the world-object
        # readout works with ONLY the teleop arm server running.
        self.declare_parameter("arm_joint_names",
                               ["joint1", "joint2", "joint3",
                                "joint4", "joint5", "joint6"])
        self._arm_joints = [str(x) for x in
                            self.get_parameter("arm_joint_names").value]
        self._latest_q = None
        self._ik = None
        try:
            from go2w_remote_arm.ik_solver import IKSolver
            self._ik = IKSolver()
            self.create_subscription(
                JointState, "/joint_states", self._on_joints, 10)
            self.get_logger().info("self-FK enabled (fallback when no TF)")
        except Exception as exc:
            self.get_logger().warn(f"self-FK unavailable: {exc}")

        self._param_clients = {
            n: self.create_client(SetParameters, n + "/set_parameters")
            for n in PARAM_NODES
        }
        self._push_status = {n: "belum" for n in PARAM_NODES}

        self._run_cli = self.create_client(Trigger, "/run_tag_pick")
        self._search_cli = self.create_client(Trigger, "/run_search")
        self._approach_cli = self.create_client(Trigger, "/run_front_approach")
        self._status_cli = self.create_client(Trigger, "/tag_pick_status")
        # direct (no-MoveIt) pick flow
        self._direct_run_cli = self.create_client(Trigger, "/run_direct_pick")
        self._direct_approach_cli = self.create_client(Trigger, "/direct_approach")
        self._direct_status_cli = self.create_client(Trigger, "/direct_pick_status")
        self._direct_origin_cli = self.create_client(Trigger, "/direct_go_origin")
        self._direct_ready_cli = self.create_client(Trigger, "/direct_go_ready")
        self._grip_open_cli = self.create_client(Trigger, "/direct_grip_open")
        self._grip_close_cli = self.create_client(Trigger, "/direct_grip_close")
        self._track_cli = self.create_client(Trigger, "/direct_track")
        self._stop_cli = self.create_client(Trigger, "/direct_stop")
        # live reachability status from direct_pick
        self._reach = "—"
        from std_msgs.msg import String as _Str
        self.create_subscription(
            _Str, "/direct_reach",
            lambda m: setattr(self, "_reach", m.data),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        port = int(self.get_parameter("port").value)
        handler = self._make_handler()
        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), handler)
        self._httpd.daemon_threads = True   # open MJPEG streams must not
        # block shutdown
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f"calib GUI: http://<robot-ip>:{port}/")

        # pull the picker's live values once at startup so the GUI starts in
        # sync with what /tag_pick_place actually loaded from its yaml
        self._sync_cli = self.create_client(
            GetParameters, "/tag_pick_place/get_parameters")
        self._synced = False
        self.create_timer(1.0, self._sync_from_picker)

    def _sync_from_picker(self) -> None:
        if self._synced or not self._sync_cli.service_is_ready():
            return
        req = GetParameters.Request(names=list(VEC_PARAMS))
        fut = self._sync_cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                if res is None:
                    return
                with self._lock:
                    for name, pv in zip(VEC_PARAMS, res.values):
                        if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY \
                                and len(pv.double_array_value) == 3:
                            self.vals[name] = [
                                float(v) for v in pv.double_array_value]
                self._synced = True
                self.get_logger().info(
                    "nilai awal disinkronkan dari /tag_pick_place")
            except Exception:
                pass
        fut.add_done_callback(_done)

    # ---------------- ROS side ----------------
    def _on_image(self, msg: Image) -> None:
        # throttle encoding to stream_fps — no point compressing 30 frames/s
        # when the browser only shows ~10
        now = time.monotonic()
        min_dt = 1.0 / max(1.0, float(self.get_parameter("stream_fps").value))
        if now - self._last_encode < min_dt:
            return
        self._last_encode = now
        try:
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            ok, buf = cv2.imencode(
                ".jpg", img,
                [cv2.IMWRITE_JPEG_QUALITY,
                 int(self.get_parameter("jpeg_quality").value)])
            if ok:
                with self._jpeg_cond:
                    self._jpeg = buf.tobytes()
                    self._jpeg_cond.notify_all()
        except Exception:
            pass

    def _on_tag(self, msg: PoseStamped) -> None:
        with self._lock:
            self._tag = msg
            self._tag_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg: JointState) -> None:
        d = dict(zip(msg.name, msg.position))
        try:
            self._latest_q = np.array([float(d[n]) for n in self._arm_joints])
        except KeyError:
            pass

    def _ee_tf(self):
        if not self.parent_frame:
            return np.eye(3), np.zeros(3), True
        # 1) prefer TF (MoveIt / robot_state_publisher)
        try:
            tf = self._tf_buffer.lookup_transform(
                self.ref_frame, self.parent_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return quat_to_matrix(q.x, q.y, q.z, q.w), \
                np.array([t.x, t.y, t.z]), True
        except TransformException:
            pass
        # 2) fallback: self-FK from /joint_states (direct_pick flow, no TF)
        if self._ik is not None and self._latest_q is not None:
            try:
                p, R = self._ik.fk_pose(self._latest_q)
                return np.asarray(R), np.asarray(p), True
            except Exception:
                pass
        return np.eye(3), np.zeros(3), False

    def snapshot(self) -> dict:
        with self._lock:
            vals = {k: list(v) for k, v in self.vals.items()}
            tag = self._tag
            tag_stamp = self._tag_stamp
        now = self.get_clock().now().nanoseconds * 1e-9
        out = {
            "vals": vals,
            "push": dict(self._push_status),
            "tag_age": None, "tag_cam": None,
            "tf_ok": False, "ee_xyz": None,
            "cam_world": None, "tag_world": None, "obj_world": None,
            "obj_to_ee": None, "obj_to_ee_dist": None,
            "reach": self._reach,
        }
        R_we, t_we, tf_ok = self._ee_tf()
        out["tf_ok"] = tf_ok
        if tf_ok:
            out["ee_xyz"] = [round(float(v), 4) for v in t_we]

        t_ec = np.array(vals["camera_xyz"])
        R_eb = rpy_to_matrix(*vals["camera_rpy"])
        R_eo = R_eb @ R_BODY_OPTICAL
        if tf_ok:
            cam_w = R_we @ t_ec + t_we
            R_wo = R_we @ R_eo
            out["cam_world"] = [round(float(v), 4) for v in cam_w]

        if tag is not None:
            out["tag_age"] = round(now - tag_stamp, 2)
            p = tag.pose.position
            q = tag.pose.orientation
            p_opt = np.array([p.x, p.y, p.z])
            out["tag_cam"] = [round(float(v), 4) for v in p_opt]
            if tf_ok:
                tag_w = R_wo @ p_opt + cam_w
                R_ot = quat_to_matrix(q.x, q.y, q.z, q.w)
                obj_w = tag_w + (R_wo @ R_ot) @ np.array(
                    vals["object_center_from_tag"])
                out["tag_world"] = [round(float(v), 4) for v in tag_w]
                out["obj_world"] = [round(float(v), 4) for v in obj_w]
                # object → gripper (EoE): vector from the current tool point
                # to the object. During approach this shrinks; at grasp it
                # should equal the fingertip offset. Great for calibration.
                d = obj_w - t_we
                out["obj_to_ee"] = [round(float(v), 4) for v in d]
                out["obj_to_ee_dist"] = round(float(np.linalg.norm(d)), 4)
        return out

    def adjust(self, name: str, index: int, delta: float) -> dict:
        with self._lock:
            self.vals[name][index] = round(self.vals[name][index] + delta, 6)
            new = list(self.vals[name])
        self._push_param(name, new)
        return {"ok": True, "value": new}

    def set_vec(self, name: str, values) -> dict:
        vec = [float(v) for v in values]
        with self._lock:
            self.vals[name] = vec
        self._push_param(name, vec)
        return {"ok": True, "value": vec}

    def _push_param(self, name: str, vec) -> None:
        pv = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=vec)
        for node_name, cli in self._param_clients.items():
            if not cli.service_is_ready():
                self._push_status[node_name] = "offline"
                continue
            req = SetParameters.Request()
            req.parameters = [Parameter(name=name, value=pv)]
            fut = cli.call_async(req)

            def _done(f, nn=node_name):
                try:
                    res = f.result()
                    ok = res is not None and all(
                        r.successful for r in res.results)
                    self._push_status[nn] = "ok" if ok else "DITOLAK"
                except Exception:
                    self._push_status[nn] = "error"
            fut.add_done_callback(_done)

    def _call_trigger(self, cli, timeout: float = 8.0) -> dict:
        """Call a Trigger service from an HTTP thread (executor spins in main)."""
        import time as _time
        if not cli.service_is_ready():
            return {"ok": False, "message": f"service {cli.srv_name} offline"}
        fut = cli.call_async(Trigger.Request())
        deadline = _time.monotonic() + timeout
        while not fut.done() and _time.monotonic() < deadline:
            _time.sleep(0.05)
        res = fut.result() if fut.done() else None
        if res is None:
            return {"ok": False, "message": "timeout menunggu respons service"}
        return {"ok": bool(res.success), "message": res.message}

    def run_pick(self) -> dict:
        return self._call_trigger(self._run_cli)

    def run_search(self) -> dict:
        return self._call_trigger(self._search_cli)

    def run_approach(self) -> dict:
        return self._call_trigger(self._approach_cli)

    def run_direct(self) -> dict:
        return self._call_trigger(self._direct_run_cli)

    def run_direct_approach(self) -> dict:
        return self._call_trigger(self._direct_approach_cli)

    def go_origin(self) -> dict:
        return self._call_trigger(self._direct_origin_cli)

    def go_ready(self) -> dict:
        return self._call_trigger(self._direct_ready_cli)

    def grip_open(self) -> dict:
        return self._call_trigger(self._grip_open_cli)

    def grip_close(self) -> dict:
        return self._call_trigger(self._grip_close_cli)

    def track(self) -> dict:
        return self._call_trigger(self._track_cli)

    def stop(self) -> dict:
        return self._call_trigger(self._stop_cli)

    def pick_status(self) -> dict:
        # prefer whichever picker is actually up
        if self._direct_status_cli.service_is_ready():
            return self._call_trigger(self._direct_status_cli, timeout=3.0)
        return self._call_trigger(self._status_cli, timeout=3.0)

    def save_yaml(self) -> dict:
        targets = [SRC_YAML]
        try:
            targets.append(
                Path(get_package_share_directory("om6dof_pick_and_place")) /
                "config" / "tag_pick.yaml")
        except Exception:
            pass
        with self._lock:
            vals = {k: list(v) for k, v in self.vals.items()}
        written = []
        for path in targets:
            if not path.exists() or path in [p for p in written]:
                continue
            try:
                if path.resolve() in [p.resolve() for p in written]:
                    continue  # symlink-install: share file == src file
            except OSError:
                pass
            text = path.read_text()
            for name, vec in vals.items():
                pretty = "[" + ", ".join(f"{v:.5f}" for v in vec) + "]"
                text = re.sub(
                    rf"({re.escape(name)}:\s*)\[[^\]]*\]",
                    lambda m, s=pretty: m.group(1) + s,
                    text)
            path.write_text(text)
            written.append(path)
        return {"ok": True, "files": [str(p) for p in written]}

    # ---------------- HTTP side ----------------
    def _make_handler(self):
        gui = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # silence request spam
                pass

            def _json(self, obj, code=200):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/state":
                    self._json(gui.snapshot())
                    return
                if self.path == "/frame.jpg":
                    with gui._jpeg_cond:
                        jpeg = gui._jpeg
                    if jpeg is None:
                        self.send_response(503)
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return
                if self.path == "/stream":
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            with gui._jpeg_cond:
                                gui._jpeg_cond.wait(timeout=2.0)
                                jpeg = gui._jpeg
                            if jpeg is None:
                                continue
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                try:
                    data = json.loads(self.rfile.read(n) or b"{}")
                except json.JSONDecodeError:
                    self._json({"ok": False, "err": "bad json"}, 400)
                    return
                try:
                    if self.path == "/adjust":
                        self._json(gui.adjust(
                            str(data["name"]), int(data["index"]),
                            float(data["delta"])))
                    elif self.path == "/set":
                        self._json(gui.set_vec(
                            str(data["name"]), data["values"]))
                    elif self.path == "/save":
                        self._json(gui.save_yaml())
                    elif self.path == "/run":
                        self._json(gui.run_pick())
                    elif self.path == "/search":
                        self._json(gui.run_search())
                    elif self.path == "/approach":
                        self._json(gui.run_approach())
                    elif self.path == "/direct_run":
                        self._json(gui.run_direct())
                    elif self.path == "/direct_approach":
                        self._json(gui.run_direct_approach())
                    elif self.path == "/go_origin":
                        self._json(gui.go_origin())
                    elif self.path == "/go_ready":
                        self._json(gui.go_ready())
                    elif self.path == "/grip_open":
                        self._json(gui.grip_open())
                    elif self.path == "/grip_close":
                        self._json(gui.grip_close())
                    elif self.path == "/track":
                        self._json(gui.track())
                    elif self.path == "/stop":
                        self._json(gui.stop())
                    elif self.path == "/pickstatus":
                        self._json(gui.pick_status())
                    else:
                        self._json({"ok": False, "err": "unknown"}, 404)
                except (KeyError, ValueError, IndexError) as exc:
                    self._json({"ok": False, "err": str(exc)}, 400)

        return Handler

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Kalibrasi Kamera OM-Chain</title><style>
body{font-family:system-ui,sans-serif;background:#14171c;color:#dfe5ec;margin:0;padding:14px}
h1{font-size:1.15rem;margin:0 0 10px}
h2{font-size:0.95rem;color:#9fb2c8;margin:16px 0 6px}
table{border-collapse:collapse;width:100%;max-width:760px}
td,th{padding:4px 8px;text-align:left;font-variant-numeric:tabular-nums}
tr{border-bottom:1px solid #262c35}
button{background:#2a3644;color:#dfe5ec;border:1px solid #3c4a5c;border-radius:6px;
 padding:6px 9px;margin:1px;cursor:pointer;font-size:0.9rem}
button:active{background:#3d5a80}
.val{display:inline-block;min-width:88px;text-align:right;font-weight:600;color:#8ecbff}
.save{background:#1f5130;border-color:#2f7a49;font-size:1rem;padding:10px 22px;margin-top:14px}
.ok{color:#7ee08b}.bad{color:#ff8f8f}.dim{color:#7a8899}
#status{margin-top:8px;min-height:1.2em}
.mono{font-family:ui-monospace,monospace}
</style></head><body>
<h1>🔧 Kalibrasi kamera live — world → EE → kamera → tag → objek</h1>
<div style="display:flex;gap:18px;align-items:flex-start;flex-wrap:wrap">
 <div style="flex:1 1 420px;min-width:340px;order:1">
  <div id="controls"></div>
  <h2>Pembacaan live</h2>
  <table id="readout"></table>
  <div style="margin-top:14px">
  <button class="save" onclick="save()">💾 SIMPAN ke tag_pick.yaml</button>
  </div>
  <div style="margin-top:10px">
  <div class="dim" style="font-size:0.85rem;margin-bottom:4px">MoveIt (lama):</div>
  <button class="save" style="background:#1d3d5c;border-color:#2f5f8a"
   onclick="runSearch()">🔍 SEARCH</button>
  <button class="save" style="background:#3a2a5c;border-color:#5a468a"
   onclick="runApproach()">➡ APPROACH</button>
  <button class="save" style="background:#5a3410;border-color:#8a5420"
   onclick="runPick()">▶ RUN PICK</button>
  </div>
  <div style="margin-top:10px">
  <div class="dim" style="font-size:0.85rem;margin-bottom:4px">Direct / tanpa MoveIt (teleop arm-server):</div>
  <button class="save" style="background:#444;border-color:#666"
   onclick="goOrigin()">⌂ ORIGIN (0,0,0,0,0,0)</button>
  <button class="save" style="background:#444;border-color:#666"
   onclick="goReady()">⌂ READY POSE</button>
  <button class="save" style="background:#3a3a3a;border-color:#666"
   onclick="gripOpen()">✋ GRIP OPEN</button>
  <button class="save" style="background:#3a3a3a;border-color:#666"
   onclick="gripClose()">✊ GRIP CLOSE</button>
  <button class="save" style="background:#1d5c5c;border-color:#2f8a8a"
   onclick="runTrack()">🎯 TRACK (ngejar objek)</button>
  <button class="save" style="background:#5c1d1d;border-color:#8a2f2f"
   onclick="runStop()">■ STOP</button>
  <button class="save" style="background:#2a5c3a;border-color:#468a5a"
   onclick="runDirectApproach()">➡ APPROACH (direct)</button>
  <button class="save" style="background:#5c1d3d;border-color:#8a2f5f"
   onclick="runDirect()">▶ RUN PICK (direct)</button>
  </div>
  <div id="pickstatus" class="mono dim" style="margin-top:8px;min-height:1.2em"></div>
  <div id="status" class="dim"></div>
 </div>
 <div style="flex:1 1 480px;min-width:360px;order:2;position:sticky;top:10px">
  <img id="cam" src="/stream" alt="menunggu kamera…"
   style="width:100%;border:1px solid #3c4a5c;border-radius:8px;background:#000">
 </div>
</div>
<script>
const SPECS = {
 camera_xyz: {label:"camera_xyz (m) — posisi kamera relatif end_effector_link",
   axes:["x","y","z"], steps:[0.001,0.005,0.02], unit:"m", dp:5},
 camera_rpy: {label:"camera_rpy (rad) — orientasi kamera",
   axes:["roll","pitch","yaw"], steps:[0.0087,0.0349,0.1745], unit:"rad", dp:4},
 object_center_from_tag: {label:"object_center_from_tag (m) — pusat kubus dari tag",
   axes:["x","y","z"], steps:[0.001,0.005,0.02], unit:"m", dp:4},
 approach_offset: {label:"approach_offset (m) — posisi HOVER ujung-arm (EE) thd objek: depan & tinggi (pick_offset TIDAK dipakai di sini)",
   axes:["standoff(depan)","naik(+atas)"], steps:[0.005,0.01,0.02], unit:"m", dp:4},
 pick_offset: {label:"pick_offset (m) — koreksi ujung jari HANYA saat grasp: maju & naik",
   axes:["maju(+dalam)","naik(+atas)"], steps:[0.002,0.005,0.01], unit:"m", dp:4},
};
let vals = {};
function fmt(v,dp){return (v>=0?"+":"")+v.toFixed(dp)}
function stepLabel(s,u){return u==="rad" ? (s*57.2958).toFixed(1)+"°" : (s*1000)+"mm"}
function buildControls(){
 let h="";
 for(const [name,s] of Object.entries(SPECS)){
  h+=`<h2>${s.label}</h2><table>`;
  s.axes.forEach((ax,i)=>{
   h+=`<tr><td style="width:52px">${ax}</td><td><span class="val mono" id="v_${name}_${i}">…</span></td><td>`;
   for(const st of [...s.steps].reverse()) h+=`<button onclick="adj('${name}',${i},${-st})">−${stepLabel(st,s.unit)}</button>`;
   for(const st of s.steps) h+=`<button onclick="adj('${name}',${i},${st})">+${stepLabel(st,s.unit)}</button>`;
   h+=`</td></tr>`;
  });
  h+=`</table>`;
 }
 document.getElementById("controls").innerHTML=h;
}
async function adj(name,i,delta){
 const r=await fetch("/adjust",{method:"POST",body:JSON.stringify({name,index:i,delta})});
 const j=await r.json();
 if(j.ok){vals[name]=j.value; paint();}
}
function row(k,v,cls){return `<tr><td>${k}</td><td class="mono ${cls||''}">${v}</td></tr>`}
function vec(a,axes){axes=axes||["x","y","z"];
 return a?a.map((v,i)=>`${axes[i]}=${fmt(v,4)}`).join("  "):"—"}
async function poll(){
 try{
  const s=await (await fetch("/state")).json();
  vals=s.vals; paint();
  let h="";
  h+=row("EE pose (TF/FK)", s.tf_ok?"OK":"TIDAK ADA (teleop / MoveIt jalan?)", s.tf_ok?"ok":"bad");
  h+=row("EE @ world", vec(s.ee_xyz)+"   <span class='dim'>x=depan y=kiri z=atas</span>");
  h+=row("Kamera @ world", vec(s.cam_world));
  h+=row("Umur deteksi tag", s.tag_age===null?"tidak ada tag":s.tag_age+" s", (s.tag_age!==null&&s.tag_age<1)?"ok":"bad");
  h+=row("Tag @ frame kamera", vec(s.tag_cam,["kanan","bawah","jarak"]));
  h+=row("Tag @ world", vec(s.tag_world));
  h+=row("★ OBJEK @ world", vec(s.obj_world), "ok");
  h+=row("OBJEK → EE (ujung arm)", s.obj_to_ee ? (vec(s.obj_to_ee)+"   <span class='dim'>jarak "+(s.obj_to_ee_dist*1000).toFixed(0)+" mm</span>") : "—", s.obj_to_ee_dist!==null && s.obj_to_ee_dist<0.02 ? "ok":"");
  var reach=s.reach||"—";
  var reachClass = reach.indexOf("reachable r")===0 ? "ok" : (reach.indexOf("OUT")===0||reach.indexOf("unreachable")>=0 ? "bad":"");
  var reachTxt = reach==="—"?"—": reach.indexOf("reachable r")===0?("✓ DALAM jangkauan ("+reach.replace("reachable ","")+")") : reach.indexOf("OUT")===0?("✗ DI LUAR jangkauan ("+reach.replace("OUT of reach ","")+")") : reach.indexOf("no_object")>=0?"tidak ada objek" : reach;
  h+=row("★ JANGKAUAN", reachTxt, reachClass);
  h+=row("Push parameter", Object.entries(s.push).map(([k,v])=>`${k}: ${v}`).join("  |  "));
  document.getElementById("readout").innerHTML=h;
 }catch(e){document.getElementById("status").textContent="koneksi terputus…";}
}
function paint(){
 for(const [name,s] of Object.entries(SPECS)){
  (vals[name]||[]).forEach((v,i)=>{
   const el=document.getElementById(`v_${name}_${i}`);
   if(el) el.textContent=fmt(v,s.dp);
  });
 }
}
async function save(){
 const j=await (await fetch("/save",{method:"POST",body:"{}"})).json();
 document.getElementById("status").textContent=
   j.ok?("Tersimpan: "+j.files.join(" , ")):("GAGAL: "+j.err);
}
let watching=false;
async function trigger(path,label,confirmMsg){
 if(confirmMsg && !confirm(confirmMsg)) return;
 const el=document.getElementById("pickstatus");
 el.textContent="mengirim "+label+" …";
 const j=await (await fetch(path,{method:"POST",body:"{}"})).json();
 el.textContent=(j.ok?"▶ ":"✗ ")+j.message;
 el.className="mono "+(j.ok?"ok":"bad");
 if(j.ok && !watching){watching=true; watchStatus();}
}
function runPick(){trigger("/run","/run_tag_pick",
 "Arm akan BERGERAK menjalankan sekuens pick. Area sekitar arm aman?")}
function runSearch(){trigger("/search","/run_search",
 "Arm akan BERGERAK menyapu pose-pose search sampai tag terlihat (tanpa pick). Lanjut?")}
function runApproach(){trigger("/approach","/run_front_approach",
 "Arm akan BERGERAK ke titik ~10 cm di depan objek lalu berhenti (tanpa grasp). Lanjut?")}
function runDirect(){trigger("/direct_run","/run_direct_pick",
 "DIRECT (tanpa MoveIt): arm akan BERGERAK menjalankan sekuens pick penuh. Area aman?")}
function runTrack(){trigger("/track","/direct_track",
 "Arm akan MENGEJAR objek terus-menerus (hover di depannya, ikut geseran termasuk y). Area aman?")}
function runStop(){trigger("/stop","/direct_stop",null)}
function runDirectApproach(){trigger("/direct_approach","/direct_approach",
 "DIRECT (tanpa MoveIt): arm akan BERGERAK ke depan objek lalu berhenti (tanpa grasp). Lanjut?")}
function goOrigin(){trigger("/go_origin","/direct_go_origin",
 "Arm akan BERGERAK ke ORIGIN (semua joint 0 — arm tegak lurus ke ATAS). Area aman?")}
function goReady(){trigger("/go_ready","/direct_go_ready",
 "Arm akan BERGERAK ke READY pose. Lanjut?")}
function gripOpen(){trigger("/grip_open","/direct_grip_open",null)}
function gripClose(){trigger("/grip_close","/direct_grip_close",null)}
async function watchStatus(){
 const el=document.getElementById("pickstatus");
 for(;;){
  await new Promise(r=>setTimeout(r,1000));
  try{
   const s=await (await fetch("/pickstatus",{method:"POST",body:"{}"})).json();
   el.textContent="status: "+s.message;
   el.className="mono "+(s.ok?"ok":"dim");
  }catch(e){}
 }
}
buildControls(); poll(); setInterval(poll,200);
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node: Optional[CalibGui] = None
    try:
        node = CalibGui()
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
