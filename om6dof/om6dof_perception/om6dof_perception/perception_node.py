#!/usr/bin/env python3
"""om6dof_perception: camera-frame object + end-effector perception.

Slow/fast split for real-time use:
  * SLOW  (seconds, background thread): a remote Ollama VLM grounds the
    target object and the gripper in the RGB image by text description.
  * FAST  (every frame, ~20-30 Hz): an OpenCV CSRT tracker follows each bbox,
    aligned RealSense depth deprojects the bbox center to a 3D point.

Everything is expressed in the camera optical frame (x right, y down,
z forward), so no camera->arm extrinsic is required. The published relative
vector (EE -> target) is the natural feedback signal for visual servoing.

Perception topics (under /om6dof_perception/):
  target_point  geometry_msgs/PointStamped   target in camera frame
  ee_point      geometry_msgs/PointStamped   gripper in camera frame
  relative      std_msgs/Float64MultiArray   [dx, dy, dz, dist] EE->target
  status        std_msgs/String              tracking state per entity
  debug_image/compressed  sensor_msgs/CompressedImage  overlay (optional)
  set_target    std_msgs/String (sub)        change target description

The same processed overlay is published by default to the application web
monitor input at /application_web_monitor/image/compressed. The web monitor's
camera card therefore appears only while this node is producing frames.

The node owns the RealSense — stop robot_api or any camera-owning GUI first.
"""
import base64
import json
import re
import sys
import threading
import time
import urllib.request

sys.path.append("/home/unitree/.local/lib/python3.10/site-packages")
import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage

CAMERA_FRAME = "camera_color_optical_frame"


class Entity:
    """One tracked thing (the target object or the gripper)."""

    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.tracker = None
        self.bbox = None            # (x, y, w, h)
        self.point = None           # [X, Y, Z] camera frame
        self.state = "acquiring"    # acquiring | tracking | lost
        self.last_acquire = 0.0


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("om6dof_perception")
        self.declare_parameter("target_description",
                               "glass jar with a black lid on the floor")
        self.declare_parameter("ee_description",
                               "robot arm gripper with two fingers")
        self.declare_parameter(
            "vlm_model", "qwen3-vl:8b-instruct-q4_K_M")
        self.declare_parameter("ollama_url", "http://192.168.123.99:11434")
        self.declare_parameter("reacquire_period_sec", 30.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("frame_rate_hz", 20.0)
        self.declare_parameter(
            "web_stream_topic", "/application_web_monitor/image/compressed")

        self.model = str(self.get_parameter("vlm_model").value)
        ollama_url = str(self.get_parameter("ollama_url").value).rstrip("/")
        self.ollama_chat_url = (
            ollama_url if ollama_url.endswith("/api/chat")
            else ollama_url + "/api/chat"
        )
        self.reacquire = float(self.get_parameter("reacquire_period_sec").value)
        self.debug_on = bool(self.get_parameter("publish_debug_image").value)
        self.rate = float(self.get_parameter("frame_rate_hz").value)
        self.web_stream_topic = str(
            self.get_parameter("web_stream_topic").value).strip()

        self.target = Entity(
            "target", str(self.get_parameter("target_description").value))
        self.ee = Entity(
            "ee", str(self.get_parameter("ee_description").value))

        self.pub_target = self.create_publisher(
            PointStamped, "~/target_point", 10)
        self.pub_ee = self.create_publisher(PointStamped, "~/ee_point", 10)
        self.pub_rel = self.create_publisher(
            Float64MultiArray, "~/relative", 10)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.pub_debug = self.create_publisher(
            CompressedImage, "~/debug_image/compressed", 2)
        self.pub_web = (
            self.create_publisher(
                CompressedImage, self.web_stream_topic, 2)
            if self.web_stream_topic else None
        )
        self.create_subscription(String, "~/set_target", self._on_set_target, 10)

        self.vlm_lock = threading.Lock()   # one ollama request at a time
        self.frame_lock = threading.Lock()
        self.rgb = None
        self.depth = None
        self.intr = None
        self.depth_scale = None

        threading.Thread(target=self._camera_loop, daemon=True).start()
        self.get_logger().info(
            f"perception up: target={self.target.description!r} "
            f"ee={self.ee.description!r} model={self.model} "
            f"ollama={self.ollama_chat_url} "
            f"web_stream={self.web_stream_topic or 'disabled'}")

    # ------------------------------------------------------------ input
    def _on_set_target(self, msg):
        self.target.description = msg.data.strip()
        self.target.state = "acquiring"
        self.target.tracker = None
        self.target.last_acquire = 0.0
        self.get_logger().info(f"new target: {self.target.description!r}")

    # ----------------------------------------------------------- camera
    def _camera_loop(self):
        pipe = None
        align = rs.align(rs.stream.color)
        period = 1.0 / self.rate
        while rclpy.ok():
            if pipe is None:
                try:
                    pipe = rs.pipeline()
                    cfg = rs.config()
                    cfg.enable_stream(rs.stream.color, 640, 480,
                                      rs.format.rgb8, 30)
                    cfg.enable_stream(rs.stream.depth, 640, 480,
                                      rs.format.z16, 30)
                    profile = pipe.start(cfg)
                    self.depth_scale = (profile.get_device()
                                        .first_depth_sensor()
                                        .get_depth_scale())
                    self.get_logger().info("realsense started")
                except Exception as e:
                    self.get_logger().warn(f"camera open failed: {e}")
                    pipe = None
                    time.sleep(5)
                    continue
            t0 = time.time()
            try:
                frames = align.process(pipe.wait_for_frames(timeout_ms=2000))
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth:
                    continue
                with self.frame_lock:
                    self.rgb = np.asanyarray(color.get_data()).copy()
                    self.depth = np.asanyarray(depth.get_data()).copy()
                    self.intr = (depth.profile.as_video_stream_profile()
                                 .intrinsics)
                self._step()
            except Exception as e:
                self.get_logger().warn(f"camera error: {e}")
                try:
                    pipe.stop()
                except Exception:
                    pass
                pipe = None
                continue
            time.sleep(max(0.0, period - (time.time() - t0)))

    # ------------------------------------------------------- processing
    def _step(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        with self.frame_lock:
            rgb = self.rgb
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        for ent in (self.target, self.ee):
            # kick a (re)acquire when needed, without blocking the loop
            stale = now - ent.last_acquire > self.reacquire
            if (ent.state in ("acquiring", "lost") or stale) and \
                    not self.vlm_lock.locked():
                ent.last_acquire = now
                threading.Thread(target=self._acquire, args=(ent, rgb.copy()),
                                 daemon=True).start()
            # fast path: advance the tracker
            if ent.tracker is not None:
                ok, box = ent.tracker.update(bgr)
                if ok:
                    ent.bbox = tuple(int(v) for v in box)
                    ent.state = "tracking"
                    ent.point = self._bbox_to_point(ent.bbox)
                else:
                    ent.state = "lost"
                    ent.tracker = None

        self._publish(stamp)
        if self.debug_on:
            self._publish_debug(bgr, stamp)

    def _bbox_to_point(self, bbox):
        x, y, w, h = bbox
        cx, cy = x + w // 2, y + h // 2
        cx, cy = max(0, min(639, cx)), max(0, min(479, cy))
        with self.frame_lock:
            depth, intr = self.depth, self.intr
        win = depth[max(0, cy - 5):cy + 6, max(0, cx - 5):cx + 6].astype(float)
        win = win[win > 0]
        if win.size == 0:
            return None
        z = float(np.median(win)) * self.depth_scale
        return list(rs.rs2_deproject_pixel_to_point(intr, [cx, cy], z))

    # ------------------------------------------------------- VLM (slow)
    def _acquire(self, ent, rgb):
        with self.vlm_lock:
            bbox = self._vlm_ground(ent.description, rgb)
        if bbox is None:
            if ent.tracker is None:
                ent.state = "lost"
            return
        x1, y1, x2, y2 = bbox
        w, h = max(4, x2 - x1), max(4, y2 - y1)
        with self.frame_lock:
            cur = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
        tracker = cv2.TrackerCSRT_create()
        tracker.init(cur, (x1, y1, w, h))
        ent.tracker = tracker
        ent.bbox = (x1, y1, w, h)
        ent.state = "tracking"
        self.get_logger().info(f"{ent.name} acquired at {bbox}")

    def _vlm_ground(self, description, rgb):
        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            return None
        prompt = (
            f"Outline the position of the {description} and output all the "
            'coordinates in JSON format {"bbox_2d": [x1, y1, x2, y2]}. If the '
            'object is not present in the image, output {"bbox_2d": null}.')
        body = {"model": self.model, "stream": False,
                "messages": [{"role": "user", "content": prompt,
                              "images": [base64.b64encode(jpg).decode()]}],
                "options": {"temperature": 0.0, "num_ctx": 4096}}
        # GPU first, CPU fallback (fragmented Jetson memory -> CUDA OOM)
        for num_gpu in (None, 0):
            if num_gpu is not None:
                body["options"]["num_gpu"] = num_gpu
            try:
                req = urllib.request.Request(
                    self.ollama_chat_url, data=json.dumps(body).encode(),
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=590) as r:
                    reply = json.loads(r.read())["message"]["content"]
                break
            except Exception as e:
                reply = None
                self.get_logger().warn(
                    f"vlm endpoint={self.ollama_chat_url} "
                    f"num_gpu={num_gpu}: {e}")
        if not reply or re.search(r'"bbox_2d"\s*:\s*null', reply):
            return None
        m = re.search(
            r"\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]", reply)
        return tuple(int(v) for v in m.groups()) if m else None

    # ----------------------------------------------------------- output
    def _publish(self, stamp):
        for ent, pub in ((self.target, self.pub_target),
                         (self.ee, self.pub_ee)):
            if ent.state == "tracking" and ent.point:
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = CAMERA_FRAME
                msg.point.x, msg.point.y, msg.point.z = ent.point
                pub.publish(msg)

        if (self.target.state == "tracking" and self.target.point and
                self.ee.state == "tracking" and self.ee.point):
            d = [t - e for t, e in zip(self.target.point, self.ee.point)]
            dist = float(np.linalg.norm(d))
            self.pub_rel.publish(Float64MultiArray(data=[*d, dist]))

        self.pub_status.publish(String(data=json.dumps({
            "target": {"state": self.target.state,
                       "desc": self.target.description,
                       "point": self.target.point},
            "ee": {"state": self.ee.state, "point": self.ee.point},
        })))

    def _publish_debug(self, bgr, stamp):
        for ent, color in ((self.target, (60, 60, 255)),
                           (self.ee, (60, 255, 60))):
            if ent.bbox and ent.state == "tracking":
                x, y, w, h = ent.bbox
                cv2.rectangle(bgr, (x, y), (x + w, y + h), color, 2)
                label = ent.name
                if ent.point:
                    label += f" z={ent.point[2]:.2f}m"
                cv2.putText(bgr, label, (x, max(12, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if (self.target.state == "tracking" and self.target.point and
                self.ee.state == "tracking" and self.ee.point):
            d = np.linalg.norm(np.subtract(self.target.point, self.ee.point))
            cv2.putText(bgr, f"EE->target {d:.3f} m", (10, 470),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(
            bgr,
            f"target: {self.target.state} | ee: {self.ee.state}",
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = CAMERA_FRAME
            msg.format = "jpeg"
            msg.data = jpg.tobytes()
            self.pub_debug.publish(msg)
            if self.pub_web is not None:
                self.pub_web.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
