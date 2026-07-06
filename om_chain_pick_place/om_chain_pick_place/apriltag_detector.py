"""AprilTag detector for the RealSense D435i.

Grabs color (+ aligned depth) frames straight from librealsense
(pyrealsense2) — no ROS camera driver needed — detects AprilTag 36h11
markers with OpenCV's aruco module, estimates the tag pose with
solvePnP(IPPE_SQUARE), and publishes:

  * `/apriltag/pose`        geometry_msgs/PoseStamped — tag pose in the
                            camera *optical* frame (x right, y down,
                            z forward, straight out of the lens)
  * `/apriltag/debug_image` sensor_msgs/Image (bgr8) — color frame with
                            the detected tag outlined + axes drawn

When depth is enabled the tag position is refined by deprojecting the
tag-center pixel through the aligned depth frame, which is usually more
accurate than the PnP scale estimate at small tag sizes.

The tag must be printed at exactly `tag_size` metres (black border edge
to black border edge) for the pose scale to be right.
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


TAG_DICTS = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


class AprilTagDetector(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_detector")

        # ---- parameters ----
        self.declare_parameter("serial", "")            # empty = first device
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        # depth stream can run at a lower resolution than color (rs.align
        # reprojects it onto the color intrinsics). On USB2 the D435i can't
        # do 1280x720 depth at all, so keep these at 640x480.
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("tag_size", 0.04)        # metres, outer black edge
        self.declare_parameter("tag_id", -1)            # -1 = nearest tag of any id
        # corner refinement: "subpix" (fast, ~25 ms/frame @720p) or
        # "apriltag" (more robust for tiny/blurry tags but ~8x slower,
        # ~195 ms/frame @720p on the Orin) or "none".
        self.declare_parameter("corner_refine", "subpix")
        self.declare_parameter("use_depth", True)
        # exposure > 0 = fixed manual exposure for the RGB sensor (RealSense
        # units); 0 = auto. Fixed exposure survives node restarts in the
        # camera firmware, so we always set it explicitly here.
        self.declare_parameter("exposure", 0)
        self.declare_parameter("publish_debug", True)
        # show a local OpenCV window with the annotated video (needs DISPLAY)
        self.declare_parameter("show_window", False)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        family = str(self.get_parameter("tag_family").value)
        if family not in TAG_DICTS:
            raise RuntimeError(f"tag_family must be one of {list(TAG_DICTS)}")
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)

        # ---- aruco detector (OpenCV 4.5 API) ----
        self._dict = cv2.aruco.Dictionary_get(TAG_DICTS[family])
        self._params = cv2.aruco.DetectorParameters_create()
        refine_modes = {
            "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
            "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
            "none": cv2.aruco.CORNER_REFINE_NONE,
        }
        refine = str(self.get_parameter("corner_refine").value)
        if refine not in refine_modes:
            raise RuntimeError(f"corner_refine must be one of {list(refine_modes)}")
        self._params.cornerRefinementMethod = refine_modes[refine]
        s = self.tag_size / 2.0
        # corner order returned by detectMarkers: TL, TR, BR, BL
        self._obj_pts = np.array(
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]],
            dtype=np.float64,
        )

        # ---- realsense ----
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        serial = str(self.get_parameter("serial").value)
        if serial:
            cfg.enable_device(serial)
        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        if self.use_depth:
            dw = int(self.get_parameter("depth_width").value)
            dh = int(self.get_parameter("depth_height").value)
            cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        profile = self._pipeline.start(cfg)
        self._align = rs.align(rs.stream.color) if self.use_depth else None

        exposure = int(self.get_parameter("exposure").value)
        for sensor in profile.get_device().query_sensors():
            if sensor.supports(rs.option.enable_auto_exposure) and \
                    sensor.get_info(rs.camera_info.name) == "RGB Camera":
                if exposure > 0:
                    sensor.set_option(rs.option.enable_auto_exposure, 0)
                    sensor.set_option(rs.option.exposure, exposure)
                else:
                    sensor.set_option(rs.option.enable_auto_exposure, 1)

        intr = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self._intr = intr
        self._K = np.array(
            [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]]
        )
        self._D = np.array(intr.coeffs[:5], dtype=np.float64)
        self.get_logger().info(
            f"RealSense up: {w}x{h}@{fps} fx={intr.fx:.1f} fy={intr.fy:.1f} "
            f"family={family} tag_size={self.tag_size} m"
        )

        # ---- publishers ----
        self._pose_pub = self.create_publisher(PoseStamped, "/apriltag/pose", 10)
        self._img_pub = (
            self.create_publisher(Image, "/apriltag/debug_image", 2)
            if self.publish_debug else None
        )
        self._bridge = CvBridge()

        # ---- capture thread ----
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------------- capture / detect loop ----------------
    def _loop(self) -> None:
        import time as _time
        no_tag_count = 0
        fps_t0 = _time.monotonic()
        fps_frames = 0
        fps = 0.0
        detect_ms = 0.0
        while not self._stop.is_set() and rclpy.ok():
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError as exc:
                self.get_logger().warn(f"frame timeout: {exc}")
                continue
            if self._align is not None:
                frames = self._align.process(frames)
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())
            depth = frames.get_depth_frame() if self._align is not None else None

            t_det = _time.monotonic()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params
            )
            best = self._pick_tag(corners, ids, depth)
            detect_ms = 0.9 * detect_ms + 0.1 * (_time.monotonic() - t_det) * 1e3

            if best is not None:
                tag_corners, tvec, rvec = best
                self._publish_pose(tvec, rvec)
                no_tag_count = 0
            else:
                no_tag_count += 1
                if no_tag_count % 90 == 1:   # ~every 3 s at 30 fps
                    self.get_logger().info("no AprilTag in view")

            # ---- fps bookkeeping (end-to-end loop rate) ----
            fps_frames += 1
            now = _time.monotonic()
            if now - fps_t0 >= 5.0:
                fps = fps_frames / (now - fps_t0)
                self.get_logger().info(
                    f"pipeline {fps:.1f} fps, detect {detect_ms:.0f} ms/frame"
                )
                fps_t0, fps_frames = now, 0

            if self._img_pub is not None or self.show_window:
                dbg = img.copy()
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
                    if best is not None:
                        cv2.drawFrameAxes(
                            dbg, self._K, self._D,
                            best[2], best[1], self.tag_size * 0.75,
                        )
                if best is not None:
                    tv = best[1]
                    status = (f"TAG  x={tv[0]:+.3f} y={tv[1]:+.3f} "
                              f"z={tv[2]:+.3f} m")
                    color_txt = (0, 220, 0)
                else:
                    status = "no tag"
                    color_txt = (0, 0, 255)
                cv2.putText(dbg, f"{fps:.1f} fps  det {detect_ms:.0f} ms",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (255, 255, 0), 2)
                cv2.putText(dbg, status, (10, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_txt, 2)
                if self._img_pub is not None:
                    msg = self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.camera_frame
                    self._img_pub.publish(msg)
                if self.show_window:
                    cv2.imshow("apriltag_detector", dbg)
                    cv2.waitKey(1)
        if self.show_window:
            cv2.destroyAllWindows()

    def _pick_tag(self, corners, ids, depth):
        """Return (corners, tvec, rvec) for the requested/nearest tag."""
        if ids is None or len(ids) == 0:
            return None
        candidates = []
        for c, i in zip(corners, ids.flatten()):
            if self.tag_id >= 0 and int(i) != self.tag_id:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self._obj_pts, c.reshape(4, 2).astype(np.float64),
                self._K, self._D, flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            tvec = tvec.reshape(3)
            if self.use_depth and depth is not None:
                refined = self._depth_refine(c.reshape(4, 2), depth)
                if refined is not None:
                    tvec = refined
            candidates.append((float(np.linalg.norm(tvec)), c, tvec, rvec))
        if not candidates:
            return None
        _, c, tvec, rvec = min(candidates, key=lambda t: t[0])
        return c, tvec, rvec

    def _depth_refine(self, corners: np.ndarray, depth) -> Optional[np.ndarray]:
        """Deproject the tag-center pixel using the aligned depth frame."""
        cx, cy = corners.mean(axis=0)
        u, v = int(round(cx)), int(round(cy))
        samples = []
        for du in range(-2, 3):
            for dv in range(-2, 3):
                uu, vv = u + du, v + dv
                if 0 <= uu < self._intr.width and 0 <= vv < self._intr.height:
                    d = depth.get_distance(uu, vv)
                    if d > 0.05:
                        samples.append(d)
        if len(samples) < 5:
            return None
        z = float(np.median(samples))
        pt = rs.rs2_deproject_pixel_to_point(self._intr, [cx, cy], z)
        return np.array(pt, dtype=np.float64)

    def _publish_pose(self, tvec: np.ndarray, rvec: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])
        R, _ = cv2.Rodrigues(rvec)
        qw = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        if qw > 1e-6:
            msg.pose.orientation.w = float(qw)
            msg.pose.orientation.x = float((R[2, 1] - R[1, 2]) / (4 * qw))
            msg.pose.orientation.y = float((R[0, 2] - R[2, 0]) / (4 * qw))
            msg.pose.orientation.z = float((R[1, 0] - R[0, 1]) / (4 * qw))
        else:
            msg.pose.orientation.w = 1.0
        self._pose_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        try:
            self._pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[AprilTagDetector] = None
    try:
        node = AprilTagDetector()
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
