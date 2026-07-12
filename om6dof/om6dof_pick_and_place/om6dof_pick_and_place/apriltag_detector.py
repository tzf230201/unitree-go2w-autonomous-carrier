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
        self.declare_parameter("terminal_log_period", 0.0)
        # show a local OpenCV window with the annotated video (needs DISPLAY)
        self.declare_parameter("show_window", False)
        self.declare_parameter("window_name", "AprilTag Detector")
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        family = str(self.get_parameter("tag_family").value)
        if family not in TAG_DICTS:
            raise RuntimeError(f"tag_family must be one of {list(TAG_DICTS)}")
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.terminal_log_period = float(self.get_parameter("terminal_log_period").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
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
        # --- robustness tuning (defaults are strict; these help blurred /
        # oblique / unevenly-lit tags, common with a moving wrist camera) ---
        # more threshold scales → tolerate uneven lighting + varying tag size
        self._params.adaptiveThreshWinSizeMin = 3
        self._params.adaptiveThreshWinSizeMax = 53
        self._params.adaptiveThreshWinSizeStep = 10
        # accept slightly rounded/blurred quads (motion blur)
        self._params.polygonalApproxAccuracyRate = 0.05
        # sample marker bits more densely + ignore wider cell margins →
        # decodes tags seen at an angle much more reliably
        self._params.perspectiveRemovePixelPerCell = 8
        self._params.perspectiveRemoveIgnoredMarginPerCell = 0.25
        # allow more bit errors before rejecting (36h11 has strong ECC)
        self._params.errorCorrectionRate = 0.8
        self._params.minMarkerPerimeterRate = 0.02
        # also accept colour-INVERTED tags (white border on dark background).
        # The user's cube tag is printed as a negative — without this flag it
        # never decodes even when razor sharp (root cause found 2026-07-09).
        self._params.detectInvertedMarker = True
        # retry a missed frame with CLAHE contrast enhancement (only costs
        # time when nothing was found, so tracking stays fast)
        self.declare_parameter("clahe_retry", True)
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)) \
            if bool(self.get_parameter("clahe_retry").value) else None
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
        dw = int(self.get_parameter("depth_width").value)
        dh = int(self.get_parameter("depth_height").value)

        # Try the requested profile, then progressively USB2-safe fallbacks.
        # The D405 negotiates USB2.1 on some ports/cables, where 1280x720 tops
        # out at 15 fps (colour) / 5 fps (depth) — a 30 fps request fails with
        # "Couldn't resolve requests". Rather than crash, step down.
        attempts = [(w, h, fps, self.use_depth, dw, dh)]
        attempts += [
            (w, h, 15, self.use_depth, 640, 480),   # USB2 720p colour @ 15
            (848, 480, 30, self.use_depth, 640, 480),
            (640, 480, 30, self.use_depth, 640, 480),
            (w, h, 15, False, 0, 0),                 # colour only, no depth
            (640, 480, 30, False, 0, 0),
        ]
        profile = None
        last_err = None
        for (cw, ch, cfps, dep, cdw, cdh) in attempts:
            cfg = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
            if dep:
                cfg.enable_stream(rs.stream.depth, cdw, cdh, rs.format.z16, cfps)
            try:
                self._pipeline = rs.pipeline()
                profile = self._pipeline.start(cfg)
                self.use_depth = dep
                w, h, fps = cw, ch, cfps
                if (cw, ch, cfps, dep) != (
                        int(self.get_parameter("width").value),
                        int(self.get_parameter("height").value),
                        int(self.get_parameter("fps").value),
                        bool(self.get_parameter("use_depth").value)):
                    self.get_logger().warn(
                        f"requested profile unavailable — using {cw}x{ch}@{cfps}"
                        f"{' +depth' if dep else ' (no depth)'} "
                        "(likely USB2.1 — check cable/port for full speed)")
                break
            except RuntimeError as exc:
                last_err = exc
                continue
        if profile is None:
            raise RuntimeError(f"no RealSense profile worked: {last_err}")
        self._align = rs.align(rs.stream.color) if self.use_depth else None

        exposure = int(self.get_parameter("exposure").value)
        # D435i exposes color on a dedicated "RGB Camera" sensor; the D405's
        # color comes from the "Stereo Module" itself. Prefer the RGB sensor
        # when present, otherwise fall back to whatever supports exposure.
        sensors = [
            s for s in profile.get_device().query_sensors()
            if s.supports(rs.option.enable_auto_exposure)
        ]
        rgb = [s for s in sensors
               if s.get_info(rs.camera_info.name) == "RGB Camera"]
        for sensor in (rgb if rgb else sensors):
            if exposure > 0:
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, exposure)
            else:
                sensor.set_option(rs.option.enable_auto_exposure, 1)
            self.get_logger().info(
                f"exposure {'manual ' + str(exposure) if exposure > 0 else 'auto'}"
                f" on sensor '{sensor.get_info(rs.camera_info.name)}'")

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
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

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
        status_log_t0 = 0.0
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
            if (ids is None or len(ids) == 0) and self._clahe is not None:
                # second chance on low-contrast / washed-out frames
                corners, ids, _ = cv2.aruco.detectMarkers(
                    self._clahe.apply(gray), self._dict,
                    parameters=self._params,
                )
            visible_ids = [] if ids is None else [int(i) for i in ids.flatten()]
            best = self._pick_tag(corners, ids, depth)
            detect_ms = 0.9 * detect_ms + 0.1 * (_time.monotonic() - t_det) * 1e3

            if best is not None:
                best_id, tag_corners, tvec, rvec = best
                self._publish_pose(tvec, rvec)
                no_tag_count = 0
            else:
                no_tag_count += 1
                status_now = _time.monotonic()
                if (
                    self.terminal_log_period > 0.0
                    and status_now - status_log_t0 >= self.terminal_log_period
                ):
                    status_log_t0 = status_now
                    if visible_ids and self.tag_id >= 0 and self.tag_id not in visible_ids:
                        self.get_logger().info(
                            f"visible AprilTag IDs {visible_ids}, waiting for ID {self.tag_id}"
                        )
                    elif visible_ids:
                        self.get_logger().info(
                            f"visible AprilTag IDs {visible_ids}, but no usable pose"
                        )
                    else:
                        self.get_logger().info("no AprilTag in view")

            # ---- fps bookkeeping (end-to-end loop rate) ----
            fps_frames += 1
            now = _time.monotonic()
            if now - fps_t0 >= 5.0:
                fps = fps_frames / (now - fps_t0)
                if self.terminal_log_period > 0.0:
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
                            best[3], best[2], self.tag_size * 0.75,
                        )
                if best is not None:
                    best_id, _, tv, _ = best
                    status = (f"TAG id={best_id}  x={tv[0]:+.3f} y={tv[1]:+.3f} "
                              f"z={tv[2]:+.3f} m")
                    color_txt = (0, 220, 0)
                else:
                    if visible_ids and self.tag_id >= 0 and self.tag_id not in visible_ids:
                        status = f"seen ids {visible_ids}, waiting id {self.tag_id}"
                    elif visible_ids:
                        status = f"seen ids {visible_ids}, no usable pose"
                    else:
                        status = "no tag"
                    color_txt = (0, 0, 255)
                self._draw_status_panel(dbg, fps, detect_ms, status, color_txt,
                                        visible_ids)
                if self._img_pub is not None:
                    msg = self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.camera_frame
                    self._img_pub.publish(msg)
                if self.show_window:
                    cv2.imshow(self.window_name, dbg)
                    cv2.waitKey(1)
        if self.show_window:
            cv2.destroyAllWindows()

    def _draw_status_panel(self, img, fps, detect_ms, status, color_txt, visible_ids):
        h, w = img.shape[:2]
        panel_h = 92
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.62, img, 0.38, 0, img)
        cv2.rectangle(img, (0, 0), (w, panel_h), color_txt, 3)

        target = "any" if self.tag_id < 0 else str(self.tag_id)
        seen = "-" if not visible_ids else ",".join(str(i) for i in visible_ids)
        line1 = (
            f"AprilTag {target} | seen: {seen} | {fps:.1f} fps | "
            f"detect {detect_ms:.0f} ms"
        )
        line2 = f"{status} | size {self.tag_size * 1000:.0f} mm | {self.camera_frame}"

        cv2.putText(img, line1, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.82,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, line2, (18, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.74,
                    color_txt, 2, cv2.LINE_AA)

    def _pick_tag(self, corners, ids, depth):
        """Return (tag_id, corners, tvec, rvec) for the requested/nearest tag."""
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
            candidates.append((float(np.linalg.norm(tvec)), int(i), c, tvec, rvec))
        if not candidates:
            return None
        _, tag_id, c, tvec, rvec = min(candidates, key=lambda t: t[0])
        return tag_id, c, tvec, rvec

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
