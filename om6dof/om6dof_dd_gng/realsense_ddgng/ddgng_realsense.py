#!/usr/bin/env python3
"""DD-GNG live on the RealSense D435i, overlaid on the RGB view.

Pipeline per frame:
    RealSense depth (aligned to colour)
      -> sub-sampled grid of valid pixels deprojected to 3D points (metres)
      -> fed to the original DD-GNG core (libddgng.so, from gng.cpp)
      -> node positions + edges read back
      -> nodes projected back to pixels and drawn on the colour image.

The GNG algorithm is the unmodified simulation code (Fritzke 1995 parameters);
only the input (real depth instead of a simulated ray scan) and the output
(OpenCV overlay instead of GL) are new.

Run inside the venv that has pyrealsense2 + cv2:
    DISPLAY=:1002 LD_LIBRARY_PATH=build_om6dof ~/ggcnn_env/bin/python ddgng_realsense.py
"""
import ctypes
import argparse
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs

# --- tunables (pre-processing only; never touch the GNG parameters) ----------
W, H, FPS = 640, 480, 30
PIXEL_STEP = 6          # sub-sample stride -> ~ (W/step)*(H/step) candidate points
Z_MIN, Z_MAX = 0.2, 4.0  # metres; clamp depth to a sane working range
GNG_ITERS = 4           # learning passes per frame (matches the sim's 4x loop)
MAX_NODES = 500         # must match GNGN in the core
MAX_EDGES = 8000


def load_lib():
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "build_om6dof", "libddgng.so"),
                 os.path.join(here, "build", "libddgng.so"),
                 os.path.join(here, "libddgng.so"),
                 "libddgng.so"):
        if os.path.exists(cand):
            lib = ctypes.CDLL(cand)
            break
    else:
        raise FileNotFoundError("libddgng.so not found - build it with cmake first")

    d, dp, ip, i, dbl = (ctypes.c_void_p, ctypes.POINTER(ctypes.c_double),
                         ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_double)
    lib.ddgng_create.restype = d
    lib.ddgng_feed.argtypes = [d, dp, i, dbl, dbl, dbl, i]
    lib.ddgng_feed.restype = i
    lib.ddgng_get_nodes.argtypes = [d, dp, i]
    lib.ddgng_get_nodes.restype = i
    lib.ddgng_get_edges.argtypes = [d, ip, i]
    lib.ddgng_get_edges.restype = i
    return lib


def parse_args():
    parser = argparse.ArgumentParser(description="OM6DOF DD-GNG RealSense")
    parser.add_argument(
        "--headless", action="store_true",
        help="disable the local OpenCV window (for systemd/web monitor)",
    )
    parser.add_argument(
        "--ros-topic", default="",
        help="publish the annotated JPEG as sensor_msgs/CompressedImage",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    lib = load_lib()
    net = lib.ddgng_create()

    ros_node = None
    ros_publisher = None
    CompressedImage = None
    if args.ros_topic:
        import rclpy
        from sensor_msgs.msg import CompressedImage as RosCompressedImage

        CompressedImage = RosCompressedImage
        rclpy.init(args=None)
        ros_node = rclpy.create_node("om6dof_dd_gng")
        ros_publisher = ros_node.create_publisher(
            CompressedImage, args.ros_topic, 2
        )
        ros_node.get_logger().info(
            f"DD-GNG web stream: {args.ros_topic}"
        )

    # --- RealSense pipeline: depth + colour, depth aligned to colour ----------
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    # Colour intrinsics (used for both deprojection and re-projection so the
    # overlay stays self-consistent even though we use a plain pinhole model).
    ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    fx, fy, ppx, ppy = ci.fx, ci.fy, ci.ppx, ci.ppy

    # Pre-built sub-sample grid of pixel coordinates.
    us = np.arange(0, W, PIXEL_STEP)
    vs = np.arange(0, H, PIXEL_STEP)
    uu, vv = np.meshgrid(us, vs)            # (gh, gw)
    uu = uu.ravel().astype(np.float64)
    vv = vv.ravel().astype(np.float64)

    node_buf = np.zeros((MAX_NODES, 3), dtype=np.float64)
    edge_buf = np.zeros((MAX_EDGES, 2), dtype=np.int32)

    win = "DD-GNG RealSense (RGB overlay)"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        print("[ddgng] running - press ESC or q in the window to quit")
    else:
        print("[ddgng] running headless")

    t_prev = time.time()
    fps = 0.0
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            df = frames.get_depth_frame()
            cf = frames.get_color_frame()
            if not df or not cf:
                continue

            color = np.asanyarray(cf.get_data())
            depth = np.asanyarray(df.get_data())            # uint16, z16

            # --- deproject sub-sampled valid pixels to 3D points (metres) ----
            z = depth[vs][:, us].ravel().astype(np.float64) * depth_scale
            m = (z > Z_MIN) & (z < Z_MAX)
            zf = z[m]
            xf = (uu[m] - ppx) / fx * zf
            yf = (vv[m] - ppy) / fy * zf
            pts = np.ascontiguousarray(np.stack([xf, yf, zf], axis=1))
            n = pts.shape[0]

            if n > 0:
                cx, cy, cz = float(xf.mean()), float(yf.mean()), float(zf.mean())
                lib.ddgng_feed(
                    net, pts.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                    n, cx, cy, cz, GNG_ITERS)

            # --- read GNG back and overlay on the colour image ---------------
            mnode = lib.ddgng_get_nodes(
                net, node_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), MAX_NODES)
            kedge = lib.ddgng_get_edges(
                net, edge_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_int)), MAX_EDGES)

            # Project nodes (pinhole) -> pixels; guard against z<=0.
            nz = node_buf[:mnode, 2].copy()
            nz[nz <= 1e-6] = 1e-6
            px = (node_buf[:mnode, 0] / nz * fx + ppx)
            py = (node_buf[:mnode, 1] / nz * fy + ppy)
            uv = np.stack([px, py], axis=1).astype(np.int32)

            # edges (green) first, then nodes (red) on top
            for a, b in edge_buf[:kedge]:
                if a < mnode and b < mnode:
                    cv2.line(color, tuple(uv[a]), tuple(uv[b]), (0, 255, 0), 1, cv2.LINE_AA)
            for (x, y) in uv:
                if 0 <= x < W and 0 <= y < H:
                    cv2.circle(color, (x, y), 2, (0, 0, 255), -1, cv2.LINE_AA)

            # HUD
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            cv2.putText(color, f"nodes={mnode} edges={kedge} pts={n} fps={fps:4.1f}",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            if ros_publisher is not None:
                ok, jpeg = cv2.imencode(
                    ".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = ros_node.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_color_optical_frame"
                    msg.format = "jpeg"
                    msg.data = jpeg.tobytes()
                    ros_publisher.publish(msg)

            if not args.headless:
                cv2.imshow(win, color)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
    finally:
        pipe.stop()
        lib.ddgng_destroy(net)
        if not args.headless:
            cv2.destroyAllWindows()
        if ros_node is not None:
            ros_node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
