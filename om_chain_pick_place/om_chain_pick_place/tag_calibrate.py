"""Interactive camera-extrinsic calibration from AprilTag observations.

Run the apriltag_detector first, then this script in another terminal:

    ros2 run om_chain_pick_place tag_calibrate

Procedure: place the tag cube at N >= 3 known positions (measured with a
ruler from the ARM BASE: x forward, y left, z up; tag CENTER height =
cube height if the tag is on top). For each position, type the measured
"x y z" and the script averages ~1 s of detections. At the end it solves
the rigid transform (Kabsch) mapping camera-optical points onto world
points and prints the `camera_xyz` / `camera_rpy` block ready to paste
into tag_pick.yaml.

Spread the points out (not in a straight line!) — collinear points leave
the rotation around that line unobservable.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# camera BODY (x fwd, y left, z up) → OPTICAL (x right, y down, z fwd)
R_BODY_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rotation_to_rpy(R: np.ndarray) -> tuple:
    """ZYX (yaw-pitch-roll) Euler angles from a rotation matrix."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


class Collector(Node):
    def __init__(self):
        super().__init__("tag_calibrate")
        self._lock = threading.Lock()
        self._buf = []
        self.create_subscription(PoseStamped, "/apriltag/pose", self._cb, 10)

    def _cb(self, msg: PoseStamped):
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        with self._lock:
            self._buf.append(p)

    def collect(self, seconds: float = 1.5, min_n: int = 5):
        with self._lock:
            self._buf = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        with self._lock:
            pts = list(self._buf)
        if len(pts) < min_n:
            return None
        return np.median(np.stack(pts), axis=0)


def solve_extrinsic(p_opt: np.ndarray, p_world: np.ndarray):
    """Kabsch: find R, t with p_world ≈ R @ p_opt + t."""
    co = p_opt.mean(axis=0)
    cw = p_world.mean(axis=0)
    H = (p_opt - co).T @ (p_world - cw)
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cw - R @ co
    resid = np.linalg.norm((R @ p_opt.T).T + t - p_world, axis=1)
    return R, t, resid, S


def main():
    rclpy.init()
    node = Collector()
    print(__doc__)
    obs_opt, obs_world = [], []
    try:
        while True:
            n = len(obs_opt)
            prompt = (f"\n[{n} titik terkumpul] Letakkan kubus, lalu ketik "
                      "posisi PUSAT TAG dari pangkal arm dalam meter "
                      "(mis: 0.25 0.0 0.03), atau 'selesai': ")
            line = input(prompt).strip().lower()
            if line in ("selesai", "done", "q", ""):
                if n >= 3:
                    break
                print(f"Butuh minimal 3 titik (baru {n}).")
                continue
            try:
                world = np.array([float(v) for v in line.split()])
                assert world.shape == (3,)
            except (ValueError, AssertionError):
                print("Format salah — contoh: 0.25 0.0 0.03")
                continue
            print("  mengambil deteksi ~1.5 s ...")
            opt = node.collect()
            if opt is None:
                print("  ✗ tag tidak terdeteksi cukup — perbaiki posisi/cahaya, ulangi.")
                continue
            print(f"  ✓ tag di frame kamera: ({opt[0]:+.3f}, {opt[1]:+.3f}, {opt[2]:+.3f})")
            obs_opt.append(opt)
            obs_world.append(world)
    except (KeyboardInterrupt, EOFError):
        print("\ndibatalkan")
        if len(obs_opt) < 3:
            rclpy.shutdown()
            return

    R_wo, t, resid, svals = solve_extrinsic(
        np.stack(obs_opt), np.stack(obs_world))
    R_wb = R_wo @ R_BODY_OPTICAL.T
    r, p, y = rotation_to_rpy(R_wb)

    print("\n================ HASIL ================")
    print(f"residual per titik (m): {np.round(resid, 4)}")
    if resid.max() > 0.03:
        print("⚠ residual > 3 cm — ada titik salah ukur atau deteksi jelek.")
    if svals[1] < 1e-4:
        print("⚠ titik-titik hampir segaris — rotasi tidak andal, tambah titik menyebar.")
    print("\nTempel ke tag_pick.yaml (bagian tag_pick_place):")
    print(f"    camera_xyz: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
    print(f"    camera_rpy: [{r:.4f}, {p:.4f}, {y:.4f}]")
    print("\nLalu restart tag_pick_place dan cek ulang dengan /tag_world.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
