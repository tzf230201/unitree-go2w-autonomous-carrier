"""
Tkinter GUI for OpenManipulator-X.

Provides on-screen buttons:
  Maju / Mundur          : EE forward / backward along its current yaw direction
  Yaw CCW / Yaw CW       : rotate whole arm around the base z-axis
  Naik / Turun           : EE up / down (world z)
  Pitch + / Pitch -      : tilt EE up / down
  Buka / Tutup gripper   : open / close gripper
Plus utility buttons: SYNC (re-read current pose as target), READY, HOME UP,
TORQUE ON / OFF, and a step-size slider.

The GUI tracks an internal Cartesian target (r, yaw, z, pitch). Each button
press updates that target, solves IK, and publishes a JointState command on
/open_manipulator_x/joint_command. Joint state is read from
/open_manipulator_x/joint_states and shown live.

The GUI never teleports the arm: on startup the target is initialised from
the current measured joint angles via FK.
"""

from __future__ import annotations

import math
import threading
import tkinter as tk
from tkinter import ttk
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

try:
    from cv_bridge import CvBridge
    import cv2
    from PIL import Image as PilImage, ImageTk
    _CAMERA_DEPS_OK = True
except Exception as _exc:  # camera optional
    _CAMERA_DEPS_OK = False
    _CAMERA_IMPORT_ERROR = _exc

from .kinematics import (
    forward_kinematics,
    inverse_kinematics,
    IKError,
    workspace_limits,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_NAME = "gripper"

TOPIC_STATE = "/open_manipulator_x/joint_states"
TOPIC_JOINT_CMD = "/open_manipulator_x/joint_command"
TOPIC_GRIPPER_CMD = "/open_manipulator_x/gripper_command"
TOPIC_CAMERA = "/camera/color/image_raw"
SRV_TORQUE = "/open_manipulator_x/enable_torque"
SRV_GO_HOME = "/open_manipulator_x/go_home"
SRV_GO_READY = "/open_manipulator_x/go_ready"

CAMERA_DISPLAY_W = 480
CAMERA_DISPLAY_H = 360


class GuiNode(Node):
    """rclpy node. The Tkinter side is in OmxGui which owns this node and
    spins it from a background thread."""

    def __init__(self) -> None:
        super().__init__("open_manipulator_x_gui")

        self.declare_parameter("camera_topic", TOPIC_CAMERA)
        self.declare_parameter("grip_open_target", -1.0)
        self.declare_parameter("grip_close_target", 0.0)
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.grip_open_target_param = float(self.get_parameter("grip_open_target").value)
        self.grip_close_target_param = float(self.get_parameter("grip_close_target").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Camera publishers usually use SENSOR_DATA (best effort). Match that
        # so we actually receive frames.
        qos_image = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_joint = self.create_publisher(JointState, TOPIC_JOINT_CMD, qos)
        self.pub_grip = self.create_publisher(Float64, TOPIC_GRIPPER_CMD, qos)
        self.sub_state = self.create_subscription(
            JointState, TOPIC_STATE, self._on_state, qos
        )

        self.cli_torque = self.create_client(SetBool, SRV_TORQUE)
        self.cli_home = self.create_client(Trigger, SRV_GO_HOME)
        self.cli_ready = self.create_client(Trigger, SRV_GO_READY)

        self._state_lock = threading.Lock()
        self.last_positions: dict[str, float] = {}

        # ---- camera ----
        self._bridge: Optional["CvBridge"] = None
        self._latest_cv_frame = None       # numpy BGR; None until first frame
        self._frame_counter = 0
        if _CAMERA_DEPS_OK:
            self._bridge = CvBridge()
            self.sub_image = self.create_subscription(
                Image, camera_topic, self._on_image, qos_image
            )
            self.get_logger().info(f"Camera subscribed: {camera_topic}")
        else:
            self.get_logger().warn(
                f"Camera disabled — missing deps: {_CAMERA_IMPORT_ERROR}"
            )

    def _on_state(self, msg: JointState) -> None:
        with self._state_lock:
            for n, p in zip(msg.name, msg.position):
                self.last_positions[n] = p

    def get_last_positions(self) -> dict[str, float]:
        with self._state_lock:
            return dict(self.last_positions)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image convert fail: {exc}", throttle_duration_sec=2.0)
            return
        # atomic Python ref-swap; Tk thread reads the latest
        self._latest_cv_frame = frame
        self._frame_counter += 1

    def take_latest_frame(self):
        """Tk-thread consumer. Returns (frame, counter) or (None, _) if nothing new."""
        return self._latest_cv_frame, self._frame_counter

    # ---- publishing helpers ----
    def publish_joint_targets(self, q1: float, q2: float, q3: float, q4: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [q1, q2, q3, q4]
        self.pub_joint.publish(msg)

    def publish_gripper(self, position: float) -> None:
        m = Float64()
        m.data = float(position)
        self.pub_grip.publish(m)

    # ---- services (fire-and-forget; caller doesn't block on response) ----
    def call_torque(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = on
        if self.cli_torque.service_is_ready():
            self.cli_torque.call_async(req)
        else:
            self.get_logger().warn(f"{SRV_TORQUE} not available")

    def call_home(self) -> None:
        if self.cli_home.service_is_ready():
            self.cli_home.call_async(Trigger.Request())
        else:
            self.get_logger().warn(f"{SRV_GO_HOME} not available")

    def call_ready(self) -> None:
        if self.cli_ready.service_is_ready():
            self.cli_ready.call_async(Trigger.Request())
        else:
            self.get_logger().warn(f"{SRV_GO_READY} not available")


class OmxGui:
    def __init__(self, node: GuiNode) -> None:
        self.node = node

        # internal target — (r, yaw, z, pitch) in metres / radians.
        # Will be replaced by FK of first received joint_state.
        self._initialised = False
        self.r = 0.20
        self.yaw = 0.0
        self.z = 0.20
        self.pitch = 0.0

        # last command sent / last measured (filled in by _tick); kept for display
        self._sent_grip: Optional[float] = None

        self.step_lin = 0.01     # m per click
        self.step_ang = math.radians(5)   # rad per click

        # config / IK limits
        self.r_min, self.r_max = 0.05, 0.32   # conservative radial range
        self.z_min, self.z_max = 0.02, 0.45
        self.pitch_min, self.pitch_max = -math.pi / 2.0, math.pi / 2.0

        # Gripper: absolute target values (rad). Initial values come from the
        # ROS parameters on the node (overridable via launch). User can refine
        # at runtime with "Set OPEN here" / "Set CLOSE here".
        self.grip_open_target = self.node.grip_open_target_param
        self.grip_close_target = self.node.grip_close_target_param
        self.gripper_safety_min = -math.pi
        self.gripper_safety_max = +math.pi

        # ---- Tk root ----
        self.root = tk.Tk()
        self.root.title("OpenManipulator-X — Jog GUI")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build_ui()

        # poll: update displayed state and try to sync the initial target
        self.root.after(100, self._tick)

    # -------------------- UI build --------------------
    def _build_ui(self) -> None:
        pad = {"padx": 6, "pady": 4}
        big_btn = {"width": 14, "height": 2}

        # left column = controls, right column = camera
        frm_left = ttk.Frame(self.root)
        frm_left.grid(row=0, column=0, sticky="nsew")
        frm_right = ttk.LabelFrame(self.root, text="RealSense view")
        frm_right.grid(row=0, column=1, sticky="nsew", padx=6, pady=4)

        frm_top = ttk.Frame(frm_left)
        frm_top.grid(row=0, column=0, sticky="nsew", **pad)

        # current measured + target display
        self.lbl_meas = ttk.Label(frm_top, text="meas: --", font=("TkFixedFont", 10))
        self.lbl_meas.grid(row=0, column=0, sticky="w")
        self.lbl_target = ttk.Label(frm_top, text="target: --", font=("TkFixedFont", 10))
        self.lbl_target.grid(row=1, column=0, sticky="w")
        self.lbl_status = ttk.Label(frm_top, text="status: warming up", foreground="orange")
        self.lbl_status.grid(row=2, column=0, sticky="w")

        # ---- jog grid ----
        frm_jog = ttk.LabelFrame(frm_left, text="Jog Cartesian")
        frm_jog.grid(row=1, column=0, sticky="nsew", **pad)

        # row 0: Naik (up)
        tk.Button(frm_jog, text="Naik (+Z)", bg="#dfe", command=lambda: self._jog("z", +1), **big_btn).grid(row=0, column=1, **pad)
        # row 1: Mundur / Pitch buttons / Maju
        tk.Button(frm_jog, text="Mundur (-X)", bg="#fee", command=lambda: self._jog("r", -1), **big_btn).grid(row=1, column=0, **pad)
        tk.Button(frm_jog, text="Pitch +", command=lambda: self._jog("pitch", +1), **big_btn).grid(row=1, column=1, **pad)
        tk.Button(frm_jog, text="Maju (+X)", bg="#efe", command=lambda: self._jog("r", +1), **big_btn).grid(row=1, column=2, **pad)
        # row 2: Yaw CCW / Turun / Yaw CW
        tk.Button(frm_jog, text="Yaw CCW", command=lambda: self._jog("yaw", +1), **big_btn).grid(row=2, column=0, **pad)
        tk.Button(frm_jog, text="Turun (-Z)", bg="#fed", command=lambda: self._jog("z", -1), **big_btn).grid(row=2, column=1, **pad)
        tk.Button(frm_jog, text="Yaw CW", command=lambda: self._jog("yaw", -1), **big_btn).grid(row=2, column=2, **pad)
        # row 3: Pitch -
        tk.Button(frm_jog, text="Pitch -", command=lambda: self._jog("pitch", -1), **big_btn).grid(row=3, column=1, **pad)

        # ---- gripper ----
        frm_grip = ttk.LabelFrame(frm_left, text="Gripper")
        frm_grip.grid(row=2, column=0, sticky="nsew", **pad)

        tk.Button(frm_grip, text="BUKA", bg="#cef", width=12, height=2,
                  command=self._gripper_open).grid(row=0, column=0, **pad)
        tk.Button(frm_grip, text="TUTUP", bg="#fcb", width=12, height=2,
                  command=self._gripper_close).grid(row=0, column=1, **pad)
        self.lbl_grip = ttk.Label(frm_grip, text="meas grip: --   sent: --",
                                   font=("TkFixedFont", 9))
        self.lbl_grip.grid(row=0, column=2, padx=10, sticky="w")

        # calibration row: shows currently saved open/close targets + buttons
        self.lbl_grip_targets = ttk.Label(
            frm_grip,
            text=f"OPEN={self.grip_open_target:+.2f}  CLOSE={self.grip_close_target:+.2f}",
            font=("TkFixedFont", 9),
        )
        self.lbl_grip_targets.grid(row=1, column=0, columnspan=3, sticky="w", padx=6)

        tk.Button(frm_grip, text="Set OPEN here",
                  command=self._set_open_here).grid(row=2, column=0, **pad)
        tk.Button(frm_grip, text="Set CLOSE here",
                  command=self._set_close_here).grid(row=2, column=1, **pad)
        ttk.Label(frm_grip,
                   text="(Torque OFF → buka/tutup\nrahang dgn tangan → klik)",
                   foreground="gray").grid(row=2, column=2, sticky="w", padx=6)

        # ---- step controls + utility ----
        frm_set = ttk.LabelFrame(frm_left, text="Step size & utility")
        frm_set.grid(row=3, column=0, sticky="nsew", **pad)

        ttk.Label(frm_set, text="Step linear (m)").grid(row=0, column=0, sticky="w")
        self.var_lin = tk.DoubleVar(value=self.step_lin)
        sc_lin = ttk.Scale(frm_set, from_=0.005, to=0.05, orient="horizontal",
                            variable=self.var_lin, command=self._on_step_lin)
        sc_lin.grid(row=0, column=1, sticky="ew")
        self.lbl_lin = ttk.Label(frm_set, text=f"{self.step_lin*1000:.0f} mm")
        self.lbl_lin.grid(row=0, column=2)

        ttk.Label(frm_set, text="Step angular (deg)").grid(row=1, column=0, sticky="w")
        self.var_ang = tk.DoubleVar(value=math.degrees(self.step_ang))
        sc_ang = ttk.Scale(frm_set, from_=1, to=20, orient="horizontal",
                            variable=self.var_ang, command=self._on_step_ang)
        sc_ang.grid(row=1, column=1, sticky="ew")
        self.lbl_ang = ttk.Label(frm_set, text=f"{math.degrees(self.step_ang):.0f}°")
        self.lbl_ang.grid(row=1, column=2)

        tk.Button(frm_set, text="SYNC target ← measured", command=self._sync_target).grid(row=2, column=0, columnspan=3, sticky="ew", **pad)
        tk.Button(frm_set, text="Go READY", command=self.node.call_ready).grid(row=3, column=0, sticky="ew", **pad)
        tk.Button(frm_set, text="Go HOME UP", command=self.node.call_home).grid(row=3, column=1, sticky="ew", **pad)
        tk.Button(frm_set, text="Torque ON", bg="#cfc", command=lambda: self.node.call_torque(True)).grid(row=4, column=0, sticky="ew", **pad)
        tk.Button(frm_set, text="Torque OFF", bg="#fcc", command=lambda: self.node.call_torque(False)).grid(row=4, column=1, sticky="ew", **pad)

        frm_set.columnconfigure(1, weight=1)

        # ---- camera panel (right column) ----
        if _CAMERA_DEPS_OK:
            placeholder = PilImage.new("RGB", (CAMERA_DISPLAY_W, CAMERA_DISPLAY_H), "black")
            self._imgtk = ImageTk.PhotoImage(placeholder)
            self.lbl_cam = ttk.Label(frm_right, image=self._imgtk)
            self.lbl_cam.grid(row=0, column=0, padx=4, pady=4)
            self.lbl_cam_info = ttk.Label(frm_right, text="waiting for frames...",
                                           foreground="orange",
                                           font=("TkFixedFont", 9))
            self.lbl_cam_info.grid(row=1, column=0, sticky="w", padx=4)
        else:
            self.lbl_cam = None
            self.lbl_cam_info = ttk.Label(frm_right, text="camera disabled (deps missing)")
            self.lbl_cam_info.grid(row=0, column=0, padx=10, pady=10)

        self._last_frame_count = 0
        self._fps_window_t0 = None
        self._fps_window_n = 0

    # -------------------- jog logic --------------------
    def _on_step_lin(self, _val) -> None:
        self.step_lin = float(self.var_lin.get())
        self.lbl_lin.config(text=f"{self.step_lin*1000:.0f} mm")

    def _on_step_ang(self, _val) -> None:
        self.step_ang = math.radians(float(self.var_ang.get()))
        self.lbl_ang.config(text=f"{math.degrees(self.step_ang):.0f}°")

    def _jog(self, axis: str, sign: int) -> None:
        if not self._initialised:
            self._set_status("waiting for joint_states...", "orange")
            return

        new_r, new_yaw, new_z, new_pitch = self.r, self.yaw, self.z, self.pitch
        if axis == "r":
            new_r = self.r + sign * self.step_lin
        elif axis == "yaw":
            new_yaw = self.yaw + sign * self.step_ang
        elif axis == "z":
            new_z = self.z + sign * self.step_lin
        elif axis == "pitch":
            new_pitch = self.pitch + sign * self.step_ang

        # clamp
        new_r = max(self.r_min, min(self.r_max, new_r))
        new_z = max(self.z_min, min(self.z_max, new_z))
        new_pitch = max(self.pitch_min, min(self.pitch_max, new_pitch))

        x = new_r * math.cos(new_yaw)
        y = new_r * math.sin(new_yaw)
        try:
            q1, q2, q3, q4 = inverse_kinematics(x, y, new_z, new_pitch, elbow_up=True)
        except IKError as exc:
            self._set_status(f"IK fail: {exc}", "red")
            return

        # accept new target, publish
        self.r, self.yaw, self.z, self.pitch = new_r, new_yaw, new_z, new_pitch
        self.node.publish_joint_targets(q1, q2, q3, q4)
        self._set_status(
            f"sent q=({q1:.2f},{q2:.2f},{q3:.2f},{q4:.2f})", "green"
        )

    def _gripper_open(self) -> None:
        target = max(self.gripper_safety_min,
                     min(self.gripper_safety_max, self.grip_open_target))
        self.node.publish_gripper(target)
        self._set_status(f"gripper → OPEN ({target:+.2f} rad)", "green")
        self._sent_grip = target

    def _gripper_close(self) -> None:
        target = max(self.gripper_safety_min,
                     min(self.gripper_safety_max, self.grip_close_target))
        self.node.publish_gripper(target)
        self._set_status(f"gripper → CLOSE ({target:+.2f} rad)", "green")
        self._sent_grip = target

    def _set_open_here(self) -> None:
        pos = self.node.get_last_positions()
        if GRIPPER_NAME not in pos:
            self._set_status("can't calibrate: no gripper in /joint_states", "red")
            return
        self.grip_open_target = pos[GRIPPER_NAME]
        self._refresh_grip_targets()
        self._set_status(
            f"calibrated OPEN = {self.grip_open_target:+.2f} rad", "green"
        )

    def _set_close_here(self) -> None:
        pos = self.node.get_last_positions()
        if GRIPPER_NAME not in pos:
            self._set_status("can't calibrate: no gripper in /joint_states", "red")
            return
        self.grip_close_target = pos[GRIPPER_NAME]
        self._refresh_grip_targets()
        self._set_status(
            f"calibrated CLOSE = {self.grip_close_target:+.2f} rad", "green"
        )

    def _refresh_grip_targets(self) -> None:
        self.lbl_grip_targets.config(
            text=f"OPEN={self.grip_open_target:+.2f}  CLOSE={self.grip_close_target:+.2f}"
        )

    # -------------------- state sync --------------------
    def _sync_target(self) -> None:
        pos = self.node.get_last_positions()
        missing = [n for n in JOINT_NAMES if n not in pos]
        if missing:
            self._set_status(f"missing joints in state: {missing}", "red")
            return
        q1, q2, q3, q4 = (pos[n] for n in JOINT_NAMES)
        x, y, z, pitch = forward_kinematics(q1, q2, q3, q4)
        r = math.hypot(x, y)
        yaw = math.atan2(y, x) if r > 1e-6 else q1
        self.r, self.yaw, self.z, self.pitch = r, yaw, z, pitch
        self._initialised = True
        self._set_status("target synced to current pose", "green")

    def _tick(self) -> None:
        self._render_camera()
        pos = self.node.get_last_positions()
        if all(n in pos for n in JOINT_NAMES):
            q1, q2, q3, q4 = (pos[n] for n in JOINT_NAMES)
            x, y, z, pitch = forward_kinematics(q1, q2, q3, q4)
            self.lbl_meas.config(
                text=(f"meas:  x={x:+.3f}  y={y:+.3f}  z={z:+.3f}  "
                      f"pitch={math.degrees(pitch):+5.1f}°")
            )
            if not self._initialised:
                self._sync_target()
        else:
            self.lbl_meas.config(text="meas: waiting for /joint_states")

        # live gripper readout
        grip_meas = pos.get(GRIPPER_NAME)
        sent_txt = f"{self._sent_grip:+.2f}" if self._sent_grip is not None else "--"
        meas_txt = f"{grip_meas:+.2f}" if grip_meas is not None else "--"
        self.lbl_grip.config(text=f"meas grip: {meas_txt}   sent: {sent_txt}")

        if self._initialised:
            x = self.r * math.cos(self.yaw)
            y = self.r * math.sin(self.yaw)
            self.lbl_target.config(
                text=(f"trg:   x={x:+.3f}  y={y:+.3f}  z={self.z:+.3f}  "
                      f"pitch={math.degrees(self.pitch):+5.1f}°  "
                      f"yaw={math.degrees(self.yaw):+5.1f}°")
            )
        self.root.after(100, self._tick)

    def _render_camera(self) -> None:
        if not _CAMERA_DEPS_OK or self.lbl_cam is None:
            return
        frame, counter = self.node.take_latest_frame()
        if frame is None or counter == self._last_frame_count:
            return
        self._last_frame_count = counter
        try:
            h, w = frame.shape[:2]
            # fit while preserving aspect
            scale = min(CAMERA_DISPLAY_W / w, CAMERA_DISPLAY_H / h)
            new_w = max(1, int(round(w * scale)))
            new_h = max(1, int(round(h * scale)))
            resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            pil = PilImage.fromarray(rgb)
            # ImageTk reference must persist or Tk drops the image
            self._imgtk = ImageTk.PhotoImage(pil)
            self.lbl_cam.configure(image=self._imgtk)

            # crude FPS over a rolling 1-second window
            import time as _t
            now = _t.monotonic()
            if self._fps_window_t0 is None:
                self._fps_window_t0 = now
                self._fps_window_n = 0
            self._fps_window_n += 1
            elapsed = now - self._fps_window_t0
            if elapsed >= 1.0:
                fps = self._fps_window_n / elapsed
                self.lbl_cam_info.config(
                    text=f"{w}x{h} @ {fps:4.1f} Hz", foreground="green"
                )
                self._fps_window_t0 = now
                self._fps_window_n = 0
        except Exception as exc:
            self.lbl_cam_info.config(text=f"render err: {exc}", foreground="red")

    def _set_status(self, text: str, color: str = "black") -> None:
        self.lbl_status.config(text=f"status: {text}", foreground=color)

    # -------------------- lifecycle --------------------
    def run(self) -> None:
        self.root.mainloop()

    def _on_close(self) -> None:
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    # spin rclpy in a background thread so the Tk mainloop owns the foreground
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        gui = OmxGui(node)
        gui.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
