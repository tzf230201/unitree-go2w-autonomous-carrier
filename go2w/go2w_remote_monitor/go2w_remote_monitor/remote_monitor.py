#!/usr/bin/env python3
import os
import struct
import time
import threading

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

# Auto-detect DISPLAY for NoMachine
if not os.environ.get("DISPLAY"):
    try:
        for f in sorted(os.listdir("/tmp/.X11-unix/")):
            if f.startswith("X"):
                os.environ["DISPLAY"] = ":" + f[1:]
                break
    except FileNotFoundError:
        pass

BUTTON_MAP = {
    0: "R1",
    1: "L1",
    2: "Start",
    3: "Select",
    4: "R2",
    5: "L2",
    6: "F1",
    7: "F3",
    8: "A",
    9: "B",
    10: "X",
    11: "Y",
    12: "Up",
    13: "Right",
    14: "Down",
    15: "Left",
}


class RemoteMonitor(Node):
    def __init__(self):
        super().__init__("remote_monitor")
        self.lowstate_topic = self.declare_parameter("lowstate_topic", "/lowstate").value
        self.prev_buttons = set()
        self.current_buttons = set()
        self.sticks = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        self.raw_key = 0
        self.last_received_time = None
        self.sub = self.create_subscription(
            LowState, self.lowstate_topic, self.callback, 10
        )
        self.get_logger().info(
            f"Monitoring remote controller data from '{self.lowstate_topic}'. Press Ctrl+C to stop."
        )

    @staticmethod
    def decode_axes(wr):
        wr_bytes = bytes(wr)
        return {
            "lx": struct.unpack_from('<f', wr_bytes, 4)[0],
            "rx": struct.unpack_from('<f', wr_bytes, 8)[0],
            "ry": struct.unpack_from('<f', wr_bytes, 12)[0],
            "ly": struct.unpack_from('<f', wr_bytes, 20)[0],
        }

    @staticmethod
    def decode_buttons(wr):
        key_value = wr[2] | (wr[3] << 8)
        pressed = {name for bit, name in BUTTON_MAP.items() if key_value & (1 << bit)}
        return key_value, pressed

    def callback(self, msg):
        wr = msg.wireless_remote
        key_value, pressed = self.decode_buttons(wr)
        axes = self.decode_axes(wr)
        self.last_received_time = time.time()

        self.sticks["lx"] = axes["lx"]
        self.sticks["ly"] = axes["ly"]
        self.sticks["rx"] = axes["rx"]
        self.sticks["ry"] = axes["ry"]
        self.raw_key = key_value
        self.current_buttons = pressed

        if pressed != self.prev_buttons:
            if pressed:
                self.get_logger().info(
                    "Buttons: %s (raw=0x%04X) Sticks: lx=%+.2f ly=%+.2f rx=%+.2f ry=%+.2f"
                    % (", ".join(sorted(pressed)), key_value,
                       axes["lx"], axes["ly"], axes["rx"], axes["ry"])
                )
            else:
                self.get_logger().info("Released all buttons")
            self.prev_buttons = pressed


def run_gui(node):
    import tkinter as tk

    root = tk.Tk()
    root.title("Unitree Go2-W Remote Monitor")
    root.configure(bg="#1e1e1e")
    root.resizable(False, False)

    # Colors
    BG = "#1e1e1e"
    BODY_COLOR = "#3a3a3a"
    GRIP_COLOR = "#2d2d2d"
    BTN_OFF = "#555555"
    BTN_ON = "#00e070"
    TEXT = "#ffffff"
    TEXT_DIM = "#999999"
    STICK_BG = "#2a2a2a"
    STICK_DOT = "#00aaff"
    SCREEN_BG = "#0a0a0a"
    SHOULDER_OFF = "#4a4a4a"
    SHOULDER_ON = "#00e070"

    W = 700
    H = 480
    canvas = tk.Canvas(root, width=W, height=H, bg=BG, highlightthickness=0)
    canvas.pack(padx=10, pady=10)

    # ── Draw controller body ──
    def rounded_rect(c, x1, y1, x2, y2, r, **kw):
        points = [
            x1+r, y1, x2-r, y1, x2, y1, x2, y1+r,
            x2, y2-r, x2, y2, x2-r, y2, x1+r, y2,
            x1, y2, x1, y2-r, x1, y1+r, x1, y1,
        ]
        return c.create_polygon(points, smooth=True, **kw)

    # Controller body
    rounded_rect(canvas, 50, 60, 650, 440, 40, fill=BODY_COLOR, outline="#4a4a4a")
    # Left grip
    rounded_rect(canvas, 30, 100, 160, 450, 30, fill=GRIP_COLOR, outline="#3a3a3a")
    # Right grip
    rounded_rect(canvas, 540, 100, 670, 450, 30, fill=GRIP_COLOR, outline="#3a3a3a")
    # Screen area (center)
    rounded_rect(canvas, 170, 100, 530, 340, 10, fill=SCREEN_BG, outline="#333333")

    # Title on screen
    canvas.create_text(350, 130, text="Unitree Go2-W Remote Monitor",
                       font=("monospace", 13, "bold"), fill="#00cc66")
    # Raw value on screen
    raw_text = canvas.create_text(350, 160, text="Button Raw: 0x0000",
                                  font=("monospace", 11), fill=TEXT_DIM)
    # Pressed buttons text on screen
    pressed_text = canvas.create_text(350, 190, text="Pressed: -",
                                       font=("monospace", 10), fill="#00aaff")
    # Status indicator on screen
    status_text = canvas.create_text(350, 325, text="WAITING FOR /lowstate",
                                      font=("monospace", 9), fill="#666666")

    # ── Shoulder buttons ──
    shoulder_items = {}

    def draw_shoulder(cx, cy, w, h, label):
        r = rounded_rect(canvas, cx - w//2, cy - h//2, cx + w//2, cy + h//2, 8,
                          fill=SHOULDER_OFF, outline="#666666")
        t = canvas.create_text(cx, cy, text=label, font=("monospace", 10, "bold"), fill=TEXT)
        shoulder_items[label] = (r, t)

    # L2 on top, L1 below
    draw_shoulder(75, 75, 60, 28, "L2")
    draw_shoulder(75, 108, 60, 28, "L1")
    # R2 on top, R1 below
    draw_shoulder(625, 75, 60, 28, "R2")
    draw_shoulder(625, 108, 60, 28, "R1")

    # ── Analog sticks ──
    STICK_R = 40
    DOT_R = 8

    def make_stick_on_canvas(cx, cy, label):
        canvas.create_oval(cx - STICK_R, cy - STICK_R, cx + STICK_R, cy + STICK_R,
                           fill=STICK_BG, outline="#555555", width=2)
        canvas.create_line(cx - STICK_R + 5, cy, cx + STICK_R - 5, cy,
                           fill="#444444", dash=(2, 3))
        canvas.create_line(cx, cy - STICK_R + 5, cx, cy + STICK_R - 5,
                           fill="#444444", dash=(2, 3))
        dot = canvas.create_oval(cx - DOT_R, cy - DOT_R, cx + DOT_R, cy + DOT_R,
                                 fill=STICK_DOT, outline="#0088cc", width=1)
        val = canvas.create_text(cx, cy + STICK_R + 15, text="0.00, 0.00",
                                 font=("monospace", 8), fill=TEXT_DIM)
        canvas.create_text(cx, cy - STICK_R - 12, text=label,
                           font=("monospace", 9), fill=TEXT_DIM)
        return dot, val, cx, cy

    left_dot, left_val, lcx, lcy = make_stick_on_canvas(95, 200, "L-Stick")
    right_dot, right_val, rcx, rcy = make_stick_on_canvas(605, 200, "R-Stick")

    def update_stick(dot, val_id, cx, cy, sx, sy):
        nx = cx + int(sx * (STICK_R - DOT_R))
        ny = cy - int(sy * (STICK_R - DOT_R))
        canvas.coords(dot, nx - DOT_R, ny - DOT_R, nx + DOT_R, ny + DOT_R)
        canvas.itemconfig(val_id, text=f"{sx:+.2f}, {sy:+.2f}")

    # ── D-Pad ──
    dpad_items = {}
    dpad_cx, dpad_cy = 95, 330
    dpad_size = 28

    def draw_dpad_btn(cx, cy, label):
        r = rounded_rect(canvas, cx - dpad_size//2, cy - dpad_size//2,
                          cx + dpad_size//2, cy + dpad_size//2, 5,
                          fill=BTN_OFF, outline="#666666")
        arrows = {"Up": "\u25b2", "Down": "\u25bc", "Left": "\u25c0", "Right": "\u25b6"}
        t = canvas.create_text(cx, cy, text=arrows.get(label, label),
                               font=("monospace", 10, "bold"), fill=TEXT)
        dpad_items[label] = (r, t)

    draw_dpad_btn(dpad_cx, dpad_cy - 32, "Up")
    draw_dpad_btn(dpad_cx, dpad_cy + 32, "Down")
    draw_dpad_btn(dpad_cx - 32, dpad_cy, "Left")
    draw_dpad_btn(dpad_cx + 32, dpad_cy, "Right")
    canvas.create_oval(dpad_cx - 8, dpad_cy - 8, dpad_cx + 8, dpad_cy + 8,
                       fill="#3a3a3a", outline="#555555")

    # ── Face buttons A B X Y ──
    face_items = {}
    face_cx, face_cy = 605, 330
    face_r = 18

    def draw_face_btn(cx, cy, label):
        colors = {"A": "#00aa44", "B": "#cc3333", "X": "#3366cc", "Y": "#ccaa00"}
        c = colors.get(label, BTN_OFF)
        r = canvas.create_oval(cx - face_r, cy - face_r, cx + face_r, cy + face_r,
                               fill=BTN_OFF, outline="#666666", width=2)
        t = canvas.create_text(cx, cy, text=label, font=("monospace", 11, "bold"), fill=TEXT)
        face_items[label] = (r, t, c)

    draw_face_btn(face_cx, face_cy - 30, "Y")
    draw_face_btn(face_cx - 30, face_cy, "X")
    draw_face_btn(face_cx + 30, face_cy, "B")
    draw_face_btn(face_cx, face_cy + 30, "A")

    # ── Center buttons ──
    center_items = {}

    def draw_center_btn(cx, cy, label, w=50, h=24):
        r = rounded_rect(canvas, cx - w//2, cy - h//2, cx + w//2, cy + h//2, 6,
                          fill=BTN_OFF, outline="#666666")
        t = canvas.create_text(cx, cy, text=label, font=("monospace", 9, "bold"), fill=TEXT)
        center_items[label] = (r, t)

    # Horizontal pairs
    draw_center_btn(220, 390, "Select")
    draw_center_btn(290, 390, "F1")
    draw_center_btn(410, 390, "F3")
    draw_center_btn(480, 390, "Start")

    # ── Stick value display on screen ──
    lx_text = canvas.create_text(260, 220, text="LX: +0.00", font=("monospace", 10), fill=TEXT_DIM)
    ly_text = canvas.create_text(260, 240, text="LY: +0.00", font=("monospace", 10), fill=TEXT_DIM)
    rx_text = canvas.create_text(440, 220, text="RX: +0.00", font=("monospace", 10), fill=TEXT_DIM)
    ry_text = canvas.create_text(440, 240, text="RY: +0.00", font=("monospace", 10), fill=TEXT_DIM)

    # Divider line on screen
    canvas.create_line(350, 210, 350, 330, fill="#333333", dash=(3, 3))

    # ── Poll loop ──
    def poll():
        pressed = node.current_buttons
        sticks = node.sticks
        raw = node.raw_key

        # Shoulder
        for label in ("L1", "L2", "R1", "R2"):
            if label in shoulder_items:
                r, t = shoulder_items[label]
                canvas.itemconfig(r, fill=SHOULDER_ON if label in pressed else SHOULDER_OFF)

        # D-pad
        for label in ("Up", "Down", "Left", "Right"):
            if label in dpad_items:
                r, t = dpad_items[label]
                canvas.itemconfig(r, fill=BTN_ON if label in pressed else BTN_OFF)

        # Face buttons
        for label in ("A", "B", "X", "Y"):
            if label in face_items:
                r, t, c = face_items[label]
                canvas.itemconfig(r, fill=c if label in pressed else BTN_OFF)

        # Center buttons
        for label in ("Select", "F1", "F3", "Start"):
            if label in center_items:
                r, t = center_items[label]
                canvas.itemconfig(r, fill=BTN_ON if label in pressed else BTN_OFF)

        # Sticks
        update_stick(left_dot, left_val, lcx, lcy, sticks["lx"], sticks["ly"])
        update_stick(right_dot, right_val, rcx, rcy, sticks["rx"], sticks["ry"])

        # Screen info
        canvas.itemconfig(raw_text, text=f"Button Raw: 0x{raw:04X}")
        if pressed:
            canvas.itemconfig(pressed_text, text=f"Pressed: {', '.join(sorted(pressed))}")
        else:
            canvas.itemconfig(pressed_text, text="Pressed: -")

        canvas.itemconfig(lx_text, text=f"LX: {sticks['lx']:+.2f}")
        canvas.itemconfig(ly_text, text=f"LY: {sticks['ly']:+.2f}")
        canvas.itemconfig(rx_text, text=f"RX: {sticks['rx']:+.2f}")
        canvas.itemconfig(ry_text, text=f"RY: {sticks['ry']:+.2f}")

        # Status
        online = node.last_received_time is not None and (time.time() - node.last_received_time) < 1.0
        if online:
            canvas.itemconfig(status_text, text="ONLINE", fill="#00cc66")
        else:
            canvas.itemconfig(status_text, text="WAITING FOR /lowstate", fill="#666666")

        root.after(50, poll)

    poll()

    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()


def main():
    rclpy.init()
    node = RemoteMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    run_gui(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
