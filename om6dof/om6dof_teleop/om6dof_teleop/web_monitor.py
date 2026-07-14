"""Network-accessible web dashboard for the Go2W robot.

A dependency-free (stdlib http.server) dashboard you can open from any device
on the same network. It shows:

  * Robot status  — hostname, IPs, uptime, load, RAM, CPU temp, ROS_DOMAIN_ID,
                    whether /dev/ttyUSB0 (the U2D2 arm bus) is present, and
                    whether remote control currently owns the arm interfaces.
  * Nodes         — the live ROS 2 graph node list, with DUPLICATE node names
                    highlighted (two nodes with the same name = trouble).
  * Topics        — the live topic list with message types.
  * Camera        — a live MJPEG stream from the RealSense colour camera
                    (the device is released automatically when nobody is
                    watching, so it never blocks other users).
  * Remote toggle — a button that publishes a momentary F3 tap onto
                    /wirelesscontroller. F3 switches controller ownership
                    between autonomous and remote control.

The page itself refreshes ONLY when you reload / press Refresh (the camera is a
separate live stream). Data is read on demand.

Run:
    ros2 run om6dof_teleop web_monitor
    # then open http://<robot-ip>:8080  from a phone/laptop on the same net
Options via ROS params:  port (default 8080), camera_width/height/fps.
"""

from __future__ import annotations

import html
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional, Tuple
from urllib.parse import parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from unitree_go.msg import WirelessController, LowState
from std_msgs.msg import Bool, String

# unitree_api is only needed for the "Robot Agent" chat mode (direct motion
# control). Import it softly so the dashboard still runs on machines that lack it.
try:
    from unitree_api.msg import Request as SportRequest
except Exception:  # pragma: no cover
    SportRequest = None


BTN_F3 = 1 << 7
ARM_BUS_DEVICE = "/dev/ttyUSB0"

# Unitree sport-mode API ids (verified against the official Unitree ROS2
# ros2_sport_client.h / .cpp and go2w_cmd_vel_control_node.cpp).
SPORT_TOPIC = "/api/sport/request"
API_DAMP = 1001            # emergency stop — all joints damp (highest priority)
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005      # lie down
API_RECOVERY_STAND = 1006  # recover from a fall / lying to balanced stand
API_MOVE = 1008
API_SWITCH_GAIT = 1011     # {"data":d}  0/1/2
API_SPEED_LEVEL = 1015     # {"data":level}  -1/0/1

# Locomotion mode -> SwitchGait "data" code (Go2W: 0 default, 1 stair/terrain
# walking, 2 height-climbing — per the official Go2W high-level control doc).
GAIT_CODES = {"normal": 0, "terrain": 1, "climb": 2}
# Speed gear -> SpeedLevel "data" value. Only effective in the default gait (0).
SPEED_LEVELS = {"slow": -1, "normal": 0, "fast": 1}

# Conservative motion caps for chat-driven commands. Must match the ranges
# documented in skills/go2w_control_skill.md.
MAX_VX = 0.4
MAX_VY = 0.3
MAX_WZ = 0.8
MAX_DURATION = 8.0


# --------------------------------------------------------------------------- #
#  Node → OS process mapping, for the per-node kill button                     #
# --------------------------------------------------------------------------- #
def find_node_pids(name: str) -> List[int]:
    """Best-effort map a ROS 2 node name to the PID(s) hosting it by scanning
    /proc cmdlines. Preference order: an explicit `__node:=<name>` remap, then a
    matching executable basename, then a bare token match. Never returns our own
    PID (so the dashboard can't kill itself)."""
    me = os.getpid()
    exact: List[int] = []
    exe: List[int] = []
    token: List[int] = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                raw = f.read()
        except Exception:
            continue
        if not raw:
            continue
        args = [a.decode("utf-8", "replace") for a in raw.split(b"\x00") if a]
        if not args:
            continue
        if f"__node:={name}" in " ".join(args):
            exact.append(pid)
        elif any(os.path.basename(a) == name for a in args):
            exe.append(pid)
        elif name in args:
            token.append(pid)
    return exact or exe or token


def restart_self(delay: float = 0.6) -> None:
    """Restart this web_monitor process so code edits take effect — no terminal,
    no sudo. Re-execs the same Python program (os.execv keeps the PID); the fresh
    interpreter re-imports the package from the symlinked source. If the new code
    fails to boot, the process exits and systemd's Restart=on-failure recovers it.
    Delayed in a background thread so the HTTP response is flushed first."""
    def _do() -> None:
        time.sleep(delay)
        try:
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception:
            os._exit(1)  # hand off to systemd Restart=on-failure
    threading.Thread(target=_do, daemon=True).start()


def shutdown_self(cam=None, delay: float = 0.6) -> None:
    """Cleanly stop the web monitor and STAY stopped — releases the RealSense
    camera (so the AprilTag detector can use it) and exits with code 0.
    systemd's Restart=on-failure does NOT restart a clean exit, so the monitor
    stays down until manually started again. Delayed in a background thread so
    the HTTP response is flushed first."""
    def _do() -> None:
        time.sleep(delay)
        # release the camera first
        try:
            if cam is not None:
                cam.running = False
                th = getattr(cam, "_thread", None)
                if th is not None:
                    th.join(timeout=2.0)
        except Exception:
            pass
        os._exit(0)   # clean exit → systemd on-failure will NOT restart
    threading.Thread(target=_do, daemon=True).start()


def kill_node(name: str) -> str:
    """Kill the process(es) hosting `name`. SIGINT first (clean rclpy shutdown),
    escalate to SIGTERM after a grace period. Returns a human-readable result."""
    pids = find_node_pids(name)
    if not pids:
        return f"No killable process found for '{name}' (it may be our own node)."
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(1.5)
    escalated = []
    for pid in pids:
        try:
            os.kill(pid, 0)          # still alive?
            os.kill(pid, signal.SIGTERM)
            escalated.append(pid)
        except ProcessLookupError:
            pass
    note = f" (SIGTERM sent to {escalated})" if escalated else ""
    return f"Killed '{name}' → PID(s) {pids}{note}."


# --------------------------------------------------------------------------- #
#  ROS node: graph introspection + F3 publisher                               #
# --------------------------------------------------------------------------- #
class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_web_monitor")
        self.flash = ""  # one-shot banner (e.g. kill result), shown then cleared
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_remote = self.create_publisher(
            WirelessController, "/wirelesscontroller", qos
        )
        self.pub_operation_mode = self.create_publisher(
            String, "/om6dof/operation_mode", 10
        )
        # Sport-API publisher for chat-driven motion (Robot Agent mode). Only
        # created when unitree_api is available.
        self.pub_sport = (
            self.create_publisher(SportRequest, SPORT_TOPIC, 10)
            if SportRequest is not None else None
        )
        # Serialise motion: one active move at a time; a new command cancels the
        # previous one. _motion_gen bumps to signal the running loop to quit.
        self._motion_lock = threading.Lock()
        self._motion_gen = 0
        # Controller/teleop state is TRANSIENT_LOCAL, so the dashboard receives
        # the latest value even when it starts after the arm stack.
        self.gripper_state: Optional[str] = None
        grip_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, "/om6dof_teleop/gripper_state", self._on_gripper, grip_qos
        )
        self.remote_enabled: Optional[bool] = None
        self.create_subscription(
            Bool,
            "/om6dof/remote_enabled/state",
            self._on_remote_enabled,
            grip_qos,
        )
        self.control_mode: Optional[str] = None
        self.create_subscription(
            String,
            "/om6dof/operation_mode/state",
            self._on_control_mode,
            grip_qos,
        )
        # Battery from /lowstate (~500 Hz). The callback only stores the latest
        # values — no processing — so it stays cheap.
        self.battery_soc: Optional[int] = None
        self.battery_v: Optional[float] = None
        self.battery_a: Optional[float] = None
        self.create_subscription(LowState, "/lowstate", self._on_lowstate, qos)

    def _on_gripper(self, msg: String) -> None:
        self.gripper_state = msg.data

    def _on_remote_enabled(self, msg: Bool) -> None:
        self.remote_enabled = bool(msg.data)

    def _on_control_mode(self, msg: String) -> None:
        self.control_mode = msg.data.strip().upper()

    def _on_lowstate(self, msg: LowState) -> None:
        try:
            self.battery_soc = int(msg.bms_state.soc)
            self.battery_v = float(msg.power_v)
            self.battery_a = float(msg.power_a)
        except Exception:
            pass

    def nodes(self) -> List[Tuple[str, str]]:
        """(name, namespace) tuples, unsorted — duplicates preserved."""
        return list(self.get_node_names_and_namespaces())

    def topics(self) -> List[Tuple[str, List[str]]]:
        return sorted(self.get_topic_names_and_types(), key=lambda x: x[0])

    def teleop_running(self) -> bool:
        return self.remote_enabled is True

    def tap_f3(self) -> None:
        """Publish an F3 edge so ros2_control switches arm ownership."""
        m = WirelessController()
        for _ in range(5):
            m.keys = BTN_F3
            self.pub_remote.publish(m)
            time.sleep(0.05)
        m.keys = 0
        self.pub_remote.publish(m)

    def set_operation_mode(self, mode: str) -> None:
        self.pub_operation_mode.publish(String(data=mode.strip().upper()))

    # ----------------------------------------------------------------------- #
    #  Sport-mode motion (chat "Robot Agent" mode)                            #
    # ----------------------------------------------------------------------- #
    def _sport(self, api_id: int, parameter: str = "") -> None:
        if self.pub_sport is None:
            raise RuntimeError(
                "unitree_api not available — cannot drive the robot from chat."
            )
        req = SportRequest()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self.pub_sport.publish(req)

    def sport_stop(self) -> None:
        self._motion_gen += 1  # cancel any running move loop
        self._sport(API_STOP_MOVE)

    def sport_stand(self) -> None:
        self._motion_gen += 1
        self._sport(API_STAND_UP)
        self._sport(API_BALANCE_STAND)

    def sport_gait(self, code: int) -> None:
        """Switch locomotion mode (api_id 1011). Sent a few times because the sport
        controller can drop a single SwitchGait (same repeat trick as the demo and
        go2w_cmd_vel_control)."""
        param = '{"data":%d}' % int(code)
        for _ in range(3):
            self._sport(API_SWITCH_GAIT, param)
            time.sleep(0.1)

    def sport_speed(self, level: int) -> None:
        """Set speed gear via SpeedLevel (api_id 1015, -1/0/1). Per the Unitree
        doc this only takes effect in the default gait, so for any non-normal gear
        we drop back to gait 0 first."""
        level = max(-1, min(1, int(level)))
        if level != 0:
            self._sport(API_SWITCH_GAIT, '{"data":0}')
            time.sleep(0.1)
        for _ in range(3):
            self._sport(API_SPEED_LEVEL, '{"data":%d}' % level)
            time.sleep(0.1)

    def sport_damp(self) -> None:
        self._motion_gen += 1
        self._sport(API_DAMP)

    def sport_lie_down(self) -> None:
        """Lie down. StandDown only responds when the robot is in the standing-lock
        (StandUp) or damping state — from balance-stand or a gait it is ignored. So
        stop, enter StandUp lock, wait for it to settle, then StandDown. Runs in a
        thread because of the settle delay."""
        with self._motion_lock:
            self._motion_gen += 1
            gen = self._motion_gen

        def _run() -> None:
            self._sport(API_STOP_MOVE)
            time.sleep(0.2)
            self._sport(API_STAND_UP)          # required precondition (joint lock)
            for _ in range(5):                 # wait ~0.5 s for stand-up to settle
                if gen != self._motion_gen:    # superseded by a newer command
                    return
                time.sleep(0.1)
            self._sport(API_STAND_DOWN)

        threading.Thread(target=_run, daemon=True).start()

    def sport_recover(self) -> None:
        self._motion_gen += 1
        self._sport(API_RECOVERY_STAND)

    def sport_move(self, vx: float, vy: float, wz: float, duration: float) -> None:
        """Stream Move at 20 Hz for `duration` s (clamped), then StopMove. Runs in
        a daemon thread so the HTTP handler returns immediately."""
        vx = max(-MAX_VX, min(MAX_VX, float(vx)))
        vy = max(-MAX_VY, min(MAX_VY, float(vy)))
        wz = max(-MAX_WZ, min(MAX_WZ, float(wz)))
        duration = max(0.0, min(MAX_DURATION, float(duration)))
        with self._motion_lock:
            self._motion_gen += 1
            gen = self._motion_gen

        def _run() -> None:
            param = '{"x":%.4f,"y":%.4f,"z":%.4f}' % (vx, vy, wz)
            end = time.time() + duration
            try:
                while time.time() < end and gen == self._motion_gen:
                    self._sport(API_MOVE, param)
                    time.sleep(0.05)
            finally:
                if gen == self._motion_gen:  # only stop if not superseded
                    self._sport(API_STOP_MOVE)

        threading.Thread(target=_run, daemon=True).start()


# --------------------------------------------------------------------------- #
#  Camera: RealSense colour → shared latest JPEG (auto start/stop)            #
# --------------------------------------------------------------------------- #
class CameraManager:
    IDLE_RELEASE_S = 8.0

    def __init__(self, width: int, height: int, fps: int) -> None:
        self.width, self.height, self.fps = width, height, fps
        self.lock = threading.Lock()
        self.latest: Optional[bytes] = None
        self.error: Optional[str] = None
        self.running = False
        self.last_access = 0.0
        self._thread: Optional[threading.Thread] = None

    def note_access(self) -> None:
        self.last_access = time.time()
        if not self.running:
            with self.lock:
                if not self.running:
                    self.running = True
                    self.error = None
                    self._thread = threading.Thread(target=self._run, daemon=True)
                    self._thread.start()

    def _run(self) -> None:
        try:
            import numpy as np
            import cv2
            import pyrealsense2 as rs
        except Exception as exc:  # pragma: no cover
            self.error = f"import failed: {exc}"
            self.running = False
            return
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )
        try:
            pipeline.start(cfg)
        except Exception as exc:
            self.error = f"cannot open RealSense: {exc}"
            self.running = False
            return
        try:
            while self.running and (time.time() - self.last_access) < self.IDLE_RELEASE_S:
                frames = pipeline.wait_for_frames(2000)
                color = frames.get_color_frame()
                if not color:
                    continue
                img = np.asanyarray(color.get_data())
                ok, jpg = cv2.imencode(
                    ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70]
                )
                if ok:
                    with self.lock:
                        self.latest = jpg.tobytes()
        except Exception as exc:
            self.error = f"stream error: {exc}"
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            self.running = False
            self.latest = None


# --------------------------------------------------------------------------- #
#  System info helpers (best-effort, never raise)                             #
# --------------------------------------------------------------------------- #
def _read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def sys_info() -> dict:
    info = {}
    info["hostname"] = socket.gethostname()
    try:
        info["ips"] = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=2
        ).stdout.strip()
    except Exception:
        info["ips"] = "?"
    up = _read("/proc/uptime").split()
    if up:
        secs = int(float(up[0]))
        info["uptime"] = f"{secs // 3600}h {(secs % 3600) // 60}m"
    else:
        info["uptime"] = "?"
    try:
        info["load"] = ", ".join(f"{x:.2f}" for x in os.getloadavg())
    except Exception:
        info["load"] = "?"
    mem = {}
    for line in _read("/proc/meminfo").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] in ("MemTotal:", "MemAvailable:"):
            mem[parts[0]] = int(parts[1])
    if "MemTotal:" in mem and "MemAvailable:" in mem:
        used = (mem["MemTotal:"] - mem["MemAvailable:"]) / 1e6
        total = mem["MemTotal:"] / 1e6
        info["ram"] = f"{used:.1f} / {total:.1f} GB"
    else:
        info["ram"] = "?"
    # CPU temp: highest thermal zone reading
    temps = []
    for zone in range(12):
        raw = _read(f"/sys/class/thermal/thermal_zone{zone}/temp").strip()
        if raw.isdigit():
            temps.append(int(raw) / 1000.0)
    info["temp"] = f"{max(temps):.1f} °C" if temps else "?"
    info["domain_id"] = os.environ.get("ROS_DOMAIN_ID", "0 (default)")
    info["arm_bus"] = os.path.exists(ARM_BUS_DEVICE)
    return info


# --------------------------------------------------------------------------- #
#  HTML rendering                                                             #
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
#  AI chat backends: local Ollama, or the codex / claude CLIs                 #
# --------------------------------------------------------------------------- #
OLLAMA_URL = "http://localhost:11434"
CHAT_SYSTEM = ("You are a concise assistant embedded in a Go2W quadruped "
               "robot's web dashboard. Answer briefly.")


def list_ollama_models() -> List[str]:
    try:
        with urllib.request.urlopen(OLLAMA_URL + "/api/tags", timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
        return [m["name"] for m in data.get("models", [])]
    except Exception:
        return []


def model_param_billions(name: str) -> float:
    """Parse the parameter count in billions from a model name ('qwen2.5:7b'→7.0,
    'llama3.2:3b'→3.0). Used to order/choose models smallest-first (smaller = more
    likely to fit the Orin GPU). Unknown → large, so it sorts last."""
    m = re.search(r"(\d+(?:\.\d+)?)\s*b\b", name.lower())
    return float(m.group(1)) if m else 999.0


def list_running_ollama() -> List[str]:
    """Models Ollama currently holds in memory (like `ollama ps`). These are the
    ones consuming GPU/RAM and worth killing."""
    try:
        with urllib.request.urlopen(OLLAMA_URL + "/api/ps", timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
        return [m["name"] for m in data.get("models", [])]
    except Exception:
        return []


def stop_ollama(model: str) -> Tuple[bool, str]:
    """Unload a loaded model from memory (frees GPU/RAM) via keep_alive=0 — the
    documented Ollama way to stop a running model."""
    payload = json.dumps({"model": model, "keep_alive": 0}).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            r.read()
        return True, model
    except Exception as exc:
        return False, f"{model}: {exc}"


def preload_ollama(model: str) -> Tuple[bool, str]:
    """Load a model into memory without generating anything (Ollama loads on an
    empty prompt), so the first real chat is fast. keep_alive keeps it resident."""
    payload = json.dumps({"model": model, "keep_alive": "30m"}).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            r.read()
        return True, model
    except Exception as exc:
        return False, f"{model}: {exc}"


def raw_ollama_model(value: str) -> Optional[str]:
    """Turn a dropdown value ('agent:qwen2.5:7b' / 'ollama:llama3.2:3b') into the
    bare Ollama model name. Returns None for non-Ollama backends (codex/claude)."""
    if ":" not in value:
        return None
    prefix, rest = value.split(":", 1)
    return rest if prefix in ("agent", "ollama") else None


def stop_running_ollama() -> str:
    """Unload every currently-loaded model. Returns a human-readable result."""
    running = list_running_ollama()
    if not running:
        return "No LLM is loaded — nothing to kill."
    ok, failed = [], []
    for m in running:
        good, info = stop_ollama(m)
        (ok if good else failed).append(info)
    parts = []
    if ok:
        parts.append(f"Killed: {', '.join(ok)}")
    if failed:
        parts.append(f"failed: {', '.join(failed)}")
    return " · ".join(parts)


def chat_ollama(model: str, message: str) -> Tuple[Optional[str], Optional[str]]:
    payload = json.dumps({
        "model": model, "prompt": message,
        "system": CHAT_SYSTEM, "stream": False,
        # Keep the KV cache small so the model fits in the Orin's shared GPU
        # memory (default 32k context OOMs on this rig).
        "options": {"num_ctx": 2048, "num_predict": 256},
    }).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            data = json.loads(r.read().decode("utf-8"))
        return (data.get("response", "").strip() or "(empty response)"), None
    except Exception as exc:
        return None, f"ollama error: {exc}"


def chat_cli(argv: List[str], message: str) -> Tuple[Optional[str], Optional[str]]:
    """Run an LLM CLI (codex / claude) with the message as a trailing arg.
    No shell — message is passed as a single argv element."""
    exe = argv[0]
    if shutil.which(exe) is None:
        return None, (f"`{exe}` CLI is not installed on this machine. "
                      f"Install it, then this option will work.")
    try:
        p = subprocess.run(
            argv + [message], capture_output=True, text=True, timeout=300,
        )
    except subprocess.TimeoutExpired:
        return None, f"{exe} timed out (300 s)."
    except Exception as exc:
        return None, f"{exe} failed: {exc}"
    out = (p.stdout or "").strip()
    if not out:
        out = (p.stderr or "").strip() or f"({exe} returned no output, rc={p.returncode})"
    return out, None


def route_chat(model: str, message: str) -> Tuple[Optional[str], Optional[str]]:
    if model.startswith("ollama:"):
        return chat_ollama(model.split(":", 1)[1], message)
    if model == "codex":
        # `codex exec <prompt>` = non-interactive one-shot (adjust if your CLI differs)
        return chat_cli(["codex", "exec"], message)
    if model == "claude":
        # `claude -p <prompt>` = headless print mode
        return chat_cli(["claude", "-p"], message)
    return None, f"unknown model '{model}'"


# --------------------------------------------------------------------------- #
#  Robot Agent: local LLM → JSON action → robot motion                        #
# --------------------------------------------------------------------------- #
# Minimal fallback used only if the skill .md can't be found on disk. The
# authoritative, editable version lives in skills/go2w_control_skill.md.
_FALLBACK_SKILL = (
    "You control a Unitree Go2W robot. Reply with ONE JSON object only, no prose. "
    'Schema: {"action":"move|stop|stand|teleop|say", "vx":f,"vy":f,"wz":f,'
    '"duration":f, "enable":bool, "reply":"short confirmation"}. '
    "move: vx forward+/back- (<=0.4 m/s), vy left+/right- (<=0.3), wz left+/right- "
    "(<=0.8 rad/s), duration seconds (<=8). Forward distance: duration=metres/0.3. "
    "Turn: duration=radians/0.6 (90deg=1.57rad). Reply in the user's language."
)


def load_skill() -> str:
    """Return the Robot Agent system prompt. Prefer the installed skill .md
    (share/om6dof_teleop/skills/), extracting the text between the <<<SKILL and
    SKILL>>> markers; fall back to the embedded minimal prompt."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory("om6dof_teleop"),
            "skills", "go2w_control_skill.md",
        )
        with open(path, encoding="utf-8") as f:
            text = f.read()
        # Match the markers only where they stand alone on their own line, so a
        # prose mention of "<<<SKILL"/"SKILL>>>" elsewhere in the doc is ignored.
        marker_a, marker_b = "<<<SKILL\n", "\nSKILL>>>"
        start = text.find(marker_a)
        end = text.find(marker_b)
        if start != -1 and end != -1:
            return text[start + len(marker_a):end].strip()
    except Exception:
        pass
    return _FALLBACK_SKILL


def extract_action(reply: str) -> Optional[dict]:
    """Pull the first JSON object out of an LLM reply (tolerates code fences and
    stray prose around it). Returns the parsed dict, or None if none/invalid."""
    if not reply:
        return None
    depth = 0
    start = -1
    for i, ch in enumerate(reply):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start != -1:
                try:
                    obj = json.loads(reply[start:i + 1])
                    if isinstance(obj, dict) and "action" in obj:
                        return obj
                except Exception:
                    start = -1  # not valid JSON; keep scanning
    return None


def execute_action(node: "MonitorNode", act: dict) -> Tuple[Optional[str], Optional[str]]:
    """Perform the parsed action on the robot. Returns (reply, error)."""
    action = str(act.get("action", "")).lower()
    reply = str(act.get("reply", "")).strip()
    try:
        if action == "move":
            node.sport_move(
                act.get("vx", 0.0), act.get("vy", 0.0),
                act.get("wz", 0.0), act.get("duration", 1.5),
            )
        elif action == "stop":
            node.sport_stop()
        elif action == "stand":
            node.sport_stand()
        elif action == "gait":
            mode = str(act.get("mode", "")).lower()
            code = act.get("code", GAIT_CODES.get(mode))
            if code is None:
                return None, (f"unknown gait mode '{mode}'. "
                              f"Use one of: {', '.join(GAIT_CODES)}.")
            node.sport_gait(int(code))
            return reply or f"Switched to {mode or code} mode.", None
        elif action == "speed":
            level_name = str(act.get("level", "")).lower()
            level = act.get("value", SPEED_LEVELS.get(level_name))
            if level is None:
                return None, (f"unknown speed '{level_name}'. "
                              f"Use one of: {', '.join(SPEED_LEVELS)}.")
            node.sport_speed(int(level))
            return reply or f"Speed set to {level_name or level}.", None
        elif action == "damp":
            node.sport_damp()
        elif action == "lie_down":
            node.sport_lie_down()
        elif action == "recover":
            node.sport_recover()
        elif action == "teleop":
            running = node.teleop_running()
            want = bool(act.get("enable", True))
            if want != running:
                node.tap_f3()
            else:
                reply = reply or (
                    "Teleop is already running." if running else "Teleop is already off."
                )
        elif action == "list_topics":
            topics = node.topics()
            lines = "\n".join(f"• {n}  ({', '.join(t)})" for n, t in topics)
            return f"{len(topics)} ROS topics:\n{lines}", None
        elif action == "list_nodes":
            names = sorted(
                (f"{ns.rstrip('/')}/{n}" if ns not in ("", "/") else n)
                for n, ns in node.nodes()
            )
            lines = "\n".join(f"• {n}" for n in names)
            return f"{len(names)} ROS nodes:\n{lines}", None
        elif action == "battery":
            if node.battery_soc is None:
                return "Battery data not available yet (needs /lowstate).", None
            extra = []
            if node.battery_v is not None:
                extra.append(f"{node.battery_v:.1f} V")
            if node.battery_a is not None:
                extra.append(f"{node.battery_a:+.1f} A")
            tail = f" ({', '.join(extra)})" if extra else ""
            return f"Battery {node.battery_soc}%{tail}.", None
        elif action == "status":
            info = sys_info()
            soc = f"{node.battery_soc}%" if node.battery_soc is not None else "?"
            teleop = "on" if node.teleop_running() else "off"
            grip = node.gripper_state or "?"
            return (f"Status: battery {soc}, CPU temp {info['temp']}, "
                    f"RAM {info['ram']}, teleop {teleop}, gripper {grip}."), None
        elif action == "say":
            pass
        else:
            return None, f"unknown action: '{action}'"
    except Exception as exc:
        return None, f"failed to run '{action}': {exc}"
    return reply or f"OK ({action}).", None


def _agent_generate(model: str, message: str
                    ) -> Tuple[Optional[str], Optional[str]]:
    """One skill-driven generate call. Returns (raw_reply, error)."""
    payload = json.dumps({
        "model": model, "prompt": message,
        "system": load_skill(), "stream": False,
        "options": {"num_ctx": 2048, "num_predict": 200, "temperature": 0.1},
    }).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_URL + "/api/generate", data=payload,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            data = json.loads(r.read().decode("utf-8"))
        return (data.get("response", "") or "").strip(), None
    except urllib.error.HTTPError as exc:
        # 500 here is almost always the model failing to load — on this Orin that
        # means the CUDA/GPU allocation OOM'd (big models like 7B don't fit).
        hint = " (likely out of GPU memory — pick a smaller model)" if exc.code == 500 else ""
        return None, f"{model}: HTTP {exc.code}{hint}"
    except Exception as exc:
        return None, f"{model}: {exc}"


def pick_fallback_model(exclude: str) -> Optional[str]:
    """Smallest installed Ollama model that isn't the one that just failed —
    used to auto-recover when the chosen model OOMs. Size parsed from the name."""
    models = [m for m in list_ollama_models() if m != exclude]
    return sorted(models, key=model_param_billions)[0] if models else None


def route_agent(node: "MonitorNode", ollama_model: str, message: str
                ) -> Tuple[Optional[str], Optional[str]]:
    """Chat message → local LLM (with the robot skill) → JSON action → execute.
    If the chosen model errors (typically a 7B OOMing the Orin GPU), fall back to
    the smallest other installed model and tell the user."""
    if node.pub_sport is None:
        return None, ("Motion control unavailable (unitree_api package is not "
                      "imported in this node).")
    raw, err = _agent_generate(ollama_model, message)
    note = ""
    if err:
        fb = pick_fallback_model(exclude=ollama_model)
        if not fb:
            return None, f"{err}. No fallback model installed."
        raw, err2 = _agent_generate(fb, message)
        if err2:
            return None, f"{err}; fallback {err2}."
        note = f"[{ollama_model} unavailable → used {fb}] "
    act = extract_action(raw)
    if act is None:
        return None, (f"LLM did not return a valid JSON action. "
                      f"Raw reply: {raw[:200]}")
    reply, aerr = execute_action(node, act)
    if aerr:
        return None, aerr
    return (note + (reply or "")), None


CSS = """
:root{color-scheme:dark}
body{font-family:system-ui,sans-serif;margin:0;background:#12141a;color:#e6e6e6}
header{background:#1c1f27;padding:14px 20px;display:flex;align-items:center;
  justify-content:space-between;border-bottom:1px solid #2a2e39}
h1{font-size:18px;margin:0}
main{padding:16px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:800px){.grid{grid-template-columns:1fr}}
.card{background:#1c1f27;border:1px solid #2a2e39;border-radius:10px;padding:14px;
  margin-bottom:16px}
.card h2{font-size:14px;margin:0 0 10px;color:#9aa4b2;text-transform:uppercase;
  letter-spacing:.05em}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:4px 6px;border-bottom:1px solid #23262f;vertical-align:top}
td.k{color:#9aa4b2;width:42%}
.pill{display:inline-block;padding:2px 9px;border-radius:20px;font-size:12px;
  font-weight:600}
.ok{background:#123d2b;color:#4ade80}
.bad{background:#3d1620;color:#f87171}
.warn{background:#3d3416;color:#fbbf24}
.mono{font-family:ui-monospace,Menlo,monospace;font-size:12px}
.dup{background:#3d1620;color:#f87171;font-weight:700}
button,.btn{background:#2b64f5;color:#fff;border:0;padding:9px 16px;border-radius:8px;
  font-size:14px;cursor:pointer;text-decoration:none;display:inline-block}
button.stop{background:#c0392b}
.btn.ghost{background:#2a2e39}
img.cam{width:100%;border-radius:8px;background:#000;min-height:220px}
.small{color:#6b7280;font-size:12px}
ul.nodes{list-style:none;padding:0;margin:0;font-size:13px}
ul.nodes li{padding:3px 6px;border-bottom:1px solid #23262f;display:flex;
  align-items:center;justify-content:space-between;gap:8px}
button.kill{background:#7a2531;color:#f3b0b7;border:0;padding:2px 10px;
  border-radius:6px;font-size:11px;cursor:pointer}
button.kill:hover{background:#c0392b;color:#fff}
.flash{background:#123d2b;color:#7ee2a8;border:1px solid #1c5c40;
  border-radius:8px;padding:10px 14px;margin-bottom:14px;font-size:13px}
.inline{display:inline;margin:0}
.chatcard{border-color:#2b64f5;box-shadow:0 0 0 1px #2b64f533}
.chatcard h2{color:#7ea1ff}
.btnrow{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:6px}
.chatlog{min-height:180px;max-height:44vh;overflow-y:auto;display:flex;
  flex-direction:column;gap:8px;padding:6px 2px;margin-bottom:10px}
.msg.hint{background:transparent;color:#8b93a2;font-size:12px;max-width:100%;
  align-self:stretch;padding-left:0}
.msg{padding:8px 11px;border-radius:10px;font-size:13px;max-width:88%;white-space:pre-wrap;
  word-wrap:break-word;overflow-wrap:anywhere;line-height:1.45}
.msg.user{align-self:flex-end;background:#2b64f5;color:#fff;border-bottom-right-radius:3px}
.msg.ai{align-self:flex-start;background:#2a2e39;color:#e6e6e6;border-bottom-left-radius:3px}
.chatrow{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.chatrow input{flex:1;min-width:150px;background:#12141a;border:1px solid #2a2e39;
  color:#e6e6e6;padding:9px 11px;border-radius:8px;font-size:14px}
.chatrow select{background:#12141a;border:1px solid #2a2e39;color:#e6e6e6;
  padding:9px;border-radius:8px;font-size:13px}
"""


# Client JS kept as a plain (non-f) string so its many { } braces need no
# escaping; interpolated into the page template as {SCRIPTS}.
SCRIPTS = """
<script>
async function pollStatus(){
  try{
    const r = await fetch('/status.json',{cache:'no-store'});
    if(!r.ok) return;
    const d = await r.json();
    for(const k in d){ const el=document.getElementById('st_'+k); if(el) el.innerHTML=d[k]; }
    const dot=document.getElementById('st_dot');
    if(dot){ dot.style.color='#4ade80'; setTimeout(()=>{dot.style.color='#6b7280';},400); }
  }catch(e){}
}
setInterval(pollStatus,3000); pollStatus();

function _log(){ return document.getElementById('chatlog'); }
function addMsg(who,text){
  const d=document.createElement('div'); d.className='msg '+who; d.textContent=text;
  _log().appendChild(d); _log().scrollTop=_log().scrollHeight; return d;
}
async function sendChat(){
  const inp=document.getElementById('chatinput');
  const msg=inp.value.trim(); if(!msg) return;
  const model=document.getElementById('chatmodel').value;
  addMsg('user',msg); inp.value='';
  const t=addMsg('ai','…'); t.style.opacity='0.6';
  try{
    const r=await fetch('/chat',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({model:model,message:msg})});
    const d=await r.json();
    t.style.opacity='1'; t.textContent = d.error ? ('⚠ '+d.error) : d.reply;
  }catch(e){ t.style.opacity='1'; t.textContent='⚠ '+e; }
  _log().scrollTop=_log().scrollHeight;
}
// Preload (warm up) the selected Ollama model so the first chat is fast. Fired
// when the user first focuses the input or switches model. Keyed per model so we
// only load each once.
const _preloaded={};
async function preloadModel(){
  const sel=document.getElementById('chatmodel'); if(!sel) return;
  const model=sel.value;
  if(!model || _preloaded[model]) return;
  if(!(model.startsWith('agent:')||model.startsWith('ollama:'))) return; // skip codex/claude
  _preloaded[model]=true;
  const s=document.getElementById('chatstatus');
  if(s) s.textContent='⏳ loading model into memory…';
  try{
    const r=await fetch('/preload_llm',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({model:model})});
    const d=await r.json();
    if(s) s.textContent = d.ok ? '✓ model ready' : ('⚠ '+(d.model||'load failed'));
    if(!d.ok) _preloaded[model]=false;  // allow a retry
  }catch(e){ _preloaded[model]=false; if(s) s.textContent=''; }
  if(s) setTimeout(()=>{ if(s.textContent==='✓ model ready') s.textContent=''; },2500);
}
document.addEventListener('DOMContentLoaded',function(){
  const b=document.getElementById('chatsend'); if(b) b.addEventListener('click',sendChat);
  const i=document.getElementById('chatinput');
  if(i){
    i.addEventListener('keydown',function(e){ if(e.key==='Enter') sendChat(); });
    i.addEventListener('focus',preloadModel,{once:true});  // warm up on first chat
  }
  const m=document.getElementById('chatmodel');
  if(m) m.addEventListener('change',preloadModel);         // and when model switches
});
</script>
"""


def pill(ok: bool, yes: str, no: str) -> str:
    cls = "ok" if ok else "bad"
    return f'<span class="pill {cls}">{yes if ok else no}</span>'


def gripper_pill(state: Optional[str]) -> str:
    """Colour the gripper state string (e.g. 'HOLDING pos=-0.31 cur=95mA')."""
    if not state:
        return '<span class="pill warn">unknown</span>'
    word = state.split()[0].upper()
    cls = {
        "HOLDING": "ok", "OPEN": "ok",
        "DROPPED": "bad",
        "CLOSED": "warn", "CLOSING": "warn", "OPENING": "warn", "MID": "warn",
    }.get(word, "warn")
    return f'<span class="pill {cls}">{html.escape(state)}</span>'


def battery_pill(node) -> str:
    soc = node.battery_soc
    if soc is None:
        return '<span class="pill warn">no data</span>'
    cls = "ok" if soc >= 40 else ("warn" if soc >= 20 else "bad")
    volt = f" · {node.battery_v:.1f} V" if node.battery_v is not None else ""
    amp = f" · {node.battery_a:+.1f} A" if node.battery_a is not None else ""
    return f'<span class="pill {cls}">{soc}%{volt}{amp}</span>'


def status_fields(node) -> dict:
    """The live, cheap-to-compute status cells (HTML strings). Used both for the
    initial page render and the /status.json poll. One graph query (node names)
    plus /proc reads and cached battery/gripper — light enough to poll often."""
    info = sys_info()
    counts: dict = {}
    for name, ns in node.nodes():
        key = f"{ns.rstrip('/')}/{name}" if ns not in ("", "/") else name
        counts[key] = counts.get(key, 0) + 1
    dups = sum(1 for c in counts.values() if c > 1)
    teleop_on = node.remote_enabled is True
    running_llm = list_running_ollama()
    if running_llm:
        llm_cell = (f'<span class="pill warn">{html.escape(", ".join(running_llm))}</span>'
                    ' <form class="inline" method="POST" action="/stop_llm">'
                    '<button class="kill" type="submit">kill</button></form>')
    else:
        llm_cell = '<span class="pill ok">none loaded</span>'
    return {
        "uptime": html.escape(info["uptime"]),
        "load": f'<span class="mono">{html.escape(info["load"])}</span>',
        "ram": html.escape(info["ram"]),
        "temp": html.escape(info["temp"]),
        "battery": battery_pill(node),
        "arm_bus": pill(info["arm_bus"], "present", "MISSING"),
        "teleop": pill(teleop_on, "REMOTE ENABLED", "remote disabled"),
        "control_mode": (
            f'<span class="pill ok">{html.escape(node.control_mode)}</span>'
            if node.control_mode else '<span class="pill warn">unknown</span>'
        ),
        "gripper": gripper_pill(node.gripper_state),
        "llm": llm_cell,
        "dups": (f'<span class="pill bad">{dups} FOUND</span>' if dups
                 else '<span class="pill ok">none</span>'),
    }


def render_page(node: MonitorNode, cam: CameraManager) -> str:
    info = sys_info()
    nodes = node.nodes()

    # duplicate detection
    counts: dict = {}
    for name, ns in nodes:
        key = f"{ns.rstrip('/')}/{name}" if ns not in ("", "/") else name
        counts[key] = counts.get(key, 0) + 1
    dups = {k: c for k, c in counts.items() if c > 1}
    teleop_on = node.remote_enabled is True

    # --- status card (dynamic cells carry id="st_*" and are live-updated by
    # the poller script below hitting /status.json; static cells are plain) ---
    fields = status_fields(node)
    rows = [
        ("Hostname", html.escape(info["hostname"]), None),
        ("IP addresses", f'<span class="mono">{html.escape(info["ips"])}</span>', None),
        ("Uptime", fields["uptime"], "uptime"),
        ("Load avg", fields["load"], "load"),
        ("RAM used", fields["ram"], "ram"),
        ("CPU temp", fields["temp"], "temp"),
        ("Battery", fields["battery"], "battery"),
        ("ROS_DOMAIN_ID", html.escape(str(info["domain_id"])), None),
        (f"Arm bus {ARM_BUS_DEVICE}", fields["arm_bus"], "arm_bus"),
        ("Teleop", fields["teleop"], "teleop"),
        ("Arm control mode", fields["control_mode"], "control_mode"),
        ("Gripper", fields["gripper"], "gripper"),
        ("LLM loaded", fields["llm"], "llm"),
        ("Duplicate nodes", fields["dups"], "dups"),
    ]
    status_rows = "".join(
        f'<tr><td class="k">{k}</td>'
        + (f'<td id="st_{key}">{v}</td>' if key else f'<td>{v}</td>')
        + '</tr>'
        for k, v, key in rows
    )

    # --- duplicate warning banner ---
    dup_banner = ""
    if dups:
        items = "".join(
            f"<li class='dup'>{html.escape(k)} ×{c}</li>" for k, c in dups.items()
        )
        dup_banner = (
            '<div class="card"><h2>⚠ Duplicate nodes detected</h2>'
            f'<ul class="nodes">{items}</ul>'
            '<p class="small">Two nodes sharing a name usually means a stray '
            'process — kill the extra one.</p></div>'
        )

    # Node/topic lists are no longer rendered — ask the Robot Agent chat instead
    # ("listkan ros topic" / "listkan rosnode"). We still compute `dups`/`counts`
    # above for the duplicate-node safety banner.

    # --- camera + teleop control (separate cards, arranged in render below) ---
    cam_err = f'<p class="small">{html.escape(cam.error)}</p>' if cam.error else ""
    cam_html = f"""
    <div class="card">
      <h2>📷 Camera (RealSense)</h2>
      <img class="cam" src="/camera.mjpg" alt="camera stream loading…">
      {cam_err}
      <p class="small">Live stream; released automatically when unwatched.</p>
    </div>
    """
    teleop_html = f"""
    <div class="card">
      <h2>🎮 Remote control</h2>
      <p>Status: {pill(teleop_on, "REMOTE ENABLED", "remote disabled")}</p>
      <div class="btnrow">
        <form class="inline" method="POST" action="/start_teleop">
          <button type="submit">▶ Enable remote</button>
        </form>
        <form class="inline" method="POST" action="/stop_teleop">
          <button class="stop" type="submit">■ Disable remote</button>
        </form>
      </div>
      <div class="btnrow">
        <form class="inline" method="POST" action="/mode_joint">
          <button type="submit">JOINT</button>
        </form>
        <form class="inline" method="POST" action="/mode_cartesian">
          <button type="submit">CARTESIAN</button>
        </form>
        <form class="inline" method="POST" action="/mode_cylindrical">
          <button type="submit">CYLINDRICAL</button>
        </form>
      </div>
      <p class="small">Sends a momentary F3 to /wirelesscontroller. Remote ON
      activates the forward position controller and ramps the arm to READY in
      JOINT mode; Remote OFF restores the trajectory controller for autonomous
      MoveIt execution. Select on the remote cycles JOINT → CARTESIAN →
      CYLINDRICAL.</p>
    </div>
    """

    # --- AI chat card (model selector = local Ollama models + codex + claude) ---
    # Smallest models first so the default selection is the one most likely to
    # fit the Orin GPU (7B models OOM here; 3B fits).
    ollama_models = sorted(list_ollama_models(), key=model_param_billions)
    # Robot Agent options first (they DRIVE the robot); one per local model.
    agent_opts = "".join(
        f'<option value="agent:{html.escape(m)}">🦿 Robot Agent · {html.escape(m)}</option>'
        for m in ollama_models
    )
    if ollama_models:
        opts = agent_opts + "".join(
            f'<option value="ollama:{html.escape(m)}">Chat · {html.escape(m)}</option>'
            for m in ollama_models
        )
    else:
        opts = '<option value="" disabled>(no local Ollama models)</option>'
    opts += ('<option value="codex">Codex CLI</option>'
             '<option value="claude">Claude Code CLI</option>')
    chat_html = f"""
    <div class="card chatcard">
      <h2>🦿 Robot Agent — command via chat</h2>
      <div class="chatlog" id="chatlog">
        <div class="msg ai hint">Type a command, e.g. <b>move forward 1 meter</b>,
        <b>turn left</b>, <b>stop</b>, <b>battery level</b>,
        <b>list ros topics</b>.</div>
      </div>
      <div class="chatrow">
        <select id="chatmodel" title="Choose the AI backend">{opts}</select>
        <input id="chatinput" type="text" placeholder="Robot command… e.g. 'move forward 1 meter'" autocomplete="off">
        <button id="chatsend" type="button">Send</button>
        <form class="inline" method="POST" action="/stop_llm"
              title="Unload the LLM currently held in GPU/RAM">
          <button class="stop" type="submit">⛔ Kill LLM</button>
        </form>
      </div>
      <p class="small" id="chatstatus"></p>
      <p class="small">🦿 <b>Robot Agent</b> = the local LLM turns your message into a
      real action (move, teleop, ask battery/topics/nodes). <b>Chat</b> = plain Q&amp;A.
      <b>Kill LLM</b> frees GPU/RAM by unloading whichever model is loaded (see the
      "LLM loaded" row below). Skill: skills/go2w_control_skill.md.</p>
    </div>
    """

    # one-shot flash banner (kill result etc.)
    flash_html = ""
    if node.flash:
        flash_html = f'<div class="flash">{html.escape(node.flash)}</div>'
        node.flash = ""

    now = time.strftime("%Y-%m-%d %H:%M:%S")
    return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2W Monitor — {html.escape(info['hostname'])}</title>
<style>{CSS}</style></head><body>
<header><h1>🤖 Go2W Robot Monitor</h1>
<div style="display:flex;gap:8px">
<a class="btn ghost" href="/">↻ Refresh</a>
<form class="inline" method="POST" action="/restart"
      onsubmit="return confirm('Restart the web monitor service? The page will reconnect in a few seconds.')">
  <button class="stop" type="submit">♻ Restart service</button>
</form>
<form class="inline" method="POST" action="/shutdown"
      onsubmit="return confirm('Matikan web monitor & lepas kamera? Monitor TIDAK akan hidup lagi sampai dijalankan manual (systemctl start).')">
  <button class="kill" type="submit">🛑 Kill (lepas kamera)</button>
</form>
</div></header>
<main>
<p class="small">Battery/temperature status refreshes automatically every 3 s.
Node &amp; topic lists are now available via chat. Snapshot {now}.</p>
{flash_html}
{dup_banner}
{chat_html}
<div class="grid">
  <div>
    <div class="card"><h2>📊 Robot status <span class="small" id="st_dot">●</span></h2>
      <table>{status_rows}</table></div>
    {teleop_html}
  </div>
  <div>
    {cam_html}
  </div>
</div>
</main>
{SCRIPTS}
</body></html>"""


# --------------------------------------------------------------------------- #
#  HTTP handler                                                               #
# --------------------------------------------------------------------------- #
def make_handler(node: MonitorNode, cam: CameraManager):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):  # silence per-request logging
            pass

        def _send_html(self, body: str, code: int = 200):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _send_json(self, obj, code: int = 200):
            data = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/camera.mjpg"):
                return self._stream_camera()
            if self.path.startswith("/status.json"):
                try:
                    return self._send_json(status_fields(node))
                except Exception as exc:
                    return self._send_json({"error": str(exc)}, 500)
            if self.path in ("/", "/index.html"):
                try:
                    return self._send_html(render_page(node, cam))
                except Exception as exc:
                    return self._send_html(
                        f"<pre>render error: {html.escape(str(exc))}</pre>", 500
                    )
            self.send_error(404)

        def _read_form(self) -> dict:
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length).decode("utf-8") if length else ""
            return {k: v[0] for k, v in parse_qs(body).items()}

        def _redirect_home(self):
            self.send_response(303)
            self.send_header("Location", "/")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_POST(self):
            if self.path in (
                "/mode_joint", "/mode_cartesian", "/mode_cylindrical"
            ):
                mode = self.path.removeprefix("/mode_").upper()
                node.set_operation_mode(mode)
                node.flash = f"Operation mode request sent: {mode}."
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path in ("/start_teleop", "/stop_teleop", "/toggle_teleop"):
                want_start = self.path == "/start_teleop"
                want_stop = self.path == "/stop_teleop"
                running = node.teleop_running()
                try:
                    if want_start and running:
                        node.flash = "Remote arm ownership is already enabled."
                    elif want_stop and not running:
                        node.flash = "Remote arm ownership is already disabled."
                    else:
                        node.tap_f3()
                        node.flash = (
                            "F3 sent → requesting remote arm ownership."
                            if (want_start or (not running and not want_stop))
                            else "F3 sent → returning arm ownership to autonomy."
                        )
                except Exception as exc:
                    node.flash = f"teleop control failed: {exc}"
                node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path == "/chat":
                try:
                    length = int(self.headers.get("Content-Length", 0) or 0)
                    body = self.rfile.read(length).decode("utf-8") if length else "{}"
                    data = json.loads(body)
                    model = str(data.get("model", "")).strip()
                    message = str(data.get("message", "")).strip()
                except Exception as exc:
                    return self._send_json({"error": f"bad request: {exc}"}, 400)
                if not message:
                    return self._send_json({"error": "empty message"}, 400)
                if model.startswith("agent:"):
                    reply, err = route_agent(node, model.split(":", 1)[1], message)
                else:
                    reply, err = route_chat(model, message)
                return self._send_json({"reply": reply or "", "error": err})
            if self.path == "/kill_node":
                name = self._read_form().get("name", "").strip()
                if name:
                    try:
                        node.flash = kill_node(name)
                    except Exception as exc:
                        node.flash = f"kill '{name}' failed: {exc}"
                    node.get_logger().info(node.flash)
                return self._redirect_home()
            if self.path == "/restart":
                node.get_logger().info("Restart requested from web — re-execing.")
                body = (
                    "<!doctype html><meta charset='utf-8'>"
                    "<meta http-equiv='refresh' content='6; url=/'>"
                    "<title>Restarting…</title>"
                    "<body style='font-family:system-ui;background:#12141a;color:#e6e6e6;"
                    "padding:40px;text-align:center'>"
                    "<h2>♻ Restarting the web monitor…</h2>"
                    "<p>Reloading the latest code. This page reconnects automatically "
                    "in a few seconds.</p>"
                    "<p><a style='color:#7ea1ff' href='/'>Click here</a> if it doesn't.</p>"
                    "</body>"
                )
                self._send_html(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                restart_self()
                return
            if self.path == "/shutdown":
                node.get_logger().info(
                    "Shutdown requested from web — releasing camera & exiting.")
                body = (
                    "<!doctype html><meta charset='utf-8'>"
                    "<title>Shutting down…</title>"
                    "<body style='font-family:system-ui;background:#12141a;"
                    "color:#e6e6e6;padding:40px;text-align:center'>"
                    "<h2>🛑 Web monitor dimatikan</h2>"
                    "<p>Kamera dilepas. Monitor tidak akan hidup lagi sampai "
                    "dijalankan manual:</p>"
                    "<pre style='color:#7ea1ff'>sudo systemctl start "
                    "go2w-web-monitor.service</pre></body>"
                )
                self._send_html(body)
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                shutdown_self(cam)
                return
            if self.path == "/preload_llm":
                try:
                    length = int(self.headers.get("Content-Length", 0) or 0)
                    body = self.rfile.read(length).decode("utf-8") if length else "{}"
                    value = str(json.loads(body).get("model", "")).strip()
                except Exception as exc:
                    return self._send_json({"error": f"bad request: {exc}"}, 400)
                model = raw_ollama_model(value)
                if not model:
                    return self._send_json({"ok": False, "skipped": True})
                ok, info = preload_ollama(model)
                return self._send_json({"ok": ok, "model": info})
            if self.path == "/stop_llm":
                # An optional `model` form field kills just that one; otherwise
                # unload every currently-loaded model.
                model = self._read_form().get("model", "").strip()
                try:
                    if model:
                        ok, info = stop_ollama(model)
                        node.flash = (f"Killed LLM {info}." if ok
                                      else f"Failed to kill LLM {info}.")
                    else:
                        node.flash = stop_running_ollama()
                except Exception as exc:
                    node.flash = f"stop LLM failed: {exc}"
                node.get_logger().info(node.flash)
                return self._redirect_home()
            self.send_error(404)

        def _stream_camera(self):
            cam.note_access()
            # wait briefly for the first frame
            deadline = time.time() + 3.0
            while cam.latest is None and cam.error is None and time.time() < deadline:
                time.sleep(0.05)
            if cam.latest is None:
                msg = cam.error or "camera has no frames"
                return self._send_html(f"<pre>{html.escape(msg)}</pre>", 503)
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                interval = 1.0 / max(1, cam.fps)
                while True:
                    cam.note_access()
                    frame = cam.latest
                    if frame is None:
                        if cam.error:
                            break
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


# --------------------------------------------------------------------------- #
#  main                                                                        #
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    node.declare_parameter("port", 8080)
    node.declare_parameter("camera_width", 640)
    node.declare_parameter("camera_height", 480)
    node.declare_parameter("camera_fps", 15)
    port = int(node.get_parameter("port").value)
    cam = CameraManager(
        int(node.get_parameter("camera_width").value),
        int(node.get_parameter("camera_height").value),
        int(node.get_parameter("camera_fps").value),
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # give discovery a moment to populate the graph before first request
    time.sleep(1.5)

    httpd = ThreadingHTTPServer(("0.0.0.0", port), make_handler(node, cam))
    ip = subprocess.run(
        ["hostname", "-I"], capture_output=True, text=True
    ).stdout.split()
    url = f"http://{ip[0]}:{port}" if ip else f"http://<robot-ip>:{port}"
    node.get_logger().info(f"Go2W web monitor serving at {url}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
