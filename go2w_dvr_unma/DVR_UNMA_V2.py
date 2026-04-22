import cv2, numpy as np, time, os, struct, threading, csv, datetime, signal
import pyrealsense2 as rs
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

# Auto-detect DISPLAY (e.g. when launched from headless shell via NoMachine virtual desktop)
if not os.environ.get("DISPLAY"):
    try:
        for f in sorted(os.listdir("/tmp/.X11-unix/")):
            if f.startswith("X"):
                os.environ["DISPLAY"] = ":" + f[1:]
                break
    except FileNotFoundError:
        pass

# =======================
# CONFIG
# =======================
RS_SERIAL = None
RS_WIDTH  = 640
RS_HEIGHT = 480
RS_FPS    = 30

LOWSTATE_TOPIC = "/lowstate"

# CSV logs go here (one file per kind, all files share the same session_id prefix)
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

DS_W, DS_H    = 64, 48
K             = 5
SMOOTH_ALPHA  = 0.2

# --------------------------------------------------
# DVR Zone boundaries
#   V_LOW  : hunger threshold — regulation starts
#   V_HIGH : satiation threshold — regulation stops
#   V_STAR : regulation target
#   V_INIT : starting viability (start kenyang)
# --------------------------------------------------
V_LOW   = 0.4
V_HIGH  = 0.8
V_STAR  = 0.8
V_INIT  = 0.8

# --------------------------------------------------
# DVR Parameters
#   DECAY_ALPHA : 0.4 unit turun dalam ~60 detik @ 30fps
#   BETA        : 0.4 unit naik dalam ~30 detik @ 30fps (2x safety factor)
# --------------------------------------------------
DECAY_ALPHA = 0.00022
BETA        = 0.001
DELTA_MIN   = 0.05
DELTA_MAX   = 0.8

# --------------------------------------------------
# Proof of concept — robot selalu lapar
#   True  : REGULATING_ALWAYS — cocok untuk manual experiment sekarang
#   False : hunger flag aktif — untuk future autonomous experiment
# --------------------------------------------------
REGULATING_ALWAYS = True

# --------------------------------------------------
# DVR Ablation
#   True  : viability anchor aktif
#   False : tidak ada decay, regulasi, atau anchor (ablation)
# --------------------------------------------------
DVR_ENABLED = False

# --------------------------------------------------
# Food scenario
#   False : pixel stabil = makanan enak  (Scenario 1 — default)
#   True  : pixel stabil = makanan tidak enak (Scenario 2 — inverted)
#           artinya pixel chaos yang menaikkan viability
# --------------------------------------------------
INVERT_FOOD = False

# Tension bucket edges
BUCKET_EDGES = (0.05, 0.15)

# UNMA reification thresholds
MIN_SUPPORT_PAIR = 5
MIN_MEAN_DV_PAIR = 0.001
MAX_STD_DV_PAIR  = 0.003

# Visualization
PLOT_LEN       = 220
PANEL_W        = 620
PANEL_H        = 760
NODE_DV_MAXLEN = 300
PAIR_DV_MAXLEN = 300

DEFAULT_ACTION = "STOP"

# Analog stick mapping:
#   Left stick  -> translation (FORWARD/BACK on ly, LEFT/RIGHT on lx)
#   Right stick -> rotation    (ROT_L/ROT_R on rx)
# The action with the largest axis magnitude wins; all below deadzone = STOP.
STICK_DEADZONE = 0.3

# Only buttons still consumed (for exit).
REMOTE_BUTTON_MAP = {
    0:  "R1",    1:  "L1",    2:  "Start", 3:  "Select",
    4:  "R2",    5:  "L2",    6:  "F1",    7:  "F3",
    8:  "A",     9:  "B",     10: "X",     11: "Y",
    12: "Up",    13: "Right", 14: "Down",  15: "Left",
}
REMOTE_EXIT_BUTTON = "Select"


def decode_remote_buttons(wr):
    key_value = wr[2] | (wr[3] << 8)
    return {name for bit, name in REMOTE_BUTTON_MAP.items() if key_value & (1 << bit)}


def decode_remote_axes(wr):
    wr_bytes = bytes(wr)
    return {
        "lx": struct.unpack_from('<f', wr_bytes, 4)[0],
        "rx": struct.unpack_from('<f', wr_bytes, 8)[0],
        "ry": struct.unpack_from('<f', wr_bytes, 12)[0],
        "ly": struct.unpack_from('<f', wr_bytes, 20)[0],
    }


def action_from_sticks(lx, ly, rx):
    candidates = [
        (abs(ly), "FORWARD" if ly > 0 else "BACK"),
        (abs(lx), "RIGHT"   if lx > 0 else "LEFT"),
        (abs(rx), "ROT_R"   if rx > 0 else "ROT_L"),
    ]
    mag, action = max(candidates, key=lambda x: x[0])
    return action if mag >= STICK_DEADZONE else DEFAULT_ACTION


class RemoteState(Node):
    def __init__(self, topic=LOWSTATE_TOPIC):
        super().__init__("dvr_unma_v2_remote")
        self.pressed = set()
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.last_rx_time = None
        self.create_subscription(LowState, topic, self._cb, 10)

    def _cb(self, msg):
        wr = msg.wireless_remote
        self.pressed = decode_remote_buttons(wr)
        axes = decode_remote_axes(wr)
        self.lx, self.ly = axes["lx"], axes["ly"]
        self.rx, self.ry = axes["rx"], axes["ry"]
        self.last_rx_time = time.time()

# =======================
# HELPERS
# =======================
def tension(a, b):
    return float(np.mean(cv2.absdiff(a, b)) / 255.0)

def bucket_tension(t):
    lo, hi = BUCKET_EDGES
    if t < lo:   return "L"
    elif t < hi: return "M"
    else:        return "H"

def mean_std(dq):
    if not dq: return 0.0, 0.0
    arr = np.array(dq, dtype=float)
    return float(arr.mean()), float(arr.std())

def put(frame, txt, x, y, scale=0.7, th=2):
    cv2.putText(frame, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (255,255,255), th, cv2.LINE_AA)

def draw_timeseries(panel, series, x0, y0, w, h, vmin, vmax, title):
    cv2.rectangle(panel, (x0, y0), (x0+w, y0+h), (80,80,80), 1)
    cv2.putText(panel, title, (x0+10, y0+20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220,220,220), 1, cv2.LINE_AA)
    if len(series) < 2: return
    vals = np.clip(np.array(series, dtype=float), vmin, vmax)
    xs   = np.linspace(x0+10, x0+w-10, len(vals)).astype(int)
    def ymap(val):
        t = (val - vmin) / (vmax - vmin + 1e-9)
        return int(y0 + h - 10 - t*(h-30))
    pts = np.array([(int(xs[i]), ymap(vals[i]))
                    for i in range(len(vals))], dtype=np.int32)
    cv2.polylines(panel, [pts], False, (230,230,230), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{vmin:+.4f}..{vmax:+.4f}", (x0+10, y0+h-8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1, cv2.LINE_AA)


def open_video_source():
    devices = rs.context().query_devices()
    if len(devices) == 0:
        raise RuntimeError("No Intel RealSense device detected.")

    selected_serial = None
    selected_name   = None
    if RS_SERIAL is not None:
        for dev in devices:
            if dev.get_info(rs.camera_info.serial_number) == RS_SERIAL:
                selected_serial = RS_SERIAL
                selected_name   = dev.get_info(rs.camera_info.name)
                break
        if selected_serial is None:
            raise RuntimeError(f"Requested RealSense serial {RS_SERIAL} not found.")
    else:
        dev = devices[0]
        selected_serial = dev.get_info(rs.camera_info.serial_number)
        selected_name   = dev.get_info(rs.camera_info.name)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_device(selected_serial)
    config.enable_stream(rs.stream.color, RS_WIDTH, RS_HEIGHT, rs.format.bgr8, RS_FPS)
    pipeline.start(config)

    profile       = pipeline.get_active_profile()
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr          = color_profile.get_intrinsics()
    print(
        "Video source: Intel RealSense color "
        f"({intr.width}x{intr.height} @ {RS_FPS}fps) "
        f"name={selected_name} serial={selected_serial}"
    )
    return pipeline


def read_frame(pipeline):
    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return False, None
    return True, np.asanyarray(color_frame.get_data())


def close_video_source(pipeline):
    pipeline.stop()

# =======================
# INIT
# =======================
video_source = open_video_source()

rclpy.init()
remote = RemoteState()
spin_thread = threading.Thread(target=rclpy.spin, args=(remote,), daemon=True)
spin_thread.start()

# Catch Ctrl+C: set a flag instead of raising, so the main loop can break cleanly
# and the CSV/summary section still runs. Second Ctrl+C just re-sets the flag.
_interrupted = False
def _sigint_handler(signum, frame):
    global _interrupted
    _interrupted = True
    print("\n[Ctrl+C received — finishing current frame, then saving logs]")
signal.signal(signal.SIGINT, _sigint_handler)

buf    = deque(maxlen=K+1)
t_prev = None
t_ema  = None
v      = V_INIT

# Hunger flag
regulating = REGULATING_ALWAYS

# UNMA structures
node_support = defaultdict(int)
node_dv_hist = defaultdict(lambda: deque(maxlen=NODE_DV_MAXLEN))
edge_support = defaultdict(int)
pair_support = defaultdict(int)
pair_dv_hist = defaultdict(lambda: deque(maxlen=PAIR_DV_MAXLEN))

pending_edge  = None
last_node     = None
active_action = DEFAULT_ACTION
last_print    = time.time()

ts_t  = deque(maxlen=PLOT_LEN)
ts_dv = deque(maxlen=PLOT_LEN)
ts_v  = deque(maxlen=PLOT_LEN)

# Metrics
reification_count    = 0
total_viability_area = 0.0
step_count           = 0

# --------------------------------------------------
# Movement duration tracking
#   action_start_time  : when current action started
#   action_log         : list of completed action segments
#                        each entry = (action, duration_sec, mean_tension, mean_dv)
#   action_tension_buf : tension values collected during current action
#   action_dv_buf      : dv values collected during current action
# --------------------------------------------------
action_start_time  = time.time()
action_log         = []
action_tension_buf = []
action_dv_buf      = []
action_stick_buf   = []  # magnitude of the axis driving the current action (0..1)


def stick_intensity_for(action, lx, ly, rx):
    """Magnitude of the analog axis that drives the given action."""
    if action in ("FORWARD", "BACK"):  return abs(ly)
    if action in ("LEFT", "RIGHT"):    return abs(lx)
    if action in ("ROT_L", "ROT_R"):   return abs(rx)
    return 0.0  # STOP

# Labels
mode_label     = "DVR-ON" if DVR_ENABLED else "DVR-OFF (ablation)"
poc_label      = "POC-ALWAYS-HUNGRY" if REGULATING_ALWAYS else "HUNGER-FLAG-ACTIVE"
scenario_label = "S2-INVERTED" if INVERT_FOOD else "S1-NORMAL"

print(f"UNMA Stage 6 — {mode_label} | {poc_label} | {scenario_label}")
print(f"Remote (via {LOWSTATE_TOPIC}): L-stick=move (WASD), R-stick X=rotate, "
      f"deadzone={STICK_DEADZONE}, {REMOTE_EXIT_BUTTON}=exit")

# Session stem is fixed at start so the realtime timeseries log and the
# final summary CSV share the same filename prefix.
os.makedirs(LOG_DIR, exist_ok=True)
session_id   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
mode_tag     = "DVRon"  if DVR_ENABLED       else "DVRoff"
scen_tag     = "S2inv"  if INVERT_FOOD       else "S1norm"
poc_tag      = "always" if REGULATING_ALWAYS else "hunger"
session_stem = f"{session_id}_{mode_tag}_{scen_tag}_{poc_tag}"

# Realtime per-frame log of the values driving the UI graphs.
# Flushed every frame so data survives a crash or power cut.
ts_csv_path = os.path.join(LOG_DIR, f"{session_stem}_timeseries.csv")
ts_csv_file = open(ts_csv_path, "w", newline="")
ts_csv      = csv.writer(ts_csv_file)
ts_csv.writerow([
    "wall_iso", "wall_s", "step", "action",
    "t_raw", "t_ema", "dv_reg", "v", "u", "food",
    "tension_bucket", "regulating", "reif_count",
    "lx", "ly", "rx", "stick_intensity",
])
ts_csv_file.flush()
t_session_start = time.time()
print(f"Realtime timeseries log: {ts_csv_path}")

# =======================
# MAIN LOOP
# =======================
while True:
    if _interrupted: break

    ret, frame = read_frame(video_source)
    if not ret: break

    # Keep cv2.waitKey so the OpenCV window pumps events
    cv2.waitKey(1)

    # Read remote state
    if REMOTE_EXIT_BUTTON in remote.pressed:
        break

    # Snapshot current stick state (used for both action selection and logging)
    cur_lx, cur_ly, cur_rx = remote.lx, remote.ly, remote.rx

    # Detect action change (from analog sticks) and log completed segment
    new_action = action_from_sticks(cur_lx, cur_ly, cur_rx)
    if new_action != active_action:
        duration    = time.time() - action_start_time
        mean_t      = float(np.mean(action_tension_buf)) if action_tension_buf else 0.0
        std_t       = float(np.std(action_tension_buf))  if action_tension_buf else 0.0
        mean_dv_act = float(np.mean(action_dv_buf))      if action_dv_buf      else 0.0
        mean_stick  = float(np.mean(action_stick_buf))   if action_stick_buf   else 0.0
        max_stick   = float(np.max(action_stick_buf))    if action_stick_buf   else 0.0
        action_log.append({
            "action":       active_action,
            "duration_s":   round(duration, 2),
            "mean_tension": round(mean_t, 6),
            "std_tension":  round(std_t, 6),
            "mean_dv":      round(mean_dv_act, 6),
            "mean_stick":   round(mean_stick, 4),
            "max_stick":    round(max_stick, 4),
        })
        print(f"  [ACTION END] {active_action:8s} dur={duration:.2f}s "
              f"mean_t={mean_t:.4f} std_t={std_t:.4f} mean_dv={mean_dv_act:+.6f} "
              f"mean_stick={mean_stick:.3f} max_stick={max_stick:.3f}")

        active_action      = new_action
        action_start_time  = time.time()
        action_tension_buf = []
        action_dv_buf      = []
        action_stick_buf   = []

    # Preprocess frame
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (DS_W, DS_H), interpolation=cv2.INTER_AREA)
    buf.append(small)

    t_raw = 0.0
    if len(buf) == K + 1:
        t_raw = tension(buf[0], buf[-1])

    if t_ema is None: t_ema = t_raw
    else: t_ema = (1 - SMOOTH_ALPHA)*t_ema + SMOOTH_ALPHA*t_raw

    dv_reg = 0.0 if t_prev is None else (t_prev - t_ema)
    t_prev = t_ema

    # Collect per-action buffers
    action_tension_buf.append(t_ema)
    action_dv_buf.append(dv_reg)
    action_stick_buf.append(stick_intensity_for(active_action, cur_lx, cur_ly, cur_rx))

    # ===========================
    # DVR — zone activation
    # ===========================
    if DVR_ENABLED:
        if not REGULATING_ALWAYS:
            if v <= V_LOW:
                regulating = True
            elif v >= V_HIGH:
                regulating = False

        if regulating:
            delta   = V_STAR - v
            clipped = np.clip(abs(delta), DELTA_MIN, DELTA_MAX) * np.sign(delta)
            u       = BETA * clipped
        else:
            u = 0.0

        decay_term = -DECAY_ALPHA

        # Food signal: normal or inverted depending on scenario
        # Scenario 1: pixel stabil (dv positif) = makanan enak → viability naik
        # Scenario 2: pixel stabil (dv positif) = makanan tidak enak → viability turun
        food_signal  = dv_reg if not INVERT_FOOD else -dv_reg
        effective_dv = food_signal if regulating else 0.0

    else:
        # DVR ablation: no decay, no regulation, no anchor, no food signal
        u            = 0.0
        decay_term   = 0.0
        effective_dv = 0.0
        regulating   = False

    v = v + decay_term + u + effective_dv
    v = float(np.clip(v, 0.0, 1.0))

    # ===========================
    # UNMA — node + edge
    # ===========================
    tb   = bucket_tension(t_ema)
    node = (active_action, tb)

    node_support[node] += 1
    node_dv_hist[node].append(dv_reg)

    if pending_edge is not None:
        edge_support[pending_edge] += 1
        pair_support[pending_edge] += 1
        pair_dv_hist[pending_edge].append(dv_reg)

    # Skip STOP at either endpoint — STOP is not an intentional "food-seeking"
    # action, so edges involving it shouldn't count toward reification.
    if (last_node is not None
            and node != last_node
            and last_node[0] != "STOP"
            and node[0] != "STOP"):
        pending_edge = (last_node, node)
    else:
        pending_edge = None

    last_node = node

    # ===========================
    # Reification candidates
    # ===========================
    meta_pairs = []
    for e, cnt in pair_support.items():
        if cnt < MIN_SUPPORT_PAIR: continue
        mdv, sdv = mean_std(pair_dv_hist[e])
        if mdv >= MIN_MEAN_DV_PAIR and sdv <= MAX_STD_DV_PAIR:
            meta_pairs.append((e, cnt, mdv, sdv))
    meta_pairs.sort(key=lambda x: (x[2], x[1]), reverse=True)

    reification_count    = len(meta_pairs)
    total_viability_area += abs(v - V_STAR)
    step_count           += 1

    # Print every second
    if time.time() - last_print >= 1.0:
        avg_v_dev    = total_viability_area / max(step_count, 1)
        hungry_label = "HUNGRY" if regulating else "FULL"
        cur_dur      = time.time() - action_start_time
        print(f"[{mode_label}][{scenario_label}][{hungry_label}] "
              f"action={active_action}({cur_dur:.1f}s) "
              f"t={t_ema:.4f} tb={tb} dv={dv_reg:+.6f} "
              f"food={effective_dv:+.6f} v={v:+.3f} "
              f"reif={reification_count} avg_|dv|={avg_v_dev:.5f}")
        if meta_pairs:
            for e, cnt, mdv, sdv in meta_pairs[:3]:
                print(f"  REIF: {e[0]} -> {e[1]} cnt={cnt} "
                      f"mean_dv={mdv:+.6f} std={sdv:.6f}")
        last_print = time.time()

    # ===========================
    # VISUAL PANEL
    # ===========================
    ts_t.append(t_ema)
    ts_dv.append(dv_reg)
    ts_v.append(v)

    now = time.time()
    ts_csv.writerow([
        datetime.datetime.now().isoformat(timespec="milliseconds"),
        round(now - t_session_start, 3),
        step_count,
        active_action,
        round(t_raw, 6),
        round(t_ema, 6),
        round(dv_reg, 6),
        round(v, 6),
        round(u, 6),
        round(effective_dv, 6),
        tb,
        int(regulating),
        reification_count,
        round(cur_lx, 4),
        round(cur_ly, 4),
        round(cur_rx, 4),
        round(stick_intensity_for(active_action, cur_lx, cur_ly, cur_rx), 4),
    ])
    ts_csv_file.flush()

    hungry_label = "HUNGRY" if regulating else "FULL"
    cur_dur      = time.time() - action_start_time
    put(frame, f"{mode_label} | {scenario_label} (x=exit)", 10, 25, 0.5, 2)
    put(frame, f"ACTION={active_action} ({cur_dur:.1f}s)  [{hungry_label}]", 10, 55, 0.7, 2)

    panel      = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    mode_color = (100, 255, 100) if DVR_ENABLED else (100, 100, 255)
    scen_color = (0, 200, 255) if INVERT_FOOD else (200, 255, 0)

    cv2.putText(panel, f"{mode_label} | {poc_label}", (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2, cv2.LINE_AA)
    cv2.putText(panel, f"SCENARIO: {scenario_label}", (20, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, scen_color, 2, cv2.LINE_AA)

    hunger_color = (0, 165, 255) if regulating else (160, 160, 160)
    cv2.putText(panel, f"STATE: {hungry_label}", (350, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, hunger_color, 2, cv2.LINE_AA)

    put(panel, f"ACTION: {active_action} ({cur_dur:.1f}s)", 20, 90, 0.65, 1)
    put(panel, f"t_ema={t_ema:.4f}  bucket={tb}  dv={dv_reg:+.6f}", 20, 115, 0.55, 1)
    put(panel, f"v={v:+.3f}  u={u:+.5f}  food={effective_dv:+.6f}", 20, 138, 0.55, 1)
    put(panel, f"reif={reification_count}  nodes={len(node_support)}  edges={len(edge_support)}", 20, 161, 0.55, 1)

    draw_timeseries(panel, list(ts_t),  20, 175, PANEL_W-40, 100, 0.0,   0.15, "t_ema (EMA tension)")
    draw_timeseries(panel, list(ts_dv), 20, 285, PANEL_W-40, 100, -0.03, 0.03, "dv_reg (T_prev - T_now)")
    draw_timeseries(panel, list(ts_v),  20, 395, PANEL_W-40, 100,  0.0,   1.0, "viability v(t)  [V_LOW=0.4  V_HIGH=0.8]")

    # V_LOW and V_HIGH reference lines on viability plot
    vy0, vy1 = 395, 495
    def vmap(val):
        t = (val - 0.0) / (1.0 + 1e-9)
        return int(vy1 - 10 - t*(vy1 - vy0 - 30))
    cv2.line(panel, (20, vmap(V_LOW)),  (PANEL_W-20, vmap(V_LOW)),  (0, 100, 255), 1)
    cv2.line(panel, (20, vmap(V_HIGH)), (PANEL_W-20, vmap(V_HIGH)), (0, 255, 100), 1)
    put(panel, "V_LOW",  PANEL_W-75, vmap(V_LOW)-3,  0.38, 1)
    put(panel, "V_HIGH", PANEL_W-75, vmap(V_HIGH)-3, 0.38, 1)

    # Reification candidates box
    y = 505
    cv2.rectangle(panel, (20, y), (PANEL_W-20, y+120), (80,80,80), 1)
    put(panel, "REIFICATION candidates:", 30, y+22, 0.55, 1)
    if meta_pairs:
        for i, (e, cnt, mdv, sdv) in enumerate(meta_pairs[:4]):
            s = f"{i+1}) {e[0]} -> {e[1]} cnt={cnt} mean_dv={mdv:+.6f}"
            put(panel, s, 30, y+45+i*22, 0.48, 1)
    else:
        put(panel, "(none yet)", 30, y+45, 0.5, 1)

    # Last completed action log (bottom strip)
    y2 = 635
    cv2.rectangle(panel, (20, y2), (PANEL_W-20, PANEL_H-10), (60,60,60), 1)
    put(panel, "LAST ACTION LOG:", 30, y2+20, 0.52, 1)
    if action_log:
        for i, entry in enumerate(action_log[-4:]):
            s = (f"{entry['action']:8s} {entry['duration_s']:.1f}s  "
                 f"mean_t={entry['mean_tension']:.4f}  "
                 f"std_t={entry['std_tension']:.4f}  "
                 f"mean_dv={entry['mean_dv']:+.6f}")
            put(panel, s, 30, y2+42+i*22, 0.45, 1)
    else:
        put(panel, "(no completed actions yet)", 30, y2+42, 0.45, 1)

    h_max = max(frame.shape[0], panel.shape[0])
    def pad_h(img, h):
        if img.shape[0] < h:
            p = np.zeros((h-img.shape[0], img.shape[1], 3), dtype=np.uint8)
            return np.vstack([img, p])
        return img

    combined = np.hstack([pad_h(frame, h_max), pad_h(panel, h_max)])
    cv2.imshow("UNMA_STAGE6", combined)

# =======================
# CLEANUP + SUMMARY
# =======================

# Log the final active action segment before exit
if action_tension_buf:
    duration    = time.time() - action_start_time
    mean_t      = float(np.mean(action_tension_buf))
    std_t       = float(np.std(action_tension_buf))
    mean_dv_act = float(np.mean(action_dv_buf))    if action_dv_buf    else 0.0
    mean_stick  = float(np.mean(action_stick_buf)) if action_stick_buf else 0.0
    max_stick   = float(np.max(action_stick_buf))  if action_stick_buf else 0.0
    action_log.append({
        "action":       active_action,
        "duration_s":   round(duration, 2),
        "mean_tension": round(mean_t, 6),
        "std_tension":  round(std_t, 6),
        "mean_dv":      round(mean_dv_act, 6),
        "mean_stick":   round(mean_stick, 4),
        "max_stick":    round(max_stick, 4),
    })

close_video_source(video_source)
cv2.destroyAllWindows()
remote.destroy_node()
rclpy.try_shutdown()

ts_csv_file.flush()
ts_csv_file.close()
print(f"Realtime timeseries log closed: {ts_csv_path}")

avg_v_deviation = total_viability_area / max(step_count, 1)

# =======================
# CSV LOGS
# (session_stem was already fixed at startup — shared with the realtime
# timeseries log so both files have the same prefix)
# =======================

# Pre-compute aggregates for all sections
action_agg_out = defaultdict(lambda: {"dur": 0.0, "tensions": [], "dvs": [],
                                      "sticks": [], "max_stick": 0.0, "segs": 0})
for e in action_log:
    a = e["action"]
    action_agg_out[a]["dur"]      += e["duration_s"]
    action_agg_out[a]["tensions"].append(e["mean_tension"])
    action_agg_out[a]["dvs"].append(e["mean_dv"])
    action_agg_out[a]["sticks"].append(e["mean_stick"])
    action_agg_out[a]["max_stick"] = max(action_agg_out[a]["max_stick"], e["max_stick"])
    action_agg_out[a]["segs"]     += 1

meta_final = []
for e, cnt in pair_support.items():
    if cnt < MIN_SUPPORT_PAIR:
        continue
    mdv, sdv = mean_std(pair_dv_hist[e])
    if mdv >= MIN_MEAN_DV_PAIR and sdv <= MAX_STD_DV_PAIR:
        meta_final.append((e, cnt, mdv, sdv))
meta_final.sort(key=lambda x: (x[2], x[1]), reverse=True)

# Write everything into a single CSV with section markers
csv_path = os.path.join(LOG_DIR, f"{session_stem}.csv")
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)

    # --- Section 1: Summary ---
    w.writerow(["# SECTION", "summary"])
    w.writerow(["session_id", "mode", "scenario", "poc",
                "total_steps", "total_nodes", "total_edges",
                "reification_count", "avg_abs_v_deviation",
                "dvr_enabled", "invert_food", "regulating_always",
                "v_low", "v_high", "v_star", "v_init",
                "decay_alpha", "beta", "bucket_edges_lo", "bucket_edges_hi",
                "stick_deadzone"])
    w.writerow([session_id, mode_tag, scen_tag, poc_tag,
                step_count, len(node_support), len(edge_support),
                len(meta_final), round(avg_v_deviation, 6),
                int(DVR_ENABLED), int(INVERT_FOOD), int(REGULATING_ALWAYS),
                V_LOW, V_HIGH, V_STAR, V_INIT,
                DECAY_ALPHA, BETA, BUCKET_EDGES[0], BUCKET_EDGES[1],
                STICK_DEADZONE])
    w.writerow([])

    # --- Section 2: Per-segment action log ---
    w.writerow(["# SECTION", "action_log"])
    w.writerow(["seg_idx", "action", "duration_s", "mean_tension", "std_tension",
                "mean_dv", "mean_stick", "max_stick"])
    for i, e in enumerate(action_log):
        w.writerow([i, e["action"], e["duration_s"], e["mean_tension"],
                    e["std_tension"], e["mean_dv"],
                    e["mean_stick"], e["max_stick"]])
    w.writerow([])

    # --- Section 3: Aggregated per-action ---
    w.writerow(["# SECTION", "action_agg"])
    w.writerow(["action", "total_dur_s", "mean_tension", "std_tension", "mean_dv",
                "mean_stick", "max_stick", "segments"])
    for a, agg in sorted(action_agg_out.items(), key=lambda x: x[1]["dur"], reverse=True):
        mt   = float(np.mean(agg["tensions"])) if agg["tensions"] else 0.0
        st   = float(np.std(agg["tensions"]))  if agg["tensions"] else 0.0
        mdv  = float(np.mean(agg["dvs"]))      if agg["dvs"]      else 0.0
        msk  = float(np.mean(agg["sticks"]))   if agg["sticks"]   else 0.0
        w.writerow([a, round(agg["dur"], 2), round(mt, 6), round(st, 6),
                    round(mdv, 6), round(msk, 4), round(agg["max_stick"], 4),
                    agg["segs"]])
    w.writerow([])

    # --- Section 4: UNMA nodes ---
    w.writerow(["# SECTION", "nodes"])
    w.writerow(["action", "tension_bucket", "count", "mean_dv", "std_dv"])
    for (act, tb), cnt in sorted(node_support.items(), key=lambda x: x[1], reverse=True):
        mdv, sdv = mean_std(node_dv_hist[(act, tb)])
        w.writerow([act, tb, cnt, round(mdv, 6), round(sdv, 6)])
    w.writerow([])

    # --- Section 5: Reification pairs ---
    w.writerow(["# SECTION", "reification"])
    w.writerow(["from_action", "from_bucket", "to_action", "to_bucket",
                "count", "mean_dv", "std_dv"])
    for (src, dst), cnt, mdv, sdv in meta_final:
        w.writerow([src[0], src[1], dst[0], dst[1], cnt,
                    round(mdv, 6), round(sdv, 6)])

print(f"\nCSV log saved to {csv_path}")

print(f"\n=== SUMMARY [{mode_label}] | [{poc_label}] | [{scenario_label}] ===")
print(f"Total steps:        {step_count}")
print(f"Final nodes:        {len(node_support)}")
print(f"Final edges:        {len(edge_support)}")
print(f"Reification nodes:  {reification_count}")
print(f"Avg |v - v*|:       {avg_v_deviation:.6f}  (lower = more stable)")

print("\n=== MOVEMENT LOG (all segments) ===")
print(f"  {'ACTION':<10} {'DUR(s)':>7}  {'MEAN_T':>9}  {'STD_T':>9}  "
      f"{'MEAN_DV':>10}  {'MEAN_STK':>8}  {'MAX_STK':>7}")
print(f"  {'-'*10} {'-'*7}  {'-'*9}  {'-'*9}  {'-'*10}  {'-'*8}  {'-'*7}")
for entry in action_log:
    print(f"  {entry['action']:<10} {entry['duration_s']:>7.2f}  "
          f"{entry['mean_tension']:>9.6f}  "
          f"{entry['std_tension']:>9.6f}  "
          f"{entry['mean_dv']:>+10.6f}  "
          f"{entry['mean_stick']:>8.3f}  "
          f"{entry['max_stick']:>7.3f}")

print("\n=== PIXEL QUALITY PER ACTION (aggregated) ===")
print(f"  {'ACTION':<10} {'TOTAL_DUR(s)':>13}  {'MEAN_T':>9}  {'STD_T':>9}  {'MEAN_DV':>10}  {'SEGMENTS':>9}")
action_agg = defaultdict(lambda: {"dur": 0.0, "tensions": [], "dvs": [], "segs": 0})
for entry in action_log:
    a = entry["action"]
    action_agg[a]["dur"]      += entry["duration_s"]
    action_agg[a]["tensions"].append(entry["mean_tension"])
    action_agg[a]["dvs"].append(entry["mean_dv"])
    action_agg[a]["segs"]     += 1
print(f"  {'-'*10} {'-'*13}  {'-'*9}  {'-'*9}  {'-'*10}  {'-'*9}")
for action, agg in sorted(action_agg.items(), key=lambda x: x[1]["dur"], reverse=True):
    mt  = float(np.mean(agg["tensions"])) if agg["tensions"] else 0.0
    st  = float(np.std(agg["tensions"]))  if agg["tensions"] else 0.0
    mdv = float(np.mean(agg["dvs"]))      if agg["dvs"]      else 0.0
    print(f"  {action:<10} {agg['dur']:>13.2f}  {mt:>9.6f}  {st:>9.6f}  {mdv:>+10.6f}  {agg['segs']:>9}")

print("\n=== NODE LIST (top 20) ===")
print(f"  {'NODE':<30} {'COUNT':>7}  {'MEAN_DV':>10}  {'STD_DV':>9}")
print(f"  {'-'*30} {'-'*7}  {'-'*10}  {'-'*9}")
for n, c in sorted(node_support.items(), key=lambda x: x[1], reverse=True)[:20]:
    mdv, sdv = mean_std(node_dv_hist[n])
    print(f"  {str(n):<30} {c:>7}  {mdv:>+10.6f}  {sdv:>9.6f}")

print("\n=== REIFICATION NODES FOUND ===")
meta_final = []
for e, cnt in pair_support.items():
    if cnt < MIN_SUPPORT_PAIR: continue
    mdv, sdv = mean_std(pair_dv_hist[e])
    if mdv >= MIN_MEAN_DV_PAIR and sdv <= MAX_STD_DV_PAIR:
        meta_final.append((e, cnt, mdv, sdv))
meta_final.sort(key=lambda x: (x[2], x[1]), reverse=True)

if not meta_final:
    print("  (none) — try longer session or adjust thresholds")
else:
    for i, (e, cnt, mdv, sdv) in enumerate(meta_final):
        print(f"  {i+1}) {str(e):<50} cnt={cnt}  mean_dv={mdv:+.6f}  std={sdv:.6f}")
