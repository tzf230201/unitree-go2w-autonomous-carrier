import cv2, numpy as np, time
from collections import defaultdict, deque

# =======================
# CONFIG
# =======================
CAM_INDEX = 0
DS_W, DS_H = 64, 48
K = 5
SMOOTH_ALPHA = 0.2

# DVR parameters (proper formulation)
DECAY_ALPHA = 0.001
V_STAR      = 0.5        # viability setpoint
BETA        = 0.05       # regulation sensitivity
DELTA_MIN   = 0.05       # activation threshold
DELTA_MAX   = 0.8        # capacity ceiling
V_INIT      = 0.5

# DVR Ablation Mode (set True for Exp A comparison)
DVR_ENABLED = True       # <-- toggle this for ablation

# Tension bucket
BUCKET_EDGES = (0.025, 0.07)

# UNMA reification thresholds (stricter for paper)
MIN_SUPPORT_PAIR = 20
MIN_MEAN_DV_PAIR = 0.001    # raised from 0.00002
MAX_STD_DV_PAIR  = 0.003    # lowered from 0.005

# Visualization
PLOT_LEN = 220
PANEL_W  = 620
PANEL_H  = 720
NODE_DV_MAXLEN = 300
PAIR_DV_MAXLEN = 300

ACTION_KEYS = {
    ord('w'): "FORWARD",
    ord('s'): "BACK",
    ord('a'): "LEFT",
    ord('d'): "RIGHT",
    ord('q'): "ROT_L",
    ord('e'): "ROT_R",
    ord(' '): "STOP",
}
DEFAULT_ACTION = "STOP"

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
    xs = np.linspace(x0+10, x0+w-10, len(vals)).astype(int)
    def ymap(v):
        t = (v - vmin) / (vmax - vmin + 1e-9)
        return int(y0 + h - 10 - t*(h-30))
    pts = np.array([(int(xs[i]), ymap(vals[i]))
                    for i in range(len(vals))], dtype=np.int32)
    cv2.polylines(panel, [pts], False, (230,230,230), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{vmin:+.4f}..{vmax:+.4f}", (x0+10, y0+h-8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1, cv2.LINE_AA)

# =======================
# INIT
# =======================
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera {CAM_INDEX}")

buf          = deque(maxlen=K+1)
t_prev       = None
t_ema        = None
v            = V_INIT

node_support  = defaultdict(int)
node_dv_hist  = defaultdict(lambda: deque(maxlen=NODE_DV_MAXLEN))
edge_support  = defaultdict(int)
pair_support  = defaultdict(int)
pair_dv_hist  = defaultdict(lambda: deque(maxlen=PAIR_DV_MAXLEN))

# FIX: one-step delayed edge recording
pending_edge  = None

last_node     = None
active_action = DEFAULT_ACTION
last_print    = time.time()

ts_t  = deque(maxlen=PLOT_LEN)
ts_dv = deque(maxlen=PLOT_LEN)
ts_v  = deque(maxlen=PLOT_LEN)

# Metrics for Exp A comparison
reification_count = 0
total_viability_area = 0.0   # sum of |v - V_STAR| over time (stability metric)
step_count = 0

mode_label = "DVR-ON" if DVR_ENABLED else "DVR-OFF (ablation)"
print(f"UNMA Stage 6 — {mode_label}")
print("Keys: w/a/s/d/q/e=action, space=STOP, x=exit")

# =======================
# MAIN LOOP
# =======================
while True:
    ret, frame = cap.read()
    if not ret: break

    key = cv2.waitKey(1) & 0xFF
    if key == ord('x'): break
    if key in ACTION_KEYS:
        active_action = ACTION_KEYS[key]

    # Preprocess
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

    # ===========================
    # DVR — proper formulation
    # ===========================
    if DVR_ENABLED:
        delta      = V_STAR - v
        clipped    = np.clip(abs(delta), DELTA_MIN, DELTA_MAX) * np.sign(delta)
        u          = BETA * clipped
        decay_term = -DECAY_ALPHA
    else:
        # DVR ablation: no decay, no regulation
        u          = 0.0
        decay_term = 0.0

    v = v + decay_term + u + dv_reg
    v = float(np.clip(v, -1.0, 1.0))

    # ===========================
    # UNMA — node + edge
    # ===========================
    tb   = bucket_tension(t_ema)
    node = (active_action, tb)

    node_support[node] += 1
    node_dv_hist[node].append(dv_reg)

    # FIX: delayed edge — record consequence of PREVIOUS transition
    if pending_edge is not None:
        edge_support[pending_edge] += 1
        pair_support[pending_edge] += 1
        pair_dv_hist[pending_edge].append(dv_reg)  # consequence dv

    # Queue new transition
    if last_node is not None and node != last_node:
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

    # Track metrics for Exp A
    reification_count   = len(meta_pairs)
    total_viability_area += abs(v - V_STAR)
    step_count          += 1

    # Print
    if time.time() - last_print >= 1.0:
        avg_v_dev = total_viability_area / max(step_count, 1)
        print(f"[{mode_label}] t={t_ema:.4f} tb={tb} dv={dv_reg:+.6f} "
              f"v={v:+.3f} reif={reification_count} avg_|Δv|={avg_v_dev:.5f}")
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

    put(frame, f"UNMA {mode_label} (x=exit)", 10, 30, 0.7, 2)
    put(frame, f"ACTIVE={active_action}", 10, 60, 0.8, 2)

    panel = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    mode_color = (100, 255, 100) if DVR_ENABLED else (100, 100, 255)
    cv2.putText(panel, f"VISUAL OUTPUT — {mode_label}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2, cv2.LINE_AA)
    put(panel, f"ACTIVE: {active_action}", 20, 75, 0.8, 2)
    put(panel, f"t_ema={t_ema:.4f}  bucket={tb}  dv={dv_reg:+.6f}", 20, 108, 0.6, 1)
    put(panel, f"v={v:+.3f}  u={u:+.4f}  reif_count={reification_count}", 20, 133, 0.6, 1)
    put(panel, f"nodes={len(node_support)}  edges={len(edge_support)}", 20, 158, 0.6, 1)

    draw_timeseries(panel, list(ts_t),  20, 185, PANEL_W-40, 110, 0.0,  0.15, "t_ema (EMA tension)")
    draw_timeseries(panel, list(ts_dv), 20, 305, PANEL_W-40, 110, -0.03, 0.03, "dv_reg (T_prev - T_now)")
    draw_timeseries(panel, list(ts_v),  20, 425, PANEL_W-40, 110, -0.2,  0.8,  "viability v(t)")

    y = 555
    cv2.rectangle(panel, (20, y), (PANEL_W-20, PANEL_H-20), (80,80,80), 1)
    put(panel, "META(PAIR) candidates:", 30, y+25, 0.6, 1)
    if meta_pairs:
        for i, (e, cnt, mdv, sdv) in enumerate(meta_pairs[:5]):
            s = f"{i+1}) {e[0]} -> {e[1]} | cnt={cnt} mean_dv={mdv:+.6f}"
            put(panel, s, 30, y+52+i*25, 0.52, 1)
    else:
        put(panel, "(none yet)", 30, y+52, 0.52, 1)

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
cap.release()
cv2.destroyAllWindows()

avg_v_deviation = total_viability_area / max(step_count, 1)

print(f"\n=== SUMMARY [{mode_label}] ===")
print(f"Total steps:        {step_count}")
print(f"Final nodes:        {len(node_support)}")
print(f"Final edges:        {len(edge_support)}")
print(f"Reification nodes:  {reification_count}")
print(f"Avg |v - v*|:       {avg_v_deviation:.6f}  (lower = more stable)")

print("\n=== NODE LIST (top 20) ===")
for n, c in sorted(node_support.items(), key=lambda x: x[1], reverse=True)[:20]:
    mdv, sdv = mean_std(node_dv_hist[n])
    print(f"  {n}  count={c}  mean_dv={mdv:+.6f}  std={sdv:.6f}")

print("\n=== REIFICATION NODES FOUND ===")
meta_final = []
for e, cnt in pair_support.items():
    if cnt < MIN_SUPPORT_PAIR: continue
    mdv, sdv = mean_std(pair_dv_hist[e])
    if mdv >= MIN_MEAN_DV_PAIR and sdv <= MAX_STD_DV_PAIR:
        meta_final.append((e, cnt, mdv, sdv))
meta_final.sort(key=lambda x: (x[2], x[1]), reverse=True)

if not meta_final:
    print("(none) — try longer session or adjust thresholds")
else:
    for i, (e, cnt, mdv, sdv) in enumerate(meta_final):
        print(f"  {i+1}) {e}  cnt={cnt}  mean_dv={mdv:+.6f}  std={sdv:.6f}")