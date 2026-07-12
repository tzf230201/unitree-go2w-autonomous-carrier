"""Visualize a DVR_UNMA realtime timeseries CSV.

Usage:
    python3 plot_timeseries.py                    # latest *_timeseries.csv in ./logs
    python3 plot_timeseries.py path/to/file.csv   # specific file
    python3 plot_timeseries.py --save out.png     # save instead of showing
"""
import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

V_LOW, V_HIGH, V_STAR = 0.4, 0.8, 0.8

ACTION_COLORS = {
    "STOP":    "#888888",
    "FORWARD": "#2ecc71",
    "BACK":    "#e74c3c",
    "LEFT":    "#3498db",
    "RIGHT":   "#f39c12",
    "ROT_L":   "#9b59b6",
    "ROT_R":   "#e67e22",
}


def find_latest_timeseries():
    pattern = os.path.join(LOG_DIR, "*_timeseries.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        sys.exit(f"No timeseries CSV found in {LOG_DIR}")
    return files[-1]


def action_segments(df):
    """Yield (t_start, t_end, action) for each contiguous run of the action column."""
    if df.empty:
        return
    changes = df["action"].ne(df["action"].shift()).to_numpy()
    idxs = np.flatnonzero(changes)
    idxs = np.append(idxs, len(df))
    for a, b in zip(idxs[:-1], idxs[1:]):
        yield df["wall_s"].iloc[a], df["wall_s"].iloc[b - 1], df["action"].iloc[a]


def shade_actions(ax, df, alpha=0.12):
    for t0, t1, act in action_segments(df):
        color = ACTION_COLORS.get(act, "#cccccc")
        ax.axvspan(t0, t1, color=color, alpha=alpha, linewidth=0)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", nargs="?", help="Path to *_timeseries.csv")
    p.add_argument("--save", help="Save figure to this path instead of showing")
    args = p.parse_args()

    csv_path = args.csv or find_latest_timeseries()
    df = pd.read_csv(csv_path)
    if df.empty:
        sys.exit(f"{csv_path} has no data rows")

    session_name = os.path.splitext(os.path.basename(csv_path))[0]
    t = df["wall_s"].to_numpy()

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"DVR-UNMA timeseries — {session_name}\n"
                 f"{len(df)} samples, {t[-1] - t[0]:.1f}s wall time",
                 fontsize=11)

    # --- Row 1: tension (t_ema) with t_raw shadow ---
    ax = axes[0]
    shade_actions(ax, df)
    ax.plot(t, df["t_raw"], color="#bbbbbb", linewidth=0.7, label="t_raw")
    ax.plot(t, df["t_ema"], color="#222222", linewidth=1.2, label="t_ema")
    ax.set_ylabel("tension")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Row 2: dv_reg ---
    ax = axes[1]
    shade_actions(ax, df)
    ax.axhline(0, color="#888", linewidth=0.6)
    ax.plot(t, df["dv_reg"], color="#c0392b", linewidth=1.0, label="dv_reg")
    if "food" in df.columns:
        ax.plot(t, df["food"], color="#16a085", linewidth=0.8, alpha=0.7, label="food")
    ax.set_ylabel("dv_reg / food")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Row 3: viability v(t) ---
    ax = axes[2]
    shade_actions(ax, df)
    ax.axhspan(0.0, V_LOW, color="#e74c3c", alpha=0.08, label=f"danger (<{V_LOW})")
    ax.axhline(V_LOW,  color="#e67e22", linewidth=1.0, linestyle="--",
               label=f"V_LOW={V_LOW}")
    ax.axhline(V_HIGH, color="#27ae60", linewidth=1.0, linestyle="--",
               label=f"V_HIGH={V_HIGH}")
    ax.axhline(V_STAR, color="#2c3e50", linewidth=0.8, linestyle=":",
               label=f"V*={V_STAR}")
    ax.plot(t, df["v"], color="#2980b9", linewidth=1.3, label="v(t)")
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("viability v")
    ax.legend(loc="lower right", fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Row 4: stick intensity + regulating flag ---
    ax = axes[3]
    shade_actions(ax, df)
    ax.plot(t, df["stick_intensity"], color="#34495e", linewidth=0.9,
            label="stick_intensity")
    if "regulating" in df.columns:
        ax.fill_between(t, 0, df["regulating"], step="pre", alpha=0.15,
                        color="#e67e22", label="regulating=1")
    ax.set_ylim(-0.05, 1.1)
    ax.set_ylabel("stick / reg")
    ax.set_xlabel("wall_s (seconds since session start)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Action color legend (compact, below suptitle) ---
    seen = []
    handles = []
    for _, _, act in action_segments(df):
        if act in seen:
            continue
        seen.append(act)
        handles.append(plt.Rectangle((0, 0), 1, 1,
                                     color=ACTION_COLORS.get(act, "#cccccc"),
                                     alpha=0.4))
    if handles:
        fig.legend(handles, seen, loc="upper right", ncol=len(seen),
                   fontsize=8, title="actions", bbox_to_anchor=(0.99, 0.97))

    fig.tight_layout(rect=(0, 0, 1, 0.95))

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"Saved: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
