#!/usr/bin/env bash
set -euo pipefail

SESSION="${1:-go2w_slam}"
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required. Install it with: sudo apt install tmux"
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists. Attach with:"
  echo "tmux attach -t $SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n slam
tmux send-keys -t "${SESSION}:slam.0" "cd '$BASE_DIR' && ./scripts/run_unitree_slam.sh" C-m
tmux split-window -h -t "${SESSION}:slam.0"
tmux send-keys -t "${SESSION}:slam.1" "cd '$BASE_DIR' && ./scripts/run_xt16_driver.sh" C-m
tmux select-layout -t "${SESSION}:slam" tiled

echo "Started session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"
