#!/bin/bash
# Patch entry-point shebangs to use the project venv after `colcon build`.
# ament_python hard-codes #!/usr/bin/python3 which can't see venv packages
# (piper, requests). Rerun this after every rebuild.

set -e
VENV_PY="${VENV_PY:-$HOME/ggcnn_env/bin/python}"
PKG_DIR="$HOME/ros2_ws/install/go2w_communication/lib/go2w_communication"

if [[ ! -x "$VENV_PY" ]]; then
  echo "Error: venv python not found at $VENV_PY" >&2
  echo "Override with VENV_PY=/path/to/venv/bin/python $0" >&2
  exit 1
fi

for f in "$PKG_DIR"/chat "$PKG_DIR"/chat_stream "$PKG_DIR"/chat_webrtc "$PKG_DIR"/speak; do
  if [[ -f "$f" ]]; then
    sed -i "1s|^#!.*|#!$VENV_PY|" "$f"
    echo "Patched $f -> $VENV_PY"
  fi
done
