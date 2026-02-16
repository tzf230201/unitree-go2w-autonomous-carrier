#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-eth0}"

env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  SHELL=/bin/bash \
  TERM="${TERM:-xterm-256color}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -lc "
    cd /unitree/module/unitree_slam/bin
    exec ./keyDemo ${IFACE}
  "
