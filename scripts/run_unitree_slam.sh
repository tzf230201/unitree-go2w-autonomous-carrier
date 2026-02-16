#!/usr/bin/env bash
set -euo pipefail

env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  SHELL=/bin/bash \
  TERM="${TERM:-xterm-256color}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -lc '
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${HOME}/go2w_ws/cyclonedds_ws/cyclonedds.xml"
    cd /unitree/module/unitree_slam/bin
    exec ./unitree_slam
  '
