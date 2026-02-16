#!/usr/bin/env bash
set -euo pipefail

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/foxy/setup.bash
source "$HOME/go2w_ws/install/local_setup.bash"

exec bash -i
