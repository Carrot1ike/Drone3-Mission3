#!/usr/bin/env bash
set -euo pipefail
drone3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$drone3_root/src/crazyflie-ros2-sil/.venv/bin/python" "$drone3_root/scripts/run_mission3_physical.py" "$@"
