#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONNOUSERSITE=1 MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=scripts
exec src/crazyflie-ros2-sil/.venv/bin/python scripts/run_mission3_physical.py \
  --respect-session-deadline \
  --config config/mission3-35x12-entry-hold.json \
  --profile config/mission3-rpyt-35x12-entry-hold-limited.json "$@"
