#!/usr/bin/env bash
set -eo pipefail
task_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONNOUSERSITE=1
exec "$task_project_root/src/crazyflie-ros2-sil/.venv/bin/python" "$task_project_root/scripts/run_optical_landing.py" "$@"
