#!/usr/bin/env bash
set -eo pipefail
task_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$task_project_root/config/shared_ros_env.bash"
exec "$CRAZYSIM_ROOT/.venv/bin/python" "$task_project_root/scripts/run_optical_landing_ros.py" "$@"
