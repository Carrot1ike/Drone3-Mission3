#!/usr/bin/env bash
set -eo pipefail
DRONE3_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
DRONE3_SHARED_ROOT="$DRONE3_PROJECT_ROOT/src/crazyflie-ros2-sil"
mkdir -p "$DRONE3_PROJECT_ROOT/logs"
exec > >(tee "$DRONE3_PROJECT_ROOT/logs/shared-rebuild.log") 2>&1
cd "$DRONE3_SHARED_ROOT"
bash scripts/bootstrap.sh
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
"$DRONE3_SHARED_ROOT/.venv/bin/python" -m pip install --no-deps \
    -r "$DRONE3_PROJECT_ROOT/config/shared_python_supplement.lock.txt"
source "$DRONE3_PROJECT_ROOT/config/shared_ros_env.bash"
python -m pip check
