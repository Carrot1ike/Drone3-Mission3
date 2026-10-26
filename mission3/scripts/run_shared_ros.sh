#!/usr/bin/env bash
set -euo pipefail
DRONE3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec /usr/bin/python3 "$DRONE3_ROOT/scripts/run_shared_ros.py" "$@"
