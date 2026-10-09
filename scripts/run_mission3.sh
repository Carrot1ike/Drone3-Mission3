#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/mission3_runtime.bash"
cd "$mission3_root"
exec "$mission3_python" "$mission3_root/scripts/run_mission3.py" "$@"
