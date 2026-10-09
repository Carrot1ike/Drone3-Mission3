#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/mission3_runtime.bash"
"$mission3_python" "$mission3_root/scripts/check_mission3_environment.py" --quick
exec "$mission3_python" "$mission3_root/scripts/run_mission3_physical.py" "$@"
