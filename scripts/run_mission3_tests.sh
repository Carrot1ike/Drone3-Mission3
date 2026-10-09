#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/mission3_runtime.bash"
cd "$mission3_root"
"$mission3_python" "$mission3_root/scripts/check_mission3_environment.py" --quick
"$mission3_python" -m unittest discover -s scripts/mission3 -t scripts -p 'test_*.py'
"$mission3_python" -m unittest test_mission3_automation test_mission3_bootstrap
