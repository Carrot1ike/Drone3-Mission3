#!/usr/bin/env bash
set -euo pipefail
mission3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$mission3_root/scripts/setup_mission3.py" "$@"
