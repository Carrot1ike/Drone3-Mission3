#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
export PATH="$PROJECT_ROOT/.venv/bin:$PATH"
mkdir -p build logs
FIRMWARE="$PROJECT_ROOT/src/CrazySim/crazyflie-firmware"
cmake -S "$FIRMWARE/sitl_make" -B build/firmware 2>&1 | tee logs/configure-firmware.log
cmake --build build/firmware --target cf2 -j 8 2>&1 | tee logs/build-firmware.log
if [[ "${1:-}" == --firmware-only ]]; then
  exit 0
fi
if ! command -v gz >/dev/null; then
  echo "Gazebo가 없습니다. scripts/install_gazebo.sh를 먼저 실행하세요." >&2
  exit 1
fi
cmake -S "$FIRMWARE/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim" \
  -B build/gazebo -DCMAKE_BUILD_TYPE=Debug 2>&1 | tee logs/configure-gazebo.log
cmake --build build/gazebo -j 8 2>&1 | tee logs/build-gazebo.log
