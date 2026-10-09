#!/usr/bin/env bash
set -eo pipefail
gateway_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$gateway_project_root/config/shared_ros_env.bash"
# ROS 기록도 이 sidecar의 소유 디렉터리 안에 보존한다.
export ROS_LOG_DIR="$gateway_project_root/artifacts/mission3/ros-gateway/ros-logs"
mkdir -p "$ROS_LOG_DIR"
gateway_script=mission3_ros_gateway.py
if [[ "${1:-}" == "--test" ]]; then
    gateway_script=test_mission3_ros_gateway.py
    shift
fi
exec "$CRAZYSIM_ROOT/.venv/bin/python" "$gateway_project_root/scripts/$gateway_script" "$@"
