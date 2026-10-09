#!/usr/bin/env bash
set -euo pipefail
: "${DRONE3_PROJECT_ROOT:?}" "${DRONE3_EXPERIMENT_DIR:?}" "${CRAZYSIM_ROOT:?}"
task_sim_dir="$CRAZYSIM_ROOT/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco"
task_run_dir="$DRONE3_EXPERIMENT_DIR/firmware"
mkdir -p "$task_run_dir"
task_fw_pid=""; task_cpx_pid=""
cleanup_yaw() {
    [[ -z "$task_fw_pid" ]] || kill "$task_fw_pid" 2>/dev/null || true
    [[ -z "$task_cpx_pid" ]] || kill "$task_cpx_pid" 2>/dev/null || true
}
trap cleanup_yaw EXIT INT TERM
(cd "$task_run_dir" && exec "$CRAZYSIM_ROOT/crazyflie-firmware/sitl_make/build/cf2" 19950 >out.log 2>error.log) &
task_fw_pid=$!
sleep 1
"$CRAZYSIM_ROOT/.venv/bin/python" "$task_sim_dir/crazysim_cpx.py" --camera-only &
task_cpx_pid=$!
sleep 1
"$CRAZYSIM_ROOT/.venv/bin/python" "$DRONE3_PROJECT_ROOT/scripts/yaw_sim.py" \
    --model-type cf2x_P250 --port 19950 --vis --camera --flowdeck \
    --scene "$DRONE3_EXPERIMENT_DIR/scene.xml" -- 0,0
