# 프로젝트 소유 실행 환경 선택. 다른 저장소/전역 설정을 변경하지 않습니다.
mission3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${MISSION3_PYTHON:-}" ]]; then
    mission3_python="$MISSION3_PYTHON"
elif [[ -x "$mission3_root/.venv-mission3/bin/python" ]]; then
    mission3_python="$mission3_root/.venv-mission3/bin/python"
else
    mission3_python="$mission3_root/src/crazyflie-ros2-sil/.venv/bin/python"
fi
if [[ ! -x "$mission3_python" ]]; then
    echo '미션3 환경이 없습니다. bash scripts/setup_mission3.sh --shared-root /팀장/저장소/경로 를 먼저 실행하세요.' >&2
    return 1
fi
export PYTHONNOUSERSITE=1 MUJOCO_GL="${MUJOCO_GL:-egl}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="$mission3_root/scripts"
