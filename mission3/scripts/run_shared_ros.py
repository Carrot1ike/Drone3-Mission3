#!/usr/bin/env python3
"""Run the unmodified shared ROS launch, keeping cleanup inside its session."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SHARED = ROOT / "src/crazyflie-ros2-sil"


def main():
    parser = argparse.ArgumentParser(description="공유 저장소 ROS 2 SIL 실행")
    parser.add_argument("--duration", type=float, help="지정한 초 뒤 종료")
    parser.add_argument("--verify", action="store_true", help="공유 baseline 수신 검사 후 종료")
    parser.add_argument("--scene", help="공유 저장소의 scene 이름 또는 XML 절대 경로")
    parser.add_argument("--launch-file", type=Path, help="프로젝트 확장 ROS launch 파일")
    parser.add_argument("--experiment-dir", type=Path, help="프로젝트 실험 출력 폴더")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration은 양수여야 합니다.")
    if args.verify and args.duration is not None:
        parser.error("--verify와 --duration 중 하나를 선택하세요.")
    if args.launch_file and (not args.launch_file.is_file() or ROOT / "scripts" not in args.launch_file.resolve().parents):
        parser.error("--launch-file은 프로젝트 scripts 아래의 파일이어야 합니다.")
    for required in [Path("/opt/ros/humble/setup.bash"),
                     SHARED / ".venv/bin/python",
                     SHARED / "crazyswarm2_ws/install/local_setup.bash"]:
        if not required.is_file():
            parser.error(f"설치가 필요합니다: {required}")
    if not os.environ.get("DISPLAY"):
        parser.error("공유 baseline은 GUI를 사용합니다. 그래픽 세션에서 실행하세요.")
    launch_args = []
    if args.scene:
        path = Path(args.scene).expanduser()
        if not path.is_absolute():
            path = SHARED / "crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco" / path
        if not path.is_file():
            parser.error(f"scene을 찾을 수 없습니다: {path}")
        launch_args.append(f"scene:={path.resolve()}")
    (ROOT / "build").mkdir(exist_ok=True)
    lock = (ROOT / "build/shared-runtime.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("이미 이 프로젝트의 공유 SIL이 실행 중입니다.")
    for kind, port in [(socket.SOCK_DGRAM, 19850), (socket.SOCK_DGRAM, 19950),
                       (socket.SOCK_DGRAM, 5200), (socket.SOCK_STREAM, 5050)]:
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, int(kind == socket.SOCK_STREAM))
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                parser.error(f"baseline 포트 {port} 사용 중. 기존 실행을 먼저 종료하세요.")
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    log_dir = ROOT / "logs" / f"shared-ros-{run_id}"
    log_dir.mkdir(parents=True)
    shim_dir = ROOT / "build/shared-launch/bin"
    shim_dir.mkdir(parents=True, exist_ok=True)
    shim = shim_dir / "pkill"
    if shim.exists() or shim.is_symlink():
        if not shim.is_symlink() or shim.resolve() != ROOT / "scripts/shared_session_pkill.py":
            parser.error(f"예상과 다른 실행 shim: {shim}")
    else:
        shim.symlink_to(ROOT / "scripts/shared_session_pkill.py")
    env = os.environ.copy()
    for key in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH",
                "ROS_PACKAGE_PATH", "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION",
                "VIRTUAL_ENV", "LD_LIBRARY_PATH"]:
        env.pop(key, None)
    env.update(CRAZYSIM_ROOT=str(SHARED), ROS_DOMAIN_ID="25", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
               PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
               ROS_LOG_DIR=str(log_dir / "ros"), DRONE3_SCOPED_PKILL_DIR=str(shim_dir),
               DRONE3_SHARED_LOG_DIR=str(log_dir), DRONE3_PROJECT_ROOT=str(ROOT),
               PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin")
    if args.launch_file:
        env["DRONE3_LAUNCH_FILE"] = str(args.launch_file.resolve())
    if args.experiment_dir:
        env["DRONE3_EXPERIMENT_DIR"] = str(args.experiment_dir.resolve())
    shell = ('source "$CRAZYSIM_ROOT/config/crazysim_env.bash" || exit; '
             'export ROS_LOG_DIR="$DRONE3_SHARED_LOG_DIR/ros"; '
             'export PATH="$DRONE3_SCOPED_PKILL_DIR:$PATH"; '
             'export DRONE3_SHARED_SESSION_ID=$$; '
             'if [[ -n "${DRONE3_LAUNCH_FILE:-}" ]]; then '
             'exec ros2 launch "$DRONE3_LAUNCH_FILE" "$@"; else '
             'exec ros2 launch crazysim_bringup single_cf.launch.py "$@"; fi')
    process = None
    outcome = {"run_id": run_id, "repository_commit": "7f2f5b86d9d802865508625f6b95ad214169e188",
               "launch_args": launch_args, "domain": 25, "model": "cf2x_P250" if not args.launch_file else None,
               "flowdeck": True if not args.launch_file else None, "baseline_verifier_passed": None,
               "flight_tested": False if not args.launch_file else None,
               "launch_file": str(args.launch_file) if args.launch_file else None}
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with (log_dir / "launch.log").open("w") as output:
            process = subprocess.Popen(["bash", "-c", shell, "shared-ros", *launch_args],
                                       cwd=SHARED, env=env, start_new_session=True,
                                       stdout=output, stderr=subprocess.STDOUT)
            mode = f"프로젝트 ROS 확장: {args.launch_file.name}" if args.launch_file else "공유 ROS 2 SIL: cf231 / cf2x_P250 / Flow + camera"
            print(f"{mode}\n로그: {log_dir}", flush=True)
            start = time.monotonic()
            deadline = start + (args.duration if args.duration else float("inf"))
            verification_started = False
            while time.monotonic() < deadline:
                if args.experiment_dir and (args.experiment_dir / "control-summary.json").is_file():
                    outcome["experiment_finished"] = True
                    time.sleep(0.5)
                    break
                if process.poll() is not None:
                    outcome["unexpected_exit_code"] = process.returncode
                    raise RuntimeError(f"ROS launch가 종료됐습니다: {process.returncode}; {log_dir / 'launch.log'}")
                if args.verify and not verification_started and time.monotonic() - start >= 13:
                    verification_started = True
                    command = ('source "$CRAZYSIM_ROOT/config/crazysim_env.bash" || exit; '
                               '"$CRAZYSIM_ROOT/scripts/verify_baseline.sh" && '
                               'exec "$CRAZYSIM_ROOT/.venv/bin/python" '
                               '"$DRONE3_PROJECT_ROOT/scripts/verify_shared_sensors.py" '
                               '--output "$DRONE3_SHARED_LOG_DIR/sensor-reception.json"')
                    with (log_dir / "baseline-verifier.log").open("w") as check_output:
                        result = subprocess.run(["bash", "-c", command], env=env, cwd=SHARED,
                                                stdout=check_output, stderr=subprocess.STDOUT, timeout=55)
                    report = (log_dir / "baseline-verifier.log").read_text()
                    print(report, end="", flush=True)
                    outcome["baseline_verifier_passed"] = result.returncode == 0
                    if result.returncode:
                        raise RuntimeError(f"baseline 수신 검사 실패: {log_dir}")
                    break
                time.sleep(0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        if process:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=12)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                pass
            # Include background firmware/CPX children even if ROS launch already exited.
            for sig in [signal.SIGTERM, signal.SIGKILL]:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                if sig == signal.SIGTERM:
                    time.sleep(0.3)
            if process.poll() is None:
                process.wait(timeout=3)
            outcome["launch_exit_code_after_shutdown"] = process.returncode
        outcome["log_dir"] = str(log_dir)
        (log_dir / "run.json").write_text(json.dumps(outcome, ensure_ascii=False, indent=2) + "\n")
        lock.close()


if __name__ == "__main__":
    sys.exit(main())
