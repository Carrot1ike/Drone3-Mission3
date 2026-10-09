#!/usr/bin/env python3
"""Project launchers: separate logs/builds and stop only owned processes."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "src/CrazySim/crazyflie-firmware"
SIM_FILES = FIRMWARE / "tools/crazyflie-simulation/simulator_files"
PYTHON = ROOT / ".venv/bin/python"


def main():
    parser = argparse.ArgumentParser(description="Drone3 CrazySim 실행")
    parser.add_argument("backend", choices=["mujoco", "gazebo"])
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument("--agents", type=int, default=1, help="드론 수 (기본 1)")
    parser.add_argument("--port", type=int, default=19950, help="펌웨어 UDP 시작 포트")
    parser.add_argument("--model", help="사용할 백엔드의 드론 모델 이름")
    parser.add_argument("--duration", type=float, help="지정한 초 뒤 종료")
    args = parser.parse_args()
    if not 1 <= args.agents <= 32:
        parser.error("--agents는 1~32여야 합니다.")
    if not 101 <= args.port <= 65535 - args.agents + 1:
        parser.error("유효한 펌웨어 포트 범위를 지정하세요.")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration은 양수여야 합니다.")
    models = {"mujoco": ["cf2x_T350", "cf2x_L250", "cf2x_P250", "cf21B_500"],
              "gazebo": ["crazyflie_thrust_upgrade", "crazyflie"]}
    model = args.model or models[args.backend][0]
    if model not in models[args.backend]:
        parser.error(f"모델 이름: {', '.join(models[args.backend])}")
    binary = ROOT / "build/firmware/cf2"
    if not binary.is_file():
        parser.error("펌웨어가 없습니다. scripts/build.sh를 실행하세요.")
    if args.backend == "gazebo":
        if not shutil.which("gz"):
            parser.error("Gazebo가 없습니다. scripts/install_gazebo.sh를 실행하세요.")
        if not (ROOT / "build/gazebo/libgz_crazysim_plugin.so").is_file():
            parser.error("Gazebo 플러그인이 없습니다. scripts/build.sh를 실행하세요.")

    # Fail before starting any processes if another simulation owns these ports.
    for port in [p for i in range(args.agents)
                 for p in (args.port + i, args.port - 100 + i)]:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                parser.error(f"UDP {port} 사용 중. 기존 실행을 종료하거나 --port를 변경하세요.")

    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    log_dir = ROOT / "logs" / f"{args.backend}-{run_id}"
    log_dir.mkdir(parents=True)
    processes = []
    log_handles = []
    env = os.environ.copy()
    env["PATH"] = str(PYTHON.parent) + os.pathsep + env.get("PATH", "")
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["GZ_PARTITION"] = f"drone3-{os.getpid()}"
    env["QT_QPA_PLATFORM"] = "xcb"
    for key, paths in {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": [ROOT / "build/gazebo"],
        "GZ_SIM_RESOURCE_PATH": [SIM_FILES / "gazebo/models", SIM_FILES / "gazebo/worlds"],
        "LD_LIBRARY_PATH": [ROOT / "build/gazebo"],
    }.items():
        env[key] = os.pathsep.join([str(p) for p in paths] +
                                  ([env[key]] if env.get(key) else []))

    def start(name, command, cwd=None):
        folder = log_dir / name
        folder.mkdir()
        handle = (folder / "output.log").open("w")
        log_handles.append(handle)
        process = subprocess.Popen([str(c) for c in command], cwd=cwd or folder,
                                   env=env, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((name, process))
        return process

    def check_alive():
        for name, process in processes:
            if process.poll() is not None:
                raise RuntimeError(f"{name}이 종료됐습니다 (코드 {process.returncode}).")

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        width = math.ceil(math.sqrt(args.agents))
        positions = [(i % width, i // width) for i in range(args.agents)]
        for i in range(args.agents):
            start(f"firmware-{i}", ["stdbuf", "-oL", "-eL", binary, args.port + i])
        if args.backend == "mujoco":
            command = [PYTHON, SIM_FILES / "mujoco/crazysim.py",
                       "--model-type", model, "--port", args.port]
            if not args.headless:
                command.append("--vis")
            command += ["--"] + [f"{x},{y}" for x, y in positions]
            primary = start("simulator", command)
        else:
            gz_dir = SIM_FILES / "gazebo"
            server = start("server", ["stdbuf", "-oL", "-eL", "gz", "sim", "-s", "-r",
                                      gz_dir / "worlds/crazysim_default.sdf", "-v", "3"])
            for i, (x, y) in enumerate(positions):
                model_file = log_dir / f"drone-{i}.sdf"
                subprocess.run([str(PYTHON), str(gz_dir / "launch/jinja_gen.py"),
                                str(gz_dir / f"models/{model}/model.sdf.jinja"), str(gz_dir),
                                "--cffirm_udp_port", str(args.port + i),
                                "--cflib_udp_port", str(args.port - 100 + i),
                                "--cf_id", str(i), "--cf_name", "cf",
                                "--output-file", str(model_file)],
                               env=env, check=True, timeout=10, stdout=subprocess.DEVNULL)
                request = (f'sdf_filename: "{model_file}", '
                           f'pose: {{position: {{x: {x}, y: {y}, z: 0.5}}}}, '
                           f'name: "drone_{i}", allow_renaming: false')
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    check_alive()
                    result = subprocess.run(
                        ["gz", "service", "-s", "/world/crazysim_default/create",
                         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                         "--timeout", "1000", "--req", request], env=env,
                        capture_output=True, text=True, timeout=5)
                    if result.returncode == 0 and "data: true" in result.stdout:
                        break
                    time.sleep(0.25)
                else:
                    raise RuntimeError("Gazebo에 드론 생성 실패. server/output.log를 확인하세요.")
            primary = server if args.headless else start("gui", ["gz", "sim", "-g"])

        print(f"{args.backend} / {model} / 드론 {args.agents}대", flush=True)
        for i in range(args.agents):
            print(f"  연결: udp://127.0.0.1:{args.port - 100 + i}", flush=True)
        print(f"로그: {log_dir}\n종료: Ctrl+C 또는 GUI 창 닫기", flush=True)
        deadline = time.monotonic() + args.duration if args.duration else math.inf
        while time.monotonic() < deadline:
            if primary.poll() is not None:
                if primary.returncode != 0:
                    raise RuntimeError(f"시뮬레이터 종료 코드: {primary.returncode}")
                break
            check_alive()
            time.sleep(0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"실행 실패: {error}\n로그: {log_dir}", file=sys.stderr)
        return 1
    finally:
        # Each child owns an isolated process group. Never kill unrelated sessions.
        for _, process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for _, process in reversed(processes):
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)
        for handle in log_handles:
            handle.close()


if __name__ == "__main__":
    sys.exit(main())
