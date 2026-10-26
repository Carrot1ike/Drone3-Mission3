#!/usr/bin/env python3
"""Bounded local backend test, with a separate timeout for CFLib connection."""
import argparse
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / ".venv/bin/python"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("backend", choices=["mujoco", "gazebo"])
    args = parser.parse_args()
    log = ROOT / "logs" / f"verify-{args.backend}.log"
    with log.open("w") as handle:
        runner = subprocess.Popen([str(PYTHON), str(ROOT / "scripts/run_sim.py"),
                                   args.backend, "--headless", "--duration", "60"],
                                  stdout=handle, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 35
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.settimeout(0.2)
                while time.monotonic() < deadline:
                    if runner.poll() is not None:
                        raise RuntimeError(f"시뮬레이터 시작 실패. {log}")
                    probe.sendto(b"\xff", ("127.0.0.1", 19850))
                    try:
                        if probe.recvfrom(64)[0]:
                            break
                    except socket.timeout:
                        pass
                    time.sleep(0.2)
                else:
                    raise RuntimeError(f"UDP 응답 시간 초과. {log}")
            subprocess.run([str(PYTHON), str(ROOT / "scripts/demo_flight.py"),
                            "--output", str(ROOT / f"artifacts/{args.backend}-flight.json")],
                           cwd=ROOT, check=True, timeout=45)
        finally:
            runner.terminate()
            try:
                runner.wait(timeout=20)
            except subprocess.TimeoutExpired:
                runner.kill()
                runner.wait()


if __name__ == "__main__":
    main()
