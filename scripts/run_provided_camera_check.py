#!/usr/bin/env python3
"""Reproducible fixed-camera checks; no flight model, firmware or commands."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from zoneinfo import ZoneInfo

from audit_provided_hardware import audit
from provided_hardware import ROOT
from verify_provided_camera import verify


def main():
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out = ROOT / "artifacts/provided-camera" / run_id
    out.mkdir(parents=True)
    names = ["provided_hardware.py", "audit_provided_hardware.py", "moving_landing_scene.py", "moving_landing_core.py",
             "verify_provided_camera.py", "provided_camera_stream.py", "provided_camera_node.py", "provided_camera.launch.py",
             "run_provided_camera_check.py", "run_shared_ros.py"]
    hashes = {name: hashlib.sha256((ROOT / "scripts" / name).read_bytes()).hexdigest() for name in names}
    (out / "implementation-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    (out / "hardware-audit.json").write_text(json.dumps(audit(), ensure_ascii=False, indent=2) + "\n")
    if verify(out):
        return 1
    print(f"제공 카메라 입력·전송 검사: {out}", flush=True)
    result = subprocess.run([sys.executable, str(ROOT / "scripts/run_shared_ros.py"), "--duration", "30",
                             "--launch-file", str(ROOT / "scripts/provided_camera.launch.py"), "--experiment-dir", str(out)])
    report_path = out / "camera-transport.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else {"passed": False, "failure": "missing_transport_report"}
    report["launcher_exit_code"] = result.returncode
    (out / "camera-transport.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    (ROOT / "artifacts/provided-camera/latest.json").write_text(json.dumps({"artifact_dir": str(out),
            "passed": report["passed"] and result.returncode == 0, "hardware_landing_validated": False}, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] and result.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
