#!/usr/bin/env python3
"""Local CrazySim: CFLib connection, takeoff/landing, measured altitude check."""
import argparse
import json
from pathlib import Path
import statistics
import time
from urllib.parse import urlparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description="가상 드론 0.5m 이륙·착륙 검증")
    parser.add_argument("--uri", default="udp://127.0.0.1:19850")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/flight.json")
    args = parser.parse_args()
    uri = urlparse(args.uri)
    if uri.scheme != "udp" or uri.hostname not in ("127.0.0.1", "localhost"):
        parser.error("이 데모는 PC에서 실행한 CrazySim의 localhost UDP 연결 전용입니다.")
    cflib.crtp.init_drivers()
    cache = ROOT / "build/cflib-cache"
    cache.mkdir(parents=True, exist_ok=True)
    samples = []
    started = time.monotonic()
    print(f"연결: {args.uri}", flush=True)
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache))) as scf:
        cf = scf.cf
        scf.wait_for_params()
        protocol = cf.platform.get_protocol_version()
        config = LogConfig(name="Drone3Flight", period_in_ms=100)
        config.add_variable("stateEstimate.z", "float")
        cf.log.add_config(config)
        config.data_received_cb.add_callback(
            lambda _timestamp, data, _config: samples.append(
                {"t": time.monotonic() - started, "z": float(data["stateEstimate.z"])}))
        config.start()
        time.sleep(1)
        if "enHighLevel" in cf.param.toc.toc.get("commander", {}):
            cf.param.set_value("commander.enHighLevel", "1")
        cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        cf.param.set_value("kalman.resetEstimation", "0")
        time.sleep(1)
        cf.supervisor.send_arming_request(True)
        time.sleep(0.5)
        commander = cf.high_level_commander
        try:
            print("이륙: 목표 높이 0.5m", flush=True)
            commander.takeoff(0.5, 2.0)
            time.sleep(4.5)
            hover_end = time.monotonic() - started
            hover = [s["z"] for s in samples if hover_end - 1.5 <= s["t"] <= hover_end]
            print(f"측정한 호버 높이: {statistics.median(hover):.3f}m", flush=True)
            print("착륙", flush=True)
            commander.land(0.0, 2.0)
            time.sleep(3.5)
            landing_end = time.monotonic() - started
            landing = [s["z"] for s in samples if landing_end - 1 <= s["t"] <= landing_end]
        finally:
            commander.stop()
            cf.supervisor.send_arming_request(False)
            config.stop()
        result = {
            "uri": args.uri, "protocol_version": protocol,
            "target_height_m": 0.5, "samples": len(samples),
            "hover_height_m": statistics.median(hover),
            "landed_height_m": statistics.median(landing),
            "max_height_m": max(s["z"] for s in samples),
            "telemetry": samples,
        }
        result["passed"] = (len(samples) >= 50 and
                            0.35 <= result["hover_height_m"] <= 0.65 and
                            abs(result["landed_height_m"]) < 0.12)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        if not result["passed"]:
            raise RuntimeError(f"높이 검증 실패. 측정 결과: {args.output}")
        print(f"검증 통과: {args.output}", flush=True)


if __name__ == "__main__":
    main()
