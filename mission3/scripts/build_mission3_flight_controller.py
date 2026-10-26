#!/usr/bin/env python3
"""Build official CF2 stock PID/mixer as a host library, no vendor changes."""
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
FW=ROOT/"src/bitcraze-mission3-firmware"
CMSIS=ROOT/"src/crazyflie-ros2-sil/crazyflie-firmware/vendor/CMSIS/CMSIS"
OUT=ROOT/"build/mission3-flight-controller"

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"autoconf.h").write_text("#define CONFIG_PLATFORM_CF2 1\n#define CONFIG_CRAZYFLIE_LEGACY_PROPELLERS 1\n#define CONFIG_ENABLE_THRUST_BAT_COMPENSATED 1\n")
    # RPYT decoder includes this header but calls no FreeRTOS functions.
    # The host does not emulate scheduling, supervisor or link timeouts.
    (OUT/"FreeRTOS.h").write_text("/* Host stub: RPYT decoder uses no RTOS API. */\n")
    # Extract exactly the manufacturer function; embedded peripherals are not
    # available on the host. Preserve the originating file/body hashes.
    motor_source=FW/"src/drivers/src/motors.c"
    raw=motor_source.read_text()
    start=raw.index("float motorsCompensateBatteryVoltage(")
    end=raw.index("/* Public functions */",start)
    body=raw[start:end].rstrip()+"\n"
    (OUT/"official_motor_compensation.inc").write_text(body)
    sources=[FW/p for p in ("src/modules/src/controller/controller_pid.c",
       "src/modules/src/controller/position_controller_pid.c","src/modules/src/controller/attitude_pid_controller.c",
       "src/modules/src/power_distribution_quadrotor.c","src/modules/src/crtp_commander_rpyt.c",
       "src/utils/src/pid.c","src/utils/src/filter.c","src/utils/src/num.c")]
    sources.append(ROOT/"scripts/mission3/flight_controller_host.c")
    includes=[OUT]+[FW/p for p in ("src/modules/interface","src/modules/interface/controller","src/utils/interface",
             "src/utils/interface/lighthouse","src/drivers/interface","src/hal/interface","src/platform/interface","src/config")]
    includes += [CMSIS/"DSP/Include",CMSIS/"Core/Include"]
    cmd=["gcc","-shared","-fPIC","-O2","-std=gnu11","-fno-strict-aliasing","-DUNIT_TEST_MODE",
         "-Wno-address-of-packed-member","-Wno-pointer-to-int-cast"]
    cmd += ["-I"+str(p) for p in includes]+[str(p) for p in sources]+["-lm","-o",str(OUT/"libmission3_controller.so")]
    p=subprocess.run(cmd,text=True,capture_output=True)
    (OUT/"compile.log").write_text(p.stdout+p.stderr)
    (OUT/"source-manifest.json").write_text(json.dumps({"command":cmd,"exit_code":p.returncode,
      "scope":"official stock PID and power mixer host execution, not RTOS or motor hardware",
      "firmware_commit":subprocess.check_output(["git","-C",str(FW),"rev-parse","HEAD"],text=True).strip(),
      "extracted_function":{"source":str(motor_source.relative_to(ROOT)),
          "source_sha256":hashlib.sha256(motor_source.read_bytes()).hexdigest(),
          "body_sha256":hashlib.sha256(body.encode()).hexdigest(),
          "function":"motorsCompensateBatteryVoltage", "body_modified":False},
      "header_sha256":{str(h.relative_to(ROOT)):hashlib.sha256(h.read_bytes()).hexdigest() for h in
          [OUT/"autoconf.h",OUT/"FreeRTOS.h",FW/"src/platform/interface/platform_defaults.h",FW/"src/platform/interface/platform_defaults_cf2.h",
           FW/"src/config/config.h",FW/"src/modules/interface/stabilizer_types.h"]},
      "source_sha256":{str(s.relative_to(ROOT)):hashlib.sha256(s.read_bytes()).hexdigest() for s in sources}},indent=2)+"\n")
    if p.returncode:print(p.stderr[-5000:]);return p.returncode
    print(OUT/"libmission3_controller.so");return 0
if __name__=="__main__":raise SystemExit(main())
