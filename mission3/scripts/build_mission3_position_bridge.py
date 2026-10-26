#!/usr/bin/env python3
"""Build official position/velocity PID separately for PC RPYT experiment."""
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
FW=ROOT/"src/bitcraze-mission3-firmware"
CMSIS=ROOT/"src/crazyflie-ros2-sil/crazyflie-firmware/vendor/CMSIS/CMSIS"
OUT=ROOT/"build/mission3-position-bridge"

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"autoconf.h").write_text("#define CONFIG_PLATFORM_CF2 1\n#define CONFIG_CRAZYFLIE_LEGACY_PROPELLERS 1\n#define CONFIG_ENABLE_THRUST_BAT_COMPENSATED 1\n")
    sources=[FW/p for p in ("src/modules/src/controller/position_controller_pid.c","src/utils/src/pid.c","src/utils/src/filter.c","src/utils/src/num.c")]
    sources.append(ROOT/"scripts/mission3/position_bridge_host.c")
    includes=[OUT]+[FW/p for p in ("src/modules/interface","src/modules/interface/controller","src/utils/interface",
               "src/utils/interface/lighthouse","src/drivers/interface","src/hal/interface","src/platform/interface","src/config")]
    includes += [CMSIS/"DSP/Include",CMSIS/"Core/Include"]
    command=["gcc","-shared","-fPIC","-O2","-std=gnu11","-fno-strict-aliasing","-DUNIT_TEST_MODE",
             "-Wno-address-of-packed-member","-Wno-pointer-to-int-cast"]
    command += ["-I"+str(p) for p in includes]+[str(p) for p in sources]+["-lm","-o",str(OUT/"libmission3_position_bridge.so")]
    result=subprocess.run(command,text=True,capture_output=True)
    (OUT/"compile.log").write_text(result.stdout+result.stderr)
    manifest={"command":command,"exit_code":result.returncode,
              "scope":"unmodified official position/velocity PID as PC angle/thrust outer loop; not a hardware link",
              "firmware_commit":subprocess.check_output(["git","-C",str(FW),"rev-parse","HEAD"],text=True).strip(),
              "source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              "header_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (OUT/"autoconf.h",FW/"src/platform/interface/platform_defaults_cf2.h",FW/"src/config/config.h")}}
    (OUT/"source-manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    if result.returncode:print(result.stderr[-5000:]);return result.returncode
    print(OUT/"libmission3_position_bridge.so");return 0

if __name__=="__main__":raise SystemExit(main())
