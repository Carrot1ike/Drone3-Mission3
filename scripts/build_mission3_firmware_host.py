#!/usr/bin/env python3
"""Compile official unmodified estimator C with official CMSIS host math.

Generated configuration selects the CF2 estimator constants. The legacy
propeller option has no effect without battery compensation; propulsion is not
compiled or executed by this bench.
No aircraft firmware binary is replaced or flashed.
"""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
FW=ROOT/"src/bitcraze-mission3-firmware"
CMSIS=ROOT/"src/crazyflie-ros2-sil/crazyflie-firmware/vendor/CMSIS/CMSIS"
OUT=ROOT/"build/mission3-firmware-host"

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"autoconf.h").write_text("#define CONFIG_PLATFORM_CF2 1\n#define CONFIG_CRAZYFLIE_LEGACY_PROPELLERS 1\n")
    sources=[FW/"src/modules/src/kalman_core"/p for p in ("kalman_core.c","mm_flow.c","mm_tof.c")]
    sources.append(ROOT/"scripts/mission3/firmware_host.c")
    for p in ("MatrixFunctions/arm_mat_mult_f32.c","MatrixFunctions/arm_mat_trans_f32.c",
              "MatrixFunctions/arm_mat_scale_f32.c","FastMathFunctions/arm_cos_f32.c",
              "FastMathFunctions/arm_sin_f32.c","CommonTables/arm_common_tables.c"):
        sources.append(CMSIS/"DSP/Source"/p)
    includes=[OUT]+[FW/p for p in ("src/modules/interface/kalman_core","src/modules/interface",
          "src/utils/interface","src/utils/interface/lighthouse","src/drivers/interface","src/hal/interface","src/platform/interface","src/config")]
    includes.extend((CMSIS/"DSP/Include",CMSIS/"Core/Include"))
    cmd=["gcc","-shared","-fPIC","-O2","-std=gnu11","-fno-strict-aliasing","-DUNIT_TEST_MODE",
         "-Wno-address-of-packed-member","-Wno-pointer-to-int-cast"]
    cmd += ["-I"+str(p) for p in includes]+[str(p) for p in sources]+["-lm","-o",str(OUT/"libmission3_estimator.so")]
    proc=subprocess.run(cmd,text=True,capture_output=True)
    (OUT/"compile.log").write_text(proc.stdout+proc.stderr)
    (OUT/"source-manifest.json").write_text(json.dumps({
        "scope":"official Kalman predict/process-noise/finalize/Flow/ToF C; host wrapper, no drivers/RTOS/flight control",
        "firmware_commit":subprocess.check_output(["git","-C",str(FW),"rev-parse","HEAD"],text=True).strip(),
        "cmsis_commit":subprocess.check_output(["git","-C",str(CMSIS.parent),"rev-parse","HEAD"],text=True).strip(),
        "cmsis_source_is_shared_baseline":True,
        "source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "command":cmd,"exit_code":proc.returncode},indent=2)+"\n")
    if proc.returncode:
        print(proc.stderr[-6500:]);return proc.returncode
    print(OUT/"libmission3_estimator.so");return 0

if __name__=="__main__":raise SystemExit(main())
