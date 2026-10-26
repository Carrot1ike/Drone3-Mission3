"""Build an isolated EGO firmware with the user-authorized ToF pass marker."""
from pathlib import Path
import argparse
import difflib
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = Path('/home/razer/Desktop/Drone3')

def build(dest):
    dest = Path(dest).resolve()
    if not dest.is_relative_to(ROOT/'.local') or dest.exists():
        raise ValueError('use a new .local directory')
    original = BASE/'virtual_aideck/runtime/build/ego_firmware'
    dest.mkdir(parents=True)
    for name in ('src', 'vendor', 'sitl_make'):
        shutil.copytree(original/name, dest/name)
    shutil.copy2(original/'version.c', dest/'version.c')
    shutil.copytree(BASE/'virtual_aideck/runtime/state_link', dest/'ego',
                    ignore=shutil.ignore_patterns('__pycache__'))
    changes = []
    def change(path, before, after):
        p = dest/path
        text = p.read_text()
        if text.count(before) != 1:
            raise ValueError(f'missing unique anchor in {path}: {before}')
        result = text.replace(before, after)
        p.write_text(result)
        changes.extend(difflib.unified_diff(text.splitlines(True),result.splitlines(True),
                                           fromfile='a/'+path,tofile='b/'+path))
    path = 'sitl_make/CMakeLists.txt'
    p = dest/path
    p.write_text(p.read_text().replace(str(BASE/'virtual_aideck/runtime/state_link'), str(dest/'ego')))
    path = 'src/modules/src/estimator/estimator_kalman.c'
    change(path, 'static bool robustTdoa = false;', '''static bool robustTdoa = false;

// Host explicitly marks the short straight flight through a ring.
// Omit ONLY ToF EKF updates while the host executes this marked command.
// Raw ToF telemetry, flow, IMU, attitude, PID and supervisor remain original.
// Host clears the marker at straight-flight completion and before landing.
static uint8_t ringPass = 0;
static void ringPassChanged(void) {
  DEBUG_PRINT("Ring pass ToF hold %u at %lu ms\\n", (unsigned)ringPass,
              (unsigned long)T2M(xTaskGetTickCount()));
}''')
    change(path, 'kalmanCoreUpdateWithTof(&coreData, &m.data.tof);',
           'if (!ringPass) kalmanCoreUpdateWithTof(&coreData, &m.data.tof);')
    change(path, 'PARAM_GROUP_START(kalman)', '''PARAM_GROUP_START(kalman)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, ringPass, &ringPass, ringPassChanged)''')
    (dest/'ring_pass.patch').write_text(''.join(changes))
    with (dest/'build.log').open('w') as log:
        for cmd in (['cmake','-S',str(dest/'sitl_make'),'-B',str(dest/'ON'),'-DEGO_STATE_LINK=ON'],
                    ['cmake','--build',str(dest/'ON'),'--target','cf2','-j8']):
            subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
    result = dict(binary=str(dest/'ON/cf2'),binary_sha256=hashlib.sha256((dest/'ON/cf2').read_bytes()).hexdigest(),
                  source=str(original),parameter='kalman.ringPass',automatic_timeout=None,
                  suppressed_measurement='ToF EKF update only',baseline_modified=False)
    (dest/'manifest.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))

if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('destination');build(p.parse_args().destination)
