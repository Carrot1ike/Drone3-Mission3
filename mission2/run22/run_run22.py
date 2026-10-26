"""Prepare/run the frozen run22 on an installed original Linux SITL checkout.

Only machine-specific path literals are adapted in a new working copy. The
packaged frozen source and historical evidence are never edited.
"""
import argparse, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True,help='Installed Dynamic-Drone-Mission-SITL checkout')
    p.add_argument('--work-dir',type=Path,required=True,help='New directory; must not exist')
    p.add_argument('--host-python',type=Path,default=Path(sys.executable))
    p.add_argument('--native-python',type=Path)
    p.add_argument('--firmware',type=Path)
    p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args();bundle=Path(__file__).resolve().parent
    base=a.baseline.resolve();host=a.host_python.resolve()
    native=(a.native_python or base/'Drone/bin/python').resolve()
    firmware=(a.firmware or bundle/'firmware/ON/cf2').resolve()
    expected=json.loads((bundle/'firmware/manifest.json').read_text())['binary_sha256']
    for f in (base/'virtual_aideck/runtime/launcher/launcher.py',base/'virtual_aideck/runtime/host_protocol/e2e_client.py',host,native,firmware):
        if not f.is_file():p.error(f'Missing required file: {f}. Download/extract the materials ZIP and install the original SITL first.')
    if hashlib.sha256(firmware.read_bytes()).hexdigest()!=expected:p.error('Firmware does not match run22 SHA256')
    work=a.work_dir.resolve();work.mkdir(parents=True,exist_ok=False)
    shutil.copytree(bundle/'source',work,dirs_exist_ok=True)
    adapted=[]
    for f in work.rglob('*.py'):
        old=f.read_text()
        # Replace a complete Python string literal, with correct quoting.
        new=old.replace("'/home/razer/Desktop/Drone3'",repr(str(base)))
        if new!=old:f.write_text(new);adapted.append(str(f.relative_to(work)))
    # Viewer loads OpenCV from the selected host environment, if native lacks it.
    paths=json.loads(subprocess.check_output([str(host),'-c','import site,json;print(json.dumps(site.getsitepackages()))'],text=True))
    for f in [work/'mission2_fresh/live.py']:
        old=f.read_text();f.write_text(old.replace("sys.path.append(str(ROOT/'.venv/lib/python3.10/site-packages'))",f'sys.path.extend({paths!r})'))
    firmware.chmod(firmware.stat().st_mode|0o111)
    run=work/'.local/evaluation/replay_run22';run.mkdir(parents=True)
    config=json.loads((bundle/'evidence/run22/runtime_config.json').read_text())
    config['state_link']['firmware_binary']=str(firmware)
    (run/'runtime_config.json').write_text(json.dumps(config,indent=2))
    (work/'PATH_ADAPTATIONS.json').write_text(json.dumps(dict(baseline=str(base),host_python=str(host),native_python=str(native),files=adapted,policy_changes=False),indent=2))
    print(f'Prepared {work}; historical run22 source preserved.',flush=True)
    if a.prepare_only:return
    env=dict(os.environ,MUJOCO_GL='egl',PYTHONDONTWRITEBYTECODE='1')
    runtime=[str(native),'tools/eval_runtime.py']
    try:
        subprocess.run(runtime+['start','--baseline',str(base),'--run-dir',str(run/'runtime'),'--config',str(run/'runtime_config.json'),'--max-wall-s','380','--capture-every','0'],cwd=work,env=env,check=True)
        subprocess.run([str(host),'-m','mission2_fresh.main','--out',str(run/'host'),'--duration','330','--ring-mask','1','--exit-width','1.8','--wall-scan'],cwd=work,env=env,check=True)
    finally:
        subprocess.run(runtime+['stop','--run-dir',str(run/'runtime')],cwd=work,env=env,check=True)
    subprocess.run([str(host),'-m','mission2_fresh.review',str(run)],cwd=work,env=env,check=True)
    print(f'Finished. New evidence: {run}',flush=True)

if __name__=='__main__':main()
