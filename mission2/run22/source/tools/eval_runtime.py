"""Bounded evaluation runtime supervisor; uses baseline native transport unchanged."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import types
import uuid

from eval_common import (DEFAULT_BASELINE, LOCAL, WORKSPACE, load_module, local_path,
                         process_identity, provenance, write_json)


def source_substitution(command, baseline, eval_dir, gt_hz, capture_every):
    command = [str(part) for part in command]
    target = str(Path(baseline).resolve() / 'virtual_aideck/runtime/camera_input/source.py')
    if len(command) > 1 and command[1] == target:
        # --log is unique per baseline supervisor run, and avoids overwrite across restarts.
        log_dir = Path(command[command.index('--log') + 1]).parent
        command[1:2] = [str(WORKSPACE / 'tools/eval_source.py'), '--baseline', str(baseline),
                       '--eval-dir', str(log_dir / 'evaluation'), '--gt-hz', str(gt_hz),
                       '--capture-every', str(capture_every)]
    return command


def read_state(directory):
    try:
        return json.loads((directory / 'runtime_state/state.json').read_text())
    except (OSError, ValueError):
        return {}


def alive(state):
    return bool(state.get('pid') and state.get('start_ticks') and
                process_identity(state['pid']) == state['start_ticks'])


def status(directory):
    state = read_state(directory)
    state['supervisor_alive'] = alive(state)
    try:
        state['evaluation'] = json.loads((directory / 'run.json').read_text())
    except (OSError, ValueError):
        pass
    print(json.dumps(state, indent=2))
    return state


def stop(directory):
    state = read_state(directory)
    if not alive(state):
        # During import/validation startup the wrapper identity is still available.
        try:
            state = json.loads((directory / 'owner.json').read_text())
        except (OSError, ValueError):
            return 0
    if alive(state):
        os.kill(state['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 35
        while alive(state) and time.monotonic() < deadline:
            time.sleep(.1)
        if alive(state):
            print('Owned supervisor shutdown timeout; inspect child identities in state.json', file=sys.stderr)
            return 1
    status(directory)
    return 0


def serve(args, directory):
    write_json(directory / 'owner.json', dict(pid=os.getpid(), start_ticks=process_identity(os.getpid())))
    runtime = args.baseline / 'virtual_aideck/runtime'
    baseline = load_module('_evaluation_baseline_launcher', runtime / 'launcher/launcher.py')
    # Resolve this exact dependency even if a caller imported stdlib profile first.
    load_module('profile', runtime / 'state_link/profile.py')
    real_popen = subprocess.Popen
    def popen(command, *positional, **kwargs):
        return real_popen(source_substitution(command, args.baseline, directory,
                          args.gt_hz, args.capture_every), *positional, **kwargs)
    # Use a module-local proxy; changing subprocess.Popen globally also changes our controller.
    baseline.subprocess = types.SimpleNamespace(**dict(vars(subprocess), Popen=popen))
    expiration = time.monotonic() + args.max_wall_s
    def deadline():
        while time.monotonic() < expiration:
            time.sleep(min(.2, max(0, expiration - time.monotonic())))
        write_json(directory / 'deadline.json', dict(reason='wall_time_limit',
                    host_monotonic=time.monotonic(), powered_success=False))
        os.kill(os.getpid(), signal.SIGTERM)
    threading.Thread(target=deadline, daemon=True).start()
    sys.argv = [str(runtime / 'launcher/launcher.py'), 'foreground', '--config',
                str(directory / 'config.json'), '--gui' if args.gui else '--headless']
    return baseline.main()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['start', 'foreground', 'status', 'stop', 'test', '_serve'])
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--run-dir', type=Path, default=LOCAL / 'current')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--max-wall-s', type=float, default=180.)
    parser.add_argument('--startup-wait-s', type=float, default=5.)
    parser.add_argument('--gt-hz', type=float, default=50.)
    parser.add_argument('--capture-every', type=int, default=20)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    directory = local_path(args.run_dir)
    args.baseline = args.baseline.resolve()
    if args.command == 'test':
        # No simulator creation, native ports, firmware or arming.
        return subprocess.call([sys.executable, '-m', 'pytest', '-q',
                                *[str(path) for path in sorted((WORKSPACE/'tests').glob('test_evaluation*.py'))]], cwd=WORKSPACE)
    if args.command == 'status':
        status(directory)
        return 0
    if args.command == 'stop':
        return stop(directory)
    if not 1 <= args.max_wall_s <= 3600 or not 1 <= args.gt_hz <= 1000 or args.capture_every < 0:
        parser.error('max-wall-s must be 1..3600, gt-hz 1..1000, capture-every >=0')
    if args.command == '_serve':
        return serve(args, directory)
    if alive(read_state(directory)):
        parser.error('this evaluation run is already alive')
    owner_path = directory / 'owner.json'
    if owner_path.exists() and alive(json.loads(owner_path.read_text())):
        parser.error('evaluation wrapper is still alive')
    if args.command in ('start', 'foreground'):
        config_path = args.config or args.baseline / 'virtual_aideck/config/arena_mission3_straight.json'
        config = json.loads(config_path.read_text())
        if (config.get('cpx_tcp_port'), config.get('crtp_udp_port'), config.get('firmware_udp_port')) != (5000, 20050, 20150):
            parser.error('evaluation is restricted to native TCP5000 / loopback CRTP UDP20050')
        config.update(log_dir=str(directory / 'runtime_logs'), state_dir=str(directory / 'runtime_state'))
        write_json(directory / 'config.json', config)
        write_json(directory / 'run.json', dict(evidence='native_runtime_unassessed',
            token=uuid.uuid4().hex, host_monotonic=time.monotonic(), max_wall_s=args.max_wall_s,
            policy_input='TCP127.0.0.1:5000 image+state; cflib udp://127.0.0.1:20050',
            gt_input='evaluation only; never consumed by policy', provenance=provenance(args.baseline)))
        if args.command == 'foreground':
            return serve(args, directory)
        command = [sys.executable, str(Path(__file__).resolve()), '_serve', '--baseline', str(args.baseline),
                   '--run-dir', str(directory), '--max-wall-s', str(args.max_wall_s),
                   '--gt-hz', str(args.gt_hz), '--capture-every', str(args.capture_every)]
        if args.gui:
            command.append('--gui')
        with (directory / 'supervisor.log').open('a') as log:
            process = subprocess.Popen(command, cwd=WORKSPACE, start_new_session=True,
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + max(0, min(30., args.startup_wait_s))
        while process.poll() is None and time.monotonic() < deadline:
            if read_state(directory).get('status') == 'running':
                break
            time.sleep(.1)
        status(directory)
        return 0 if process.poll() is None else process.returncode or 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
