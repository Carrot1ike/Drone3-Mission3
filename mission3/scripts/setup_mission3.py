#!/usr/bin/env python3
"""기존 팀 설치를 읽고 프로젝트 안에만 미션3 환경을 만듭니다."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT/'config/mission3-bootstrap.json').read_text())
ENV = {**os.environ, 'PYTHONNOUSERSITE': '1', 'GIT_OPTIONAL_LOCKS': '0'}
BUILDERS = ('build_mission3_firmware_host.py', 'build_mission3_flight_controller.py',
            'build_mission3_position_bridge.py')


def run(argv, capture=False):
    return subprocess.run([str(x) for x in argv], check=True, env=ENV,
                          text=True, capture_output=capture, cwd=ROOT).stdout


def git(path, *args):
    return run(['git', '-C', path, *args], capture=True).strip()


def validate_shared(path):
    path = Path(path).expanduser().resolve()
    if not (path/'manifests/upstream_sources.yaml').is_file():
        raise ValueError('--shared-root는 설치를 마친 crazyflie-ros2-sil 폴더여야 합니다')
    interpreter = path/'.venv/bin/python'
    if not interpreter.is_file():
        raise ValueError('팀 저장소 .venv가 없습니다. 팀 설치의 setup_python.sh를 먼저 실행하세요')
    version = json.loads(run([interpreter, '-c',
        'import sys,json;print(json.dumps(list(sys.version_info[:2])))'], capture=True))
    if version != [3, 10]:
        raise ValueError(f'검증 대상 Python은 3.10입니다. 현재: {version}')
    cmsis = path/'crazyflie-firmware/vendor/CMSIS'
    if not (cmsis/'CMSIS/DSP/Include/arm_math.h').is_file():
        raise ValueError('CMSIS가 없습니다. 팀 저장소 setup_sources.sh의 재귀 소스를 확인하세요')
    if git(cmsis, 'rev-parse', 'HEAD') != SPEC['cmsis_commit']:
        raise ValueError('CMSIS 커밋이 미션3 고정 버전과 다릅니다. 기존 팀 소스를 덮어쓰지 않습니다')
    if git(cmsis, 'status', '--porcelain', '--untracked-files=no'):
        raise ValueError('CMSIS 추적 파일에 변경이 있습니다. 별도의 깨끗한 팀 설치를 지정하세요')
    return path, interpreter, git(path, 'rev-parse', 'HEAD')


def link_shared(path):
    destination = ROOT/'src/crazyflie-ros2-sil'
    if destination.exists() or destination.is_symlink():
        if destination.resolve() != path:
            raise ValueError('src/crazyflie-ros2-sil이 다른 설치를 가리킵니다. 자동 교체하지 않습니다')
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.symlink_to(path, target_is_directory=True)


def prepare_firmware():
    destination = ROOT/'src/bitcraze-mission3-firmware'
    if destination.exists() or destination.is_symlink():
        if git(destination, 'rev-parse', 'HEAD') != SPEC['firmware_commit']:
            raise ValueError('기존 제조사 소스 커밋이 다릅니다. 자동 reset하지 않습니다')
        if git(destination, 'status', '--porcelain', '--untracked-files=no'):
            raise ValueError('기존 제조사 소스가 수정되어 있습니다. 원본을 보존하고 별도 설치하세요')
        return
    # 지정 커밋만 읽어 새 로컬 checkout을 만듭니다. 외부 저장소에는 쓰지 않습니다.
    with tempfile.TemporaryDirectory(prefix='.firmware-', dir=destination.parent) as temp:
        run(['git', 'init', '-q', temp])
        run(['git', '-C', temp, 'remote', 'add', 'origin', SPEC['firmware_repository']])
        run(['git', '-C', temp, 'fetch', '--depth', '1', 'origin', SPEC['firmware_commit']])
        run(['git', '-C', temp, 'checkout', '--detach', '-q', 'FETCH_HEAD'])
        if git(temp, 'rev-parse', 'HEAD') != SPEC['firmware_commit']:
            raise ValueError('제조사 소스 커밋 검증 실패')
        Path(temp).rename(destination)


def prepare_environment(interpreter):
    folder = ROOT/'.venv-mission3'
    marker = folder/'.mission3-owned.json'
    if folder.exists() or folder.is_symlink():
        if folder.is_symlink() or not marker.is_file():
            raise ValueError('.venv-mission3가 기존 다른 환경입니다. 자동 변경하지 않습니다')
        if json.loads(marker.read_text()).get('root') != str(ROOT):
            raise ValueError('복사/이동된 가상환경입니다. 새 경로에서 다시 생성해야 합니다')
    else:
        run([interpreter, '-m', 'venv', folder])
        marker.write_text(json.dumps({'root': str(ROOT), 'python_minor': '3.10'})+'\n')
    python = folder/'bin/python'
    run([python, '-m', 'pip', '--require-virtualenv', 'install', '--only-binary=:all:',
         '-r', ROOT/'requirements/mission3-lock.txt'])
    run([python, '-m', 'pip', 'check'])
    return python


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shared-root', type=Path,
                        default=ROOT/'src/crazyflie-ros2-sil')
    args = parser.parse_args()
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        parser.error('현재 검증 범위는 Linux x86_64 / Python 3.10입니다')
    for command in ('git', 'gcc'):
        if shutil.which(command) is None:
            parser.error(f'{command}가 없습니다. 팀 환경의 시스템 설치 단계를 먼저 완료하세요')
    try:
        shared, interpreter, team_commit = validate_shared(args.shared_root)
        if team_commit != SPEC['observed_team_commit']:
            print('팀 저장소 HEAD가 관측 버전과 다릅니다. 사용되는 CMSIS 버전은 일치합니다.', flush=True)
        link_shared(shared)
        prepare_firmware()
        python = prepare_environment(interpreter)
        for builder in BUILDERS:
            run([python, ROOT/'scripts'/builder])
        receipt = ROOT/'build/mission3-setup.json'
        receipt.write_text(json.dumps({
            'created_utc': datetime.now(timezone.utc).isoformat(), 'root': str(ROOT),
            'shared_root': str(shared), 'team_commit': team_commit,
            'firmware_commit': SPEC['firmware_commit'], 'cmsis_commit': SPEC['cmsis_commit'],
            'requirements_sha256': hashlib.sha256((ROOT/'requirements/mission3-lock.txt').read_bytes()).hexdigest(),
            'libraries': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted((ROOT/'build').glob('mission3-*/*.so'))},
            'compiler': run(['gcc', '--version'], capture=True).splitlines()[0],
            'shared_environment_modified': False,
            'scope': 'local dependencies/build only; not flight or hardware approval'
        }, ensure_ascii=False, indent=2)+'\n')
        run([python, ROOT/'scripts/check_mission3_environment.py'])
        print('설치·환경 검사 완료. bash scripts/run_mission3_automation.sh 로 실행하세요.', flush=True)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f'미션3 설치 중단: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
