#!/usr/bin/env python3
"""의존성·호스트 라이브러리·렌더·영상 인코딩 검사. 비행 시험은 시작하지 않습니다."""
import argparse
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MUJOCO_GL', 'egl')


def check(quick=False):
    if sys.version_info[:2] != (3, 10) or platform.system() != 'Linux':
        raise ValueError('검증 대상은 Linux / Python 3.10입니다')
    versions = {}
    for line in (ROOT/'requirements/mission3-lock.txt').read_text().splitlines():
        if not line or line.startswith('#'): continue
        name, expected = line.split('==')
        actual = metadata.version(name)
        if actual != expected:
            raise ValueError(f'{name}: {actual}, 필요 {expected}. setup_mission3.sh를 다시 실행하세요')
        versions[name] = actual
    libraries = {}
    for folder in ('mission3-firmware-host', 'mission3-flight-controller', 'mission3-position-bridge'):
        manifest = ROOT/'build'/folder/'source-manifest.json'
        if not manifest.is_file():
            raise ValueError(f'{folder} 빌드가 없습니다. setup_mission3.sh를 실행하세요')
        data = json.loads(manifest.read_text())
        if data.get('exit_code', 0) != 0:
            raise ValueError(f'{folder} 컴파일 실패: build/{folder}/compile.log 확인')
        for key in ('source_sha256', 'header_sha256'):
            for name, expected in data.get(key, {}).items():
                if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
                    raise ValueError(f'빌드 뒤 소스가 변경되었습니다: {name}. 다시 빌드하세요')
        paths = list((ROOT/'build'/folder).glob('*.so'))
        if len(paths) != 1: raise ValueError(f'{folder} 공유 라이브러리가 없습니다')
        libraries[str(paths[0].relative_to(ROOT))] = hashlib.sha256(paths[0].read_bytes()).hexdigest()
    receipt = ROOT/'build/mission3-setup.json'
    if receipt.is_file():
        built = json.loads(receipt.read_text())
        if built['libraries'] != libraries:
            raise ValueError('설치 영수증과 호스트 라이브러리 SHA가 다릅니다. 다시 설치하세요')
        lock_sha = hashlib.sha256((ROOT/'requirements/mission3-lock.txt').read_bytes()).hexdigest()
        if built['requirements_sha256'] != lock_sha:
            raise ValueError('설치 뒤 의존성 목록이 바뀌었습니다. 다시 설치하세요')
    from mission3.firmware import FirmwareCore
    from mission3.flight_controller import StockPid
    from mission3.position_bridge import PCPositionPid
    FirmwareCore().state()
    StockPid()
    PCPositionPid((0., 0., 1.)).update((0., 0., 1.), (0., 0., 0.), 0., (0., 0., 0.))
    report = {'status': 'pass', 'python': sys.version, 'interpreter': sys.executable,
              'system': platform.platform(), 'versions': versions, 'libraries': libraries,
              'backend': os.environ['MUJOCO_GL'], 'quick': quick, 'flight_trials_started': 0}
    if quick: return report
    import cv2
    import numpy as np
    import mujoco
    import imageio_ffmpeg
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    mono = np.pad(cv2.aruco.generateImageMarker(dictionary, 7, 200), 60, constant_values=255)
    _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(mono)
    if ids is None or ids.ravel().tolist() != [7]: raise ValueError('ArUco 인식 검사 실패')
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    with mujoco.Renderer(model, height=320, width=320) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
    if pixels.shape != (320, 320, 3): raise ValueError('MuJoCo 렌더 크기 검사 실패')
    with tempfile.TemporaryDirectory(prefix='mission3-check-') as temp:
        avi = Path(temp)/'mono.avi'
        writer = cv2.VideoWriter(str(avi), cv2.VideoWriter_fourcc(*'FFV1'), 20., (320, 320), False)
        if not writer.isOpened(): raise ValueError('OpenCV FFV1 기록 지원이 없습니다')
        writer.write(mono); writer.release()
        cap = cv2.VideoCapture(str(avi)); ok, decoded = cap.read(); cap.release()
        if not ok or not np.array_equal(decoded[:, :, 0], mono): raise ValueError('mono 무손실 기록 검사 실패')
        proc = subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-y', '-i', str(avi),
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(Path(temp)/'preview.mp4')],
            capture_output=True, text=True)
        if proc.returncode: raise ValueError('H264 미리보기 인코딩 실패: '+proc.stderr[-1000:])
    report.update(aruco=True, renderer_shape=list(pixels.shape), mono_ffv1_exact=True, h264=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    try:
        report = check(args.quick)
    except Exception as exc:
        print(f'환경 검사 실패: {exc}\n설치 안내: docs/mission3/teammate-setup.md', file=sys.stderr)
        return 1
    payload = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True); args.out.write_text(payload)
    if args.quick:
        print(f'미션3 환경 확인: Python 3.10 / 호스트 C 3종 / {report["backend"]}')
    else:
        print(payload)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
