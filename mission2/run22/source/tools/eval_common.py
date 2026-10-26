"""Evaluation-only utilities. Never import from mission_host policy code."""
from __future__ import annotations
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import sys
import types
import zlib

WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = Path('/home/razer/Desktop/Drone3')
LOCAL = WORKSPACE / '.local' / 'evaluation'


def local_path(path):
    path = Path(path).resolve()
    if not path.is_relative_to(WORKSPACE / '.local'):
        raise ValueError('evaluation output must be inside workspace .local')
    return path


def write_json(path, value):
    path = local_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    os.replace(tmp, path)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def baseline_modules(baseline):
    baseline = Path(baseline).resolve()
    runtime = baseline / 'virtual_aideck/runtime'
    sim = baseline / 'crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco'
    for path in (runtime, runtime / 'camera_input', sim):
        sys.path.insert(0, str(path))
    import crazysim
    return crazysim, runtime, sim


def workspace_builder():
    """Clone only the builder's output-root guard; all source/assets stay baseline."""
    from arena import model_builder
    context = dict(model_builder.build.__globals__, ROOT=WORKSPACE)
    return types.FunctionType(model_builder.build.__code__, context, 'build',
                              model_builder.build.__defaults__)


def provenance(baseline):
    baseline = Path(baseline).resolve()
    files = ['virtual_aideck/runtime/camera_input/source.py',
             'virtual_aideck/runtime/launcher/launcher.py',
             'virtual_aideck/runtime/arena/model_builder.py',
             'virtual_aideck/runtime/arena/rover_adapter.py',
             'virtual_aideck/runtime/arena/vendor/sim_final/world.xml',
             'virtual_aideck/runtime/arena/vendor/sim_final/assets/id0.png',
             'crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py']
    return {'baseline': str(baseline), 'sha256': {
        name: hashlib.sha256((baseline / name).read_bytes()).hexdigest() for name in files}}


def write_png(path, gray):
    """Native Mono8 PNG, without requiring OpenCV/Pillow in baseline Python."""
    height, width = gray.shape
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))
    rows = b''.join(b'\x00' + row.tobytes() for row in gray)
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))
    path = local_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)
    return hashlib.sha256(png).hexdigest()


def process_identity(pid):
    try:
        fields = Path(f'/proc/{int(pid)}/stat').read_text().split(') ', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, ValueError):
        return None
