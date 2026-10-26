"""Opt-in evaluation sensor/image conditions. Never import from policy code.

Installation changes Python adapters in memory only. No simulator, socket, or flight
is started here. See docs/MISSION2_CONDITIONS.md for the deliberately narrow contract.
"""
from __future__ import annotations

import copy
from collections import Counter
import hashlib
import json
import math
import marshal
from pathlib import Path
import struct
import sys
import types

import numpy as np

from eval_common import local_path


def _fields(value, expected, where):
    if not isinstance(value, dict) or set(value) != set(expected.split()):
        raise ValueError(f'{where}: expected exactly {expected}')


def _number(value, low, high, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{where}: expected finite number in [{low}, {high}]')


def _vector(value, size, low, high, where):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'{where}: expected {size} numbers')
    for item in value:
        _number(item, low, high, where)


def resolve_conditions(raw):
    """Validate a standalone config; no implicit activation or ignored fields."""
    if raw is None:
        return None
    if isinstance(raw, (str, Path)):
        raw = json.loads(Path(raw).read_text())
    _fields(raw, 'schema name scenario enabled seed calibration_status assumptions parameter_ranges support_limitations imu tof flow image', 'conditions')
    if type(raw['schema']) is not int or raw['schema'] != 1 or raw['scenario'] != 'mission2':
        raise ValueError('conditions require schema 1 and scenario mission2')
    if type(raw['enabled']) is not bool or type(raw['seed']) is not int or not 0 <= raw['seed'] < 2**64:
        raise ValueError('enabled must be boolean; seed must be uint64')
    if not isinstance(raw['name'], str) or not raw['name'].strip():
        raise ValueError('conditions need an explicit name')
    if raw['calibration_status'] != 'uncalibrated_engineering_hypotheses':
        raise ValueError('these conditions are uncalibrated engineering hypotheses')
    for key in ('assumptions', 'parameter_ranges', 'support_limitations'):
        if not isinstance(raw[key], list) or not raw[key] or any(not isinstance(v, str) or not v.strip() for v in raw[key]):
            raise ValueError(f'{key} must explicitly describe assumptions/ranges/limitations')
    imu, tof, flow, image = (raw[k] for k in ('imu', 'tof', 'flow', 'image'))
    _fields(imu, 'acc_bias_uniform_mps2 acc_noise_std_mps2 gyro_bias_uniform_radps gyro_noise_std_radps', 'imu')
    _vector(imu['acc_bias_uniform_mps2'], 3, 0, .5, 'acc bias half-width')
    _vector(imu['gyro_bias_uniform_radps'], 3, 0, .05, 'gyro bias half-width')
    _number(imu['acc_noise_std_mps2'], 0, .5, 'acc noise')
    _number(imu['gyro_noise_std_radps'], 0, .05, 'gyro noise')
    _fields(tof, 'bias_m noise_std_m outlier_probability outlier_offset_m dropout_probability dropout_mode', 'tof')
    for key, low, high in [('bias_m', -.2, .2), ('noise_std_m', 0, .1), ('outlier_probability', 0, .1), ('outlier_offset_m', -1, 1), ('dropout_probability', 0, .2)]:
        _number(tof[key], low, high, f'tof.{key}')
    _fields(flow, 'bias_px scale noise_std_px burst_start_probability burst_length_samples dropout_mode', 'flow')
    _vector(flow['bias_px'], 2, -2, 2, 'flow bias')
    _vector(flow['scale'], 2, .8, 1.2, 'flow scale')
    _number(flow['noise_std_px'], 0, 2, 'flow noise')
    _number(flow['burst_start_probability'], 0, .1, 'flow burst probability')
    lengths = flow['burst_length_samples']
    if not isinstance(lengths, list) or len(lengths) != 2 or any(type(v) is not int for v in lengths) or not 1 <= lengths[0] <= lengths[1] <= 20:
        raise ValueError('flow burst lengths must be ordered integers in 1..20')
    for sensor in (tof, flow):
        if sensor['dropout_mode'] != 'omit_packet':
            raise ValueError('only omit_packet dropout is supported; no validity/quality field or fake zero')
    _fields(image, 'brightness_gain noise_std_dn blur_sigma_px', 'image')
    for key, low, high in [('brightness_gain', .5, 1.5), ('noise_std_dn', 0, 20), ('blur_sigma_px', 0, 2)]:
        _number(image[key], low, high, f'image.{key}')
    return copy.deepcopy(raw)


class _NumpyView:
    """Delegate geometry to NumPy; redirect only baseline flow's random calls."""
    def __init__(self, rng):
        self.random = types.SimpleNamespace(normal=rng.normal)

    def __getattr__(self, name):
        return getattr(np, name)


class _AgentView:
    """Dispatch-local socket view; never swap the agent's shared/threaded socket."""
    def __init__(self, agent, sock):
        object.__setattr__(self, '_agent', agent)
        object.__setattr__(self, '_sock', sock)

    def __getattr__(self, name):
        return getattr(self._agent, name)

    def __setattr__(self, name, value):
        setattr(self._agent, name, value)


class _SensorSocket:
    def __init__(self, sock, conditions):
        self._sock, self._conditions = sock, conditions

    def sendto(self, packet, destination):
        transformed = self._conditions.sensor_packet(packet)
        if transformed is None:
            return len(packet)  # Deliberately withheld; no fabricated measurement.
        return self._sock.sendto(transformed, destination)

    def __getattr__(self, name):
        return getattr(self._sock, name)


class Conditions:
    """One single-agent evaluation run; counters are attempted adapter events."""
    def __init__(self, source, config, output):
        self.config = config
        self.output = local_path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.counts = Counter(dict.fromkeys((
            'dispatch_calls', 'imu_samples', 'tof_samples', 'tof_omitted',
            'tof_outliers', 'tof_input_invalid', 'tof_output_invalid',
            'flow_samples', 'flow_bursts', 'flow_omitted', 'image_frames',
            'image_clipped_pixels'), 0))
        self._closed = False
        self._patches = []
        self._burst_remaining = 0
        self._rngs = {}
        self.acc_bias = self.rng('imu_bias').uniform(-np.array(config['imu']['acc_bias_uniform_mps2']), config['imu']['acc_bias_uniform_mps2'])
        self.gyro_bias = self.rng('imu_bias').uniform(-np.array(config['imu']['gyro_bias_uniform_radps']), config['imu']['gyro_bias_uniform_radps'])
        canonical = json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False)
        self.config_hash = hashlib.sha256(canonical.encode()).hexdigest()
        # Exclusive creation prevents accidental overwrite of an earlier run.
        self._log = (self.output / 'conditions.jsonl').open('x', buffering=1)
        self._record('installed', config=config, config_sha256=self.config_hash,
                     seed=config['seed'], numpy_version=np.__version__, python_version=sys.version,
                     rng='PCG64; SHA256(seed:stream)',
                     implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                     baseline_flow_code_sha256=hashlib.sha256(marshal.dumps(source.crazysim.DroneAgent.compute_flow.__code__)).hexdigest(),
                     realized_bias=dict(acc_mps2=self.acc_bias.tolist(), gyro_radps=self.gyro_bias.tolist()),
                     counter_meaning='attempted adapter events, not confirmed firmware receipt',
                     clocks='unchanged; no RTF correction; image transform adds host computation',
                     image_model='post-render approximation; no exposure integration or rolling shutter')

    def rng(self, stream):
        if stream not in self._rngs:
            digest = hashlib.sha256(f'{self.config["seed"]}:{stream}'.encode()).digest()
            self._rngs[stream] = np.random.Generator(np.random.PCG64(int.from_bytes(digest, 'big')))
        return self._rngs[stream]

    def _record(self, event, **extra):
        row = dict(event=event, evaluation_only=True, policy_may_read=False,
                   counters=dict(self.counts), **extra)
        self._log.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')

    def imu(self, acc, gyro):
        self.counts['imu_samples'] += 1
        cfg, rng = self.config['imu'], self.rng('imu_noise')
        return (np.asarray(acc) + self.acc_bias + rng.normal(0, cfg['acc_noise_std_mps2'], 3),
                np.asarray(gyro) + self.gyro_bias + rng.normal(0, cfg['gyro_noise_std_radps'], 3))

    def sensor_packet(self, packet):
        if packet[:2] == b'\x65\x08':
            raise ValueError('conditions forbid external truth-pose packets; require flowdeck')
        if packet[:2] == b'\x90\x05':
            if len(packet) != 6:
                raise ValueError('unsupported ToF packet layout')
            self.counts['tof_samples'] += 1
            cfg, rng = self.config['tof'], self.rng('tof')
            distance, = struct.unpack_from('<f', packet, 2)
            if not math.isfinite(distance):
                raise ValueError('nonfinite baseline ToF')
            if rng.random() < cfg['dropout_probability']:
                self.counts['tof_omitted'] += 1
                self._record('tof_omitted')
                return None
            # Match the firmware's strict gate. Do not turn an invalid input valid.
            if not float(np.float32(.01)) < distance < 5.0:
                self.counts['tof_input_invalid'] += 1
                return packet
            distance += cfg['bias_m'] + rng.normal(0, cfg['noise_std_m'])
            if rng.random() < cfg['outlier_probability']:
                distance += cfg['outlier_offset_m']
                self.counts['tof_outliers'] += 1
                self._record('tof_outlier')
            result = b'\x90\x05' + struct.pack('<f', distance)
            # Account for float32 rounding at validity boundaries; never clamp to 0.
            realized, = struct.unpack_from('<f', result, 2)
            if not float(np.float32(.01)) < realized < 5.0:
                self.counts['tof_output_invalid'] += 1
                self.counts['tof_omitted'] += 1
                # Firmware rangeSet casts to uint16 *before* its EKF gate.
                # Withhold out-of-gate perturbations instead of sending negatives
                # or inventing a validity flag/sentinel on this protocol.
                self._record('tof_output_invalid_omitted')
                return None
            return result
        if packet[:2] == b'\x90\x06':
            if len(packet) != 14:
                raise ValueError('unsupported flow packet layout')
            self.counts['flow_samples'] += 1
            cfg, rng = self.config['flow'], self.rng('flow')
            x, y, dt = struct.unpack_from('<fff', packet, 2)
            if not all(map(math.isfinite, (x, y, dt))) or dt <= 0:
                raise ValueError('invalid baseline flow measurement')
            # Burst includes its starting sample. Surviving samples retain native dt.
            if not self._burst_remaining and rng.random() < cfg['burst_start_probability']:
                self._burst_remaining = int(rng.integers(cfg['burst_length_samples'][0], cfg['burst_length_samples'][1] + 1))
                self.counts['flow_bursts'] += 1
                self._record('flow_burst_start', length_samples=self._burst_remaining)
            if self._burst_remaining:
                self._burst_remaining -= 1
                self.counts['flow_omitted'] += 1
                self._record('flow_omitted')
                return None
            values = np.array([x, y]) * cfg['scale'] + cfg['bias_px'] + rng.normal(0, cfg['noise_std_px'], 2)
            return b'\x90\x06' + struct.pack('<fff', *values, dt)
        return packet

    def image(self, gray):
        if gray.dtype != np.uint8 or gray.shape != (244, 324):
            raise ValueError('only native 324x244 MONO8 images supported')
        self.counts['image_frames'] += 1
        cfg = self.config['image']
        values = gray.astype(np.float64) * cfg['brightness_gain']
        if cfg['blur_sigma_px']:
            # Separable Gaussian, reflect edges; spatial post-render approximation.
            sigma = cfg['blur_sigma_px']
            radius = math.ceil(3 * sigma)
            # This form also handles tiny positive sigma without sigma**2 == 0.
            with np.errstate(over='ignore', under='ignore'):
                kernel = np.exp(-.5 * (np.arange(-radius, radius + 1, dtype=float) / sigma)**2)
            kernel /= kernel.sum()
            for axis in (0, 1):
                pads = [(0, 0), (0, 0)]
                pads[axis] = (radius, radius)
                values = np.apply_along_axis(lambda v: np.convolve(v, kernel, mode='valid'), axis,
                                             np.pad(values, pads, mode='reflect'))
        values += self.rng('image').normal(0, cfg['noise_std_dn'], gray.shape)
        self.counts['image_clipped_pixels'] += int(np.count_nonzero((values < 0) | (values > 255)))
        return np.rint(np.clip(values, 0, 255)).astype(np.uint8)

    def _patch(self, owner, name, replacement):
        original = getattr(owner, name)
        setattr(owner, name, replacement)
        self._patches.append((owner, name, original, replacement))

    def close(self):
        """Call after source.main() stops; restores only hooks still owned by us."""
        if self._closed:
            return
        self._record('closed', complete=True)
        for owner, name, original, replacement in reversed(self._patches):
            if getattr(owner, name) is replacement:
                setattr(owner, name, original)
        self._log.close()
        self._closed = True


def install_conditions(source, config=None, output=None):
    """Install before source.main(), after GT/capture installation; return handle.

    None or enabled:false is a no-op (including no output writes). The caller must
    enforce official Mission2 scene selection. This hook never reads scene/GT data.
    """
    config = resolve_conditions(config)
    if config is None or not config['enabled']:
        return None
    if output is None:
        raise ValueError('enabled conditions require an evaluation output directory')
    agent_class = source.crazysim.DroneAgent
    if getattr(agent_class.send_sensor_data, '_evaluation_conditions', False):
        raise ValueError('conditions already installed')
    original_imu = agent_class.read_imu
    original_flow = agent_class.compute_flow
    original_send = agent_class.send_sensor_data
    original_socket = source.FrameSocket
    if not isinstance(original_flow, types.FunctionType) or 'np' not in original_flow.__globals__:
        raise ValueError('unsupported baseline compute_flow; requires Python/NumPy adapter')
    handle = Conditions(source, config, output)
    # Clone the *actual* flow implementation, preserving geometry and native noise.
    # Never reseed global NumPy: other simulator threads remain independent.
    flow_globals = dict(original_flow.__globals__, np=_NumpyView(handle.rng('baseline_flow')))
    seeded_flow = types.FunctionType(original_flow.__code__, flow_globals, original_flow.__name__,
                                     original_flow.__defaults__, original_flow.__closure__)

    def check(agent):
        if agent.agent_id != 0 or not agent._flowdeck or agent._noise_model is not None:
            raise ValueError('conditions require agent 0, flowdeck, and no competing noise_model')
        if handle._closed:
            raise RuntimeError('conditions handle closed')

    def read_imu(agent):
        check(agent)
        return handle.imu(*original_imu(agent))

    def compute_flow(agent):
        check(agent)
        return seeded_flow(agent)

    def send_sensor_data(agent):
        check(agent)
        handle.counts['dispatch_calls'] += 1
        result = original_send(_AgentView(agent, _SensorSocket(agent._sock, handle)))
        if handle.counts['dispatch_calls'] % 1000 == 0:
            handle._record('checkpoint')
        return result

    send_sensor_data._evaluation_conditions = True
    class ConditionedFrameSocket(original_socket):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._condition_parts = []

        def sendto(self, packet, destination):
            index, total, width, height = struct.unpack_from('<HHHH', packet)
            if (total, width, height) != (2, 324, 244) or index != len(self._condition_parts):
                raise ValueError('unsupported camera chunk sequence/layout')
            self._condition_parts.append((bytes(packet), destination))
            if index == 0:
                return len(packet)
            parts, self._condition_parts = self._condition_parts, []
            raw = b''.join(p[8:] for p, _ in parts)
            if len(raw) != width * height:
                raise ValueError('incorrect camera payload size')
            pixels = handle.image(np.frombuffer(raw, dtype=np.uint8).reshape(height, width)).tobytes()
            offset = 0
            for original, dest in parts:
                size = len(original) - 8
                super().sendto(original[:8] + pixels[offset:offset + size], dest)
                offset += size
            handle._record('image_transformed', source_sequence=self.seq - 1,
                           source_sim_ns=self.sim_ns,
                           input_sha256=hashlib.sha256(raw).hexdigest(),
                           output_sha256=hashlib.sha256(pixels).hexdigest())
            return len(packet)

    handle._patch(agent_class, 'read_imu', read_imu)
    handle._patch(agent_class, 'compute_flow', compute_flow)
    handle._patch(agent_class, 'send_sensor_data', send_sensor_data)
    handle._patch(source, 'FrameSocket', ConditionedFrameSocket)
    return handle
