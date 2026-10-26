"""Export received Mono8 + logged judgments and an independent static-M2 REPLAY.

Offline media processing only. Never imports policy, starts SITL, or opens ports.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import cv2
import numpy as np

from eval_timing import pair_frames, source_records, trace_coverage


def sha256(path):
    with Path(path).open('rb') as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def read_rows(path):
    rows, errors = [], []
    if Path(path).exists():
        for number, line in enumerate(Path(path).read_text().splitlines(), 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('expected object')
                rows.append(row)
            except ValueError as exc:
                errors.append(dict(line=number, error=str(exc)))
    return rows, dict(path=str(path), exists=Path(path).exists(), rows=len(rows),
                      malformed_rows=len(errors), errors=errors)


def frame_key(row):
    if not row:
        return None
    fields = ('session_id', 'sequence', 'capture_tick_us', 'received_monotonic')
    return tuple(row.get(k) for k in fields) if all(k in row for k in fields) else None


def diagnostics(decision):
    result = (decision or {}).get('diagnostics') or {}
    return result.get('diagnostics', result)


def completed_input_matches(decision, frame, time=None):
    """Require the complete logged input identity and an actual completion event."""
    key = frame_key(frame)
    completed = (decision or {}).get('policy_completed_monotonic')
    return (key is not None and frame_key((decision or {}).get('frame')) == key
            and type(completed) in (int, float) and math.isfinite(completed)
            and completed >= frame['received_monotonic']
            and (time is None or completed <= time))


def raw_pixel_center(center):
    if (isinstance(center, (list, tuple)) and len(center) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in center)
            and 0 <= center[0] < 324 and 0 <= center[1] < 244):
        return list(center)
    return None


def collision_checks_disabled(details):
    return any(isinstance(details.get(key), dict)
               and details[key].get('collision_checks_disabled') is True
               for key in ('minimal_navigation', 'action_permit', 'foreground_release_action_permit'))


def minimal_navigation_status(decision, frame, time=None):
    """Logged model diagnostics only; never import GT or manufacture scorer passes."""
    details = diagnostics(decision)
    navigation = details.get('minimal_navigation')
    if not isinstance(navigation, dict):
        return None
    aligned = (completed_input_matches(decision, frame, time)
               and navigation.get('source_sequence') == frame['sequence'])
    center = (raw_pixel_center(navigation.get('current_center_px'))
              if aligned and navigation.get('fresh_full_annulus') is True else None)
    return dict(source_sequence=navigation.get('source_sequence'),
        source_matches_displayed_input=aligned,
        fresh_full_annulus=navigation.get('fresh_full_annulus'),
        fresh_metric=navigation.get('fresh_metric'), current_center_px=center,
        model_source_sequence=navigation.get('model_source_sequence'),
        commit_source_sequence=navigation.get('commit_source_sequence'),
        estimated_plane_distance_m=navigation.get('estimated_plane_distance_m'),
        model_cycle_count=details.get('passed_count'), actual_scorer_pass_count=None,
        actual_scorer_pass_status='NOT IMPORTED',
        collision_checks_disabled=collision_checks_disabled(details))


def current_geometry(decision, frame, time=None):
    """Deliberate allowlist: only logged RAW pixel geometry from this exact frame.

    Level-frame boxes, metric proposals, historical failure evidence and GT are
    never projected onto the received image. New formats require explicit support.
    """
    if not completed_input_matches(decision, frame, time):
        return []
    d = diagnostics(decision)
    result = []
    navigation = minimal_navigation_status(decision, frame, time)
    if navigation is not None:
        center = navigation['current_center_px']
        if center is not None:
            result.append(dict(kind='point', center_px=center, label='logged current ring center'))
        # Cached landmarks/commit models and old entry hints are not raw pixels.
        return result
    hint = d.get('entry_ring_hint')
    if isinstance(hint, dict) and hint.get('source_sequence') == frame['sequence']:
        center = raw_pixel_center(hint.get('center_px'))
        if center is not None:
            result.append(dict(kind='point', center_px=center, label='logged ring hint'))
    return result


def load_mono(host, row):
    path = (host/row['image_file']).resolve()
    if not path.is_relative_to(host.resolve()):
        raise ValueError('image path outside trial host directory')
    with np.load(path, allow_pickle=False) as archive:
        image = archive['image']
        if image.dtype != np.uint8 or image.shape != (244, 324):
            raise ValueError('expected native (244,324) uint8 Mono8')
        return image.copy()


def level_geometry_inset(panel, decision, frame):
    """Logged level coordinates on an empty labelled chart, NOT camera pixels."""
    d = diagnostics(decision)
    source = d.get('passage_observation_source') or {}
    if source.get('sequence') != frame['sequence'] or source.get('session_id') != frame['session_id']:
        return
    fences = d.get('current_fences') or []
    if not fences:
        return
    x0, y0, width, height = panel.shape[1]-182, 100, 166, 125
    cv2.rectangle(panel, (x0-5, y0-35), (x0+width+5, y0+height+8), (24, 26, 30), -1)
    cv2.putText(panel, 'LOGGED LEVEL XY', (x0, y0-20), cv2.FONT_HERSHEY_SIMPLEX, .34, (70, 210, 255), 1)
    cv2.putText(panel, 'not camera pixels', (x0, y0-6), cv2.FONT_HERSHEY_SIMPLEX, .34, (200, 210, 220), 1)
    cv2.rectangle(panel, (x0, y0), (x0+width, y0+height), (100, 110, 120), 1)
    def point(x, y):
        return round(x0+max(0, min(324, x))*width/324), round(y0+max(0, min(244, y))*height/244)
    for fence in fences:
        box = fence.get('bbox_level_px')
        if box and len(box) == 4:
            cv2.rectangle(panel, point(*box[:2]), point(*box[2:]), (70, 210, 255), 1)
        for segment in fence.get('segments_level_px') or []:
            if len(segment) == 4:
                cv2.line(panel, point(*segment[:2]), point(*segment[2:]), (70, 160, 210), 1)


class Timeline:
    def __init__(self, frames, decisions, commands, truth):
        self.frames, self.decisions, self.commands, self.truth = frames, decisions, commands, truth
        self.frame_times = [f['received_monotonic'] for f in frames]
        if any(b < a for a, b in zip(self.frame_times, self.frame_times[1:])):
            raise ValueError('receive clock regressed; cannot construct coherent host timeline')
        self.keys = Counter(frame_key(f) for f in frames)
        self.frame_indices = {frame_key(f): i for i, f in enumerate(frames)}
        self.by_frame = defaultdict(list)
        for i, d in enumerate(decisions):
            self.by_frame[frame_key(d.get('frame'))].append(i)
        self.decision_events = sorted((d['policy_completed_monotonic'], i) for i, d in enumerate(decisions)
            if isinstance(d.get('policy_completed_monotonic'), (float, int)))
        self.command_events = sorted((d['sent_monotonic'], i) for i, d in enumerate(commands)
            if isinstance(d.get('sent_monotonic'), (float, int)))
        self.gt_times = [r['host_monotonic'] for r in truth]
        if any(b <= a for a, b in zip(self.gt_times, self.gt_times[1:])):
            raise ValueError('GT host clock regressed or duplicated')
        if any(b['sim_time_s'] <= a['sim_time_s'] for a, b in zip(truth, truth[1:])):
            raise ValueError('GT simulator clock regressed or duplicated')

    @staticmethod
    def latest(events, time):
        index = bisect_right(events, (time, math.inf))-1
        return events[index][1] if index >= 0 else None

    def at(self, time):
        fi = bisect_right(self.frame_times, time)-1
        fi = fi if fi >= 0 else None
        f = self.frames[fi] if fi is not None else None
        indices = self.by_frame.get(frame_key(f), [])
        known = [i for i in indices if self.decisions[i].get('policy_completed_monotonic', math.inf) <= time]
        unique = f is not None and self.keys[frame_key(f)] == 1 and len(indices) <= 1
        di = known[-1] if known and unique else None
        status = ('NO FRAME' if f is None else 'AMBIGUOUS IDENTITY' if not unique
                  else 'AVAILABLE' if di is not None else 'PENDING' if indices else 'NO LOGGED DECISION')
        ci = self.latest(self.command_events, time)
        latest_di = self.latest(self.decision_events, time)
        gi = bisect_right(self.gt_times, time)-1
        return dict(frame_index=fi, decision_index=di, decision_status=status,
            latest_completed_decision_index=latest_di, command_index=ci,
            command_matches_displayed_frame=(ci is not None and f is not None and
                frame_key(self.commands[ci].get('source_frame')) == frame_key(f)),
            gt_index=gi if gi >= 0 else None,
            image_hold_host_s=time-f['received_monotonic'] if f else None,
            gt_hold_host_s=time-self.gt_times[gi] if gi >= 0 else None)

    def presentation(self, sample, time, mode='decision'):
        """Choose a real NPZ for a completed judgment, never move its annotations."""
        if mode == 'decision':
            di = sample['latest_completed_decision_index']
            key = frame_key(self.decisions[di].get('frame')) if di is not None else None
            ambiguous = key is not None and (self.keys[key] > 1 or len(self.by_frame[key]) > 1)
            fi = self.frame_indices.get(key) if key is not None and self.keys[key] == 1 and not ambiguous else None
            status = ('WAITING FOR COMPLETED JUDGMENT' if di is None else
                      'AMBIGUOUS JUDGMENT INPUT' if ambiguous else
                      'UNALIGNED JUDGMENT INPUT' if fi is None else 'COMPLETED JUDGMENT')
        else:
            fi, di = sample['frame_index'], sample['decision_index']
            status = sample['decision_status']
        complete = self.decisions[di].get('policy_completed_monotonic') if di is not None else None
        ci = sample['command_index']
        return dict(displayed_frame_index=fi, displayed_decision_index=di,
            internal_mode=mode, displayed_status=status,
            displayed_input_receipt_age_host_s=time-self.frames[fi]['received_monotonic'] if fi is not None else None,
            displayed_decision_age_host_s=time-complete if complete is not None else None,
            sent_matches_displayed_frame=(ci is not None and fi is not None and
                frame_key(self.commands[ci].get('source_frame')) == frame_key(self.frames[fi])))


def ffmpeg_binary(requested=None):
    if requested:
        return str(Path(requested).resolve())
    found = shutil.which('ffmpeg')
    if found:
        return found
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return None


class Movie:
    def __init__(self, path, size, fps, ffmpeg):
        self.path, self.count, self.process, self.writer = Path(path), 0, None, None
        self.codec = 'mpeg4'
        if ffmpeg:
            encoders = subprocess.run([ffmpeg, '-hide_banner', '-encoders'], capture_output=True,
                                     text=True, check=True).stdout
            self.codec = 'libx264' if 'libx264' in encoders else 'mpeg4'
            self.command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                '-pixel_format', 'bgr24', '-video_size', f'{size[0]}x{size[1]}', '-framerate', str(fps),
                '-i', 'pipe:0', '-an', '-c:v', self.codec, '-pix_fmt', 'yuv420p']
            if self.codec == 'libx264':
                self.command += ['-preset', 'veryfast', '-crf', '21']
            self.command += ['-threads', '2', '-movflags', '+faststart', str(path)]
            self.log = self.path.with_suffix('.encoder.log').open('wb')
            self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stderr=self.log)
        else:
            self.command = None
            self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
            if not self.writer.isOpened():
                raise RuntimeError('no usable MP4 encoder: install imageio-ffmpeg in workspace .venv')

    def write(self, frame):
        if self.process:
            self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        else:
            self.writer.write(frame)
        self.count += 1

    def close(self):
        if self.process:
            self.process.stdin.close()
            code = self.process.wait(timeout=60)
            self.log.close()
            if code:
                raise RuntimeError(f'encoder failed ({code}): {self.path.with_suffix(".encoder.log")}')
        elif self.writer:
            self.writer.release()


def decoded_info(path):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    count, size = 0, None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        size = [frame.shape[1], frame.shape[0]]
        count += 1
    cap.release()
    if not count or not fps:
        raise RuntimeError(f'no decoded video frames: {path}')
    return dict(decoded_frames=count, fps=fps, duration_s=count/fps, size=size,
                fourcc=''.join(chr((fourcc >> (8*i)) & 255) for i in range(4)))


class Replay:
    def __init__(self, args, runtime, truth, out, config, trial_started_monotonic=None):
        baseline = args.baseline
        if baseline is None:
            baseline = Path(json.loads((runtime/'evaluation/provenance.json').read_text())['baseline'])
        python = args.sim_python or baseline/'Drone/bin/python'
        job = dict(baseline=str(baseline.resolve()), runtime_log_dir=str(runtime), truth=truth,
            width=args.width, height=args.image_height,
            trial_started_monotonic=trial_started_monotonic,
            evaluation_setup_present=config.get('evaluation_setup') is not None)
        if args.preflight_assets is not None:
            path = args.preflight_assets.resolve()
            raw = path.read_bytes()
            manifest = json.loads(raw)
            if not isinstance(manifest, dict):
                raise ValueError('preflight assets: manifest must be a JSON object')
            job.update(preflight_assets=manifest, preflight_assets_path=str(path),
                       preflight_assets_sha256=hashlib.sha256(raw).hexdigest())
            (out/'preflight_assets.json').write_bytes(raw)
        write_json(out/'replay_job.json', job)
        self.log = (out/'replay_worker.log').open('wb')
        self.process = subprocess.Popen([str(python), '-B', str(Path(__file__).with_name('mission2_review_media.py')),
            '--job', str(out/'replay_job.json'), '--report', str(out/'replay_validation.json')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
            env=dict(os.environ, MUJOCO_GL='egl', PYTHONDONTWRITEBYTECODE='1', LP_NUM_THREADS='2'))
        self.shape = (args.image_height, args.width, 3)
        self.last = None
        self.pixels = None
        self.marker = None
        if not self.process.stdout.readline():
            self.close()
            lines = (out/'replay_worker.log').read_text(errors='replace').splitlines()
            detail = lines[-1] if lines else 'no worker diagnostics'
            raise RuntimeError(f'replay worker rejected inputs or renderer failed: {detail}; see replay_worker.log')

    def frame(self, index):
        if index != self.last:
            self.process.stdin.write(f'{index}\n'.encode())
            self.process.stdin.flush()
            header = json.loads(self.process.stdout.readline())
            if header['gt_index'] != index:
                raise RuntimeError('replay worker sample identity mismatch')
            self.marker = header.get('marker_px')
            remaining = math.prod(self.shape)
            pieces = []
            while remaining:
                chunk = self.process.stdout.read(remaining)
                if not chunk:
                    raise RuntimeError('replay worker ended; see replay_worker.log')
                pieces.append(chunk)
                remaining -= len(chunk)
            self.pixels = np.frombuffer(b''.join(pieces), np.uint8).reshape(self.shape)[:, :, ::-1].copy()
            self.last = index
        return self.pixels.copy()

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=10)
        self.process.stdout.close()
        self.log.close()


def put_lines(canvas, lines, top, *, color=(220, 230, 235), scale=.47):
    # Measure actual glyphs, so long logged reasons/numeric IDs never clip.
    wrapped = []
    for line in lines:
        remaining = str(line)
        while remaining:
            count = len(remaining)
            while cv2.getTextSize(remaining[:count], cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > canvas.shape[1]-28:
                count -= 1
            if count < len(remaining):
                space = remaining.rfind(' ', 0, count+1)
                if space > count//2:
                    count = space
            wrapped.append(remaining[:count])
            remaining = remaining[count:].lstrip()
    y = top
    for part in wrapped:
        cv2.putText(canvas, part, (12, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += max(19, round(35*scale))
    return y


def number(value, digits=3):
    return 'unknown' if value is None else f'{value:.{digits}f}'


def command_action(command):
    """Readable summary derived from the actual logged command, not flight GT."""
    if not command:
        return 'NO COMMAND'
    reason = command.get('reason', '').lower()
    if command.get('mode') == 'stop':
        return 'STOP'
    if 'brake' in reason:
        return 'BRAKE'
    if any(abs(command.get(k, 0) or 0) > .001 for k in ('vx', 'vy', 'vz')):
        return 'MOVE'
    if abs(command.get('yawrate_deg_s', 0) or 0) > .01:
        return 'MOVE / TURN'
    if any(word in reason for word in ('takeoff', 'landing', 'descend', 'climb')):
        return 'MOVE / HEIGHT'
    return 'HOLD'


def motion_permission(details, frame=None):
    """Motion authority is distinct from the runtime's input/command gate."""
    source = next((key for key in ('action_permit', 'foreground_release_action_permit')
                   if isinstance(details.get(key), dict)), None)
    if source is None:
        return dict(state='NOT LOGGED', reason='not logged', hazard_kind=None, scope=None)
    permit = details[source]
    assessment = permit.get('assessment') or {}
    scope = 'short action' if source == 'action_permit' else 'release candidate'
    sequence = permit.get('source_sequence')
    if frame is not None and sequence is not None and sequence != frame['sequence']:
        state = 'STALE RECORD'
    elif (permit.get('requested') is False or permit.get('status') == 'not_requested'
          or permit.get('reason') in ('not_requested', 'motion_not_requested')):
        state = 'NOT REQUESTED'
    else:
        state = {True: 'ALLOWED', False: 'DENIED'}.get(permit.get('allowed')) if type(permit.get('allowed')) is bool else 'NOT LOGGED'
    return dict(state=state, reason=permit.get('reason', 'not logged'), scope=scope,
        hazard_kind=assessment.get('hazard_kind'), assessment_status=assessment.get('status'),
        blocked=assessment.get('blocked'), source_sequence=sequence, diagnostic_field=source)


def policy_risk(details, frame=None):
    if details.get('root_failure'):
        return 'FAILED', (70, 90, 255)
    if collision_checks_disabled(details):
        return 'COLLISION CHECKS OFF / SIM EXPERIMENT', (70, 205, 255)
    permit = motion_permission(details, frame)
    if permit['state'] == 'DENIED':
        return 'PERMIT DENIED', (70, 90, 255)
    if permit['state'] == 'ALLOWED':
        return 'ALLOWED (local scope)', (130, 235, 150)
    if permit['state'] == 'STALE RECORD':
        return 'UNPROVEN', (70, 205, 255)
    assessment = details.get('foreground_cross_assessment') or details.get('foreground_bounded_assessment') or {}
    if assessment.get('blocked') or assessment.get('status') == 'blocked':
        return 'BLOCKED', (70, 90, 255)
    if assessment.get('clear_to_target') is True:
        return 'LOCAL CLEAR (logged)', (130, 235, 150)
    return 'UNPROVEN', (70, 205, 255)


def internal_panel(args, time, sample, timeline, host, pairs, cache):
    panel = np.full((args.height, args.width, 3), (22, 24, 28), np.uint8)
    fi, di, ci = (sample[k] for k in ('displayed_frame_index', 'displayed_decision_index', 'command_index'))
    frame = timeline.frames[fi] if fi is not None else None
    decision = timeline.decisions[di] if di is not None else None
    d = diagnostics(decision)
    navigation = minimal_navigation_status(decision, frame, time)
    checks_off = collision_checks_disabled(d)
    cmd = (decision or {}).get('command') or {}
    reason = cmd.get('reason') or d.get('reason') or (decision or {}).get('error') or sample['displayed_status']
    if fi is not None:
        if cache.get('index') != fi:
            try:
                cache['image'], cache['error'] = load_mono(host, frame), None
            except (OSError, ValueError, KeyError, EOFError) as exc:
                cache['image'], cache['error'] = None, str(exc)
            cache['index'] = fi
        if cache['image'] is not None:
            rgb = cv2.cvtColor(cache['image'], cv2.COLOR_GRAY2BGR)
            if di is not None:
                for geometry in current_geometry(timeline.decisions[di], frame, time):
                    point = tuple(round(v) for v in geometry['center_px'])
                    cv2.drawMarker(rgb, point, (0, 220, 255), cv2.MARKER_CROSS, 10, 1)
            # Reserve the right-hand strip for the separately labelled geometry
            # chart, including at the smallest supported output width.
            scale = min((args.width-196)/324, (args.image_height-64)/244)
            w, h = round(324*scale), round(244*scale)
            x, y = (args.width-196-w)//2, 64+(args.image_height-64-h)//2
            panel[y:y+h, x:x+w] = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_NEAREST)
        else:
            put_lines(panel, ['PIXELS UNAVAILABLE', cache['error']], 130, color=(80, 90, 255))
    else:
        put_lines(panel, [sample['displayed_status']], 200, color=(80, 200, 255), scale=.7)
    if fi is not None and di is not None and navigation is None:
        level_geometry_inset(panel, decision, timeline.frames[fi])
    # Compact observation facts in the reserved strip, away from raw pixels.
    facts = [f"rings: {d.get('detected_ring_count', 'not logged')}"]
    state = (frame or {}).get('state') or {}
    yaw_fact = []
    if state.get('quaternion_xyzw'):
        x, y, z, w = state['quaternion_xyzw']
        yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        yaw_fact = [f'ego yaw: {yaw:.1f}deg']
    target = d.get('target') or {}
    if navigation is not None:
        center = navigation['current_center_px']
        facts += ['ring center px:', (f'({center[0]:.1f}, {center[1]:.1f})' if center is not None else 'NOT CURRENT')]
        count = navigation['model_cycle_count']
        facts += [f"model cycles: {count if count is not None else 'not logged'}",
                  'scorer passes:', navigation['actual_scorer_pass_status']]
        for key, label in (('fresh_full_annulus', 'full annulus'), ('fresh_metric', 'fresh metric')):
            value = navigation[key]
            text = ('YES' if value is True else 'NO' if value is False else 'NOT LOGGED')
            if not navigation['source_matches_displayed_input']:
                text = 'UNALIGNED'
            facts += [f'{label}: {text}']
        facts += yaw_fact
        facts += [f"model seq: {navigation['model_source_sequence'] if navigation['model_source_sequence'] is not None else 'not logged'}",
                  f"commit seq: {navigation['commit_source_sequence'] if navigation['commit_source_sequence'] is not None else 'not logged'}"]
        if navigation['estimated_plane_distance_m'] is not None:
            facts += [f"est. plane: {number(navigation['estimated_plane_distance_m'], 2)}m"]
    else:
        facts += yaw_fact
        if target.get('uncertainty_m') is not None:
            facts += [f"uncertainty: {target['uncertainty_m']:.2f}m"]
    if cache.get('error') and fi is not None:
        facts += ['pixels unavailable']
    facts_top = 80 if navigation is not None else 275
    visible_facts = max(0, (args.image_height-12-facts_top)//21+1)
    for i, fact in enumerate(facts[:visible_facts]):
        cv2.putText(panel, fact, (args.width-184, facts_top+21*i), cv2.FONT_HERSHEY_SIMPLEX,
                    .37, (190, 210, 220), 1, cv2.LINE_AA)
    risk, risk_color = policy_risk(d, frame)
    permit = motion_permission(d, frame)
    stage = f"{d.get('stage', 'unknown')} / {d.get('phase', sample['displayed_status'])}".upper().replace('-', ' ')
    top = args.image_height+30
    put_lines(panel, [stage], top, scale=.76, color=(240, 240, 240))
    gate = ('ACCEPTED' if decision.get('accepted') else 'REJECTED') if decision else 'PENDING'
    permit_label = permit['state']
    if not checks_off and permit['state'] in ('ALLOWED', 'DENIED'):
        permit_label += f" ({permit['scope']})"
    elif not checks_off and permit['state'] == 'NOT LOGGED':
        permit_label += ' / '+risk
    authority = 'command bounds' if checks_off else 'motion permit'
    y = put_lines(panel, [f'Policy {command_action(cmd)} | input {gate} | {authority} {permit_label}'],
                  top+31, scale=.55, color=risk_color)
    y = put_lines(panel, ['Reason: '+reason], y+7, scale=args.text_scale)
    if permit['state'] != 'NOT LOGGED':
        permit_text = (f"Bounds record: {permit['reason']}" if checks_off else
                       f"Permit: {permit['reason']} | hazard {permit.get('hazard_kind') or 'not logged'} | {permit.get('assessment_status') or 'status not logged'}")
        y = put_lines(panel, [permit_text],
                      y+3, scale=.47, color=risk_color)
    if ci is not None:
        sent = timeline.commands[ci]
        sent_cmd = sent.get('command') or {}
        source = sent.get('source_frame') or {}
        line = (f"SENT {command_action(sent_cmd)}: vx {number(sent_cmd.get('vx'), 2)} vy {number(sent_cmd.get('vy'), 2)} "
                f"yaw {number(sent_cmd.get('yawrate_deg_s'), 1)}deg/s | input #{source.get('sequence')} | {sent.get('verdict', 'unknown')}")
    else:
        line = 'SENT: no command logged yet'
    y = put_lines(panel, [line], y+6, scale=.53)
    latest = timeline.frames[sample['frame_index']]['sequence'] if sample['frame_index'] is not None else '-'
    complete = decision.get('policy_completed_monotonic')-args.origin if decision else None
    details = (f"completed +{number(complete)}s | newest receipt #{latest} | rings {d.get('detected_ring_count', 'not logged')} | "
               f"queue/event drops {args.recording_status.get('frames_dropped', '?')}/{args.recording_status.get('recording', {}).get('events_dropped', '?')}")
    put_lines(panel, [details], y+3, scale=.43, color=(160, 170, 180))
    cv2.rectangle(panel, (0, 0), (args.width, 60), (15, 16, 18), -1)
    badge = ' | SMOKE CLIP' if args.validation_only else ''
    title = 'INTERNAL | COMPLETED JUDGMENT + ITS ACTUAL IMAGE' if args.internal_mode == 'decision' else 'INTERNAL | LATEST RECEIVED IMAGE'
    if checks_off:
        title = 'INTERNAL | COLLISION CHECKS OFF | SIM EXPERIMENT'
    put_lines(panel, [title+badge], 23, color=(100, 240, 180), scale=.57)
    put_lines(panel, [f"+{time-args.origin:.2f}s | input #{frame['sequence'] if frame else '-'} | receipt age {number(sample['displayed_input_receipt_age_host_s'])}s | judgment age {number(sample['displayed_decision_age_host_s'])}s"],
              48, color=(200, 220, 220), scale=.53)
    return panel


def external_panel(args, time, sample, timeline, replay):
    panel = np.full((args.height, args.width, 3), (22, 24, 28), np.uint8)
    gi = sample['gt_index']
    available = replay is not None and gi is not None and sample['gt_hold_host_s'] <= args.max_gt_hold
    if available:
        panel[:args.image_height] = replay.frame(gi)
        if replay.marker is not None:
            x, y = (round(v) for v in replay.marker)
            if 0 <= x < args.width and 65 <= y < args.image_height:
                cv2.circle(panel, (x, y), 16, (40, 215, 255), 2, cv2.LINE_AA)
                cv2.line(panel, (x+15, y-10), (x+42, y-32), (40, 215, 255), 2, cv2.LINE_AA)
                cv2.putText(panel, 'DRONE / GT', (x+45, y-30), cv2.FONT_HERSHEY_SIMPLEX, .55, (40, 215, 255), 2, cv2.LINE_AA)
    cv2.rectangle(panel, (0, 0), (args.width, 60), (15, 16, 18), -1)
    badge = ' | SMOKE CLIP' if args.validation_only else ''
    put_lines(panel, ['EXTERNAL REPLAY | CLOSE FOLLOW'+badge], 23, color=(80, 200, 255), scale=.68)
    put_lines(panel, [f'+{time-args.origin:.2f}s | recorded pose age {number(sample["gt_hold_host_s"])}s | no interpolation'], 48,
        color=(200, 220, 220), scale=.54)
    if not available:
        put_lines(panel, ['EXTERNAL VIEW UNAVAILABLE',
            'No recorded state at this time, excessive GT gap, or replay refused.'], 150, color=(80, 90, 255))
    lines = ['Recorded flight | 1.7m review camera | GT marker identifies drone body']
    if gi is not None:
        row = timeline.truth[gi]
        lines += [f"Height {row['drone']['position_world_m'][2]:.2f}m | recorded contacts {len(row.get('contacts', []))}"]
    ci = sample['command_index']
    if ci is not None:
        sent = timeline.commands[ci]
        cmd = sent.get('command') or {}
        lines += [f"SENT {command_action(cmd)} | {sent.get('verdict', 'unknown')}",
                  'Reason: '+cmd.get('reason', 'not logged')]
    put_lines(panel, lines, args.image_height+32, scale=args.text_scale)
    put_lines(panel, ['Independent post-render; original scene and drone scale. Details: metadata.json'],
              args.height-17, scale=.43, color=(160, 170, 180))
    return panel


def export(args):
    cv2.setNumThreads(1)
    trial_path = args.trial.resolve()
    root, out = trial_path.parent, args.out_dir.resolve()
    if out == root or out.is_relative_to(root/'host') or out.is_relative_to(root/'runtime_logs'):
        raise ValueError('output must not overwrite trial inputs')
    if out.exists() and any(out.iterdir()):
        raise ValueError('out-dir must be empty (preserve existing exports)')
    out.mkdir(parents=True, exist_ok=True)
    trial = json.loads(trial_path.read_text())
    if trial.get('status') == 'running':
        raise ValueError('refusing an active trial; export after recorder closes')
    host = root/'host'
    summary = json.loads((host/'summary.json').read_text()) if (host/'summary.json').exists() else trial.get('host_summary', {})
    args.recording_status = summary.get('status') or {}
    runtime = Path(trial['runtime_log_dir']).resolve()
    if not runtime.is_dir():
        candidates = list((root/'runtime_logs').glob('*'))
        if len(candidates) != 1:
            raise ValueError('runtime_log_dir missing or ambiguous; preserve trial directory layout')
        runtime = candidates[0]
    paths = dict(frames=host/'frames.jsonl', decisions=host/'decisions.jsonl', commands=host/'commands.jsonl',
                 truth=runtime/'evaluation/ground_truth.jsonl', timing=runtime/'evaluation/timing.jsonl')
    rows, parsing = {}, {}
    for name, path in paths.items():
        rows[name], parsing[name] = read_rows(path)
    frames, decisions, commands = (rows[n] for n in ('frames', 'decisions', 'commands'))
    truth = [r for r in rows['truth'] if r.get('event') == 'ground_truth']
    if not frames:
        raise ValueError('trial contains no recorded received camera frames')
    timeline = Timeline(frames, decisions, commands, truth)
    coverage = trace_coverage(rows['timing'], dict(malformed_rows=parsing['timing']['malformed_rows'], row_limit_hit=False))
    pairs = pair_frames(frames, source_records(rows['timing']), lambda f: load_mono(host, f).tobytes(),
        same_host_monotonic=True, source_coverage_complete=coverage['complete_capture_prefix_through_wrapper_close'])
    origin = trial.get('started_monotonic', frames[0]['received_monotonic'])
    end = trial.get('finished_monotonic', trial.get('host_exit_monotonic', frames[-1]['received_monotonic']))
    if not math.isfinite(origin) or not math.isfinite(end) or end <= origin:
        raise ValueError('invalid trial host interval')
    args.origin = origin
    begin = origin+args.start
    stop = min(end, begin+args.duration) if args.duration is not None else end
    if begin >= stop:
        raise ValueError('requested clip lies outside trial interval')
    count = math.ceil((stop-begin)*args.fps-1e-9)
    inputs = {str(trial_path): sha256(trial_path)}
    for p in [*paths.values(), root/'prelaunch_provenance.json', runtime/'evaluation/provenance.json',
              runtime/'evaluation/model_metadata.json', runtime/'arena/scene.xml', runtime/'arena/arena_manifest.json',
              runtime/'config_snapshot.json', host/'observations.jsonl', host/'summary.json']:
        if p.is_file():
            inputs[str(p)] = sha256(p)
    pixel_inventory = []
    for i, frame in enumerate(frames):
        p = (host/frame.get('image_file', '')).resolve()
        valid = p.is_relative_to(host.resolve()) and p.is_file()
        pixel_inventory.append(dict(frame_index=i, path=str(p), file_sha256=sha256(p) if valid else None,
                                    source_pair=pairs[i]))
    write_json(out/'frames_provenance.json', pixel_inventory)
    write_json(out/'decisions.json', decisions)
    write_json(out/'commands.json', commands)
    ffmpeg = ffmpeg_binary(args.ffmpeg)
    replay, external_error = None, None
    if args.external != 'none':
        try:
            if not truth or parsing['truth']['malformed_rows']:
                raise ValueError('missing or malformed GT log')
            config = json.loads((runtime/'config_snapshot.json').read_text())
            replay = Replay(args, runtime, truth, out, config, trial.get('started_monotonic'))
        except (OSError, ValueError, RuntimeError) as exc:
            external_error = str(exc)
            if args.external == 'required':
                raise
    if args.preflight_assets is not None and (out/'preflight_assets.json').is_file():
        inputs[str(args.preflight_assets.resolve())] = sha256(out/'preflight_assets.json')
    movies, cache = {}, {}
    samples = []
    posters = {round(q*(count-1)) for q in np.linspace(0, 1, min(count, 12))}
    movie_paths = ['internal', 'external']+(['side_by_side'] if not args.no_side_by_side else [])
    try:
        for name in movie_paths:
            size = (args.width*(2 if name == 'side_by_side' else 1), args.height)
            movies[name] = Movie(out/f'{name}.mp4', size, args.fps, ffmpeg)
        with (out/'timeline.jsonl').open('w') as audit:
            for i in range(count):
                time = begin+i/args.fps
                sample = timeline.at(time)
                sample.update(timeline.presentation(sample, time, args.internal_mode))
                shown_di, shown_fi = sample['displayed_decision_index'], sample['displayed_frame_index']
                sample['input_accepted'] = decisions[shown_di].get('accepted') if shown_di is not None else None
                shown_decision = decisions[shown_di] if shown_di is not None else None
                shown_frame = frames[shown_fi] if shown_fi is not None else None
                sample['minimal_navigation'] = minimal_navigation_status(shown_decision, shown_frame, time)
                sample['collision_checks_disabled'] = collision_checks_disabled(diagnostics(shown_decision))
                sample['motion_permission'] = motion_permission(
                    diagnostics(decisions[shown_di]) if shown_di is not None else {},
                    frames[shown_fi] if shown_fi is not None else None)
                sample.update(output_index=i, pts_s=i/args.fps, host_monotonic=time,
                    trial_relative_host_s=time-origin,
                    external_state_visible=(replay is not None and sample['gt_index'] is not None
                                           and sample['gt_hold_host_s'] <= args.max_gt_hold))
                fi = sample['displayed_frame_index']
                sample['pixels_status'] = ('no_frame' if fi is None else
                    'unavailable' if pairs[fi]['status'] == 'pixels_unavailable' else 'retained_Mono8')
                internal = internal_panel(args, time, sample, timeline, host, pairs, cache)
                external = external_panel(args, time, sample, timeline, replay)
                sample['external_marker_px'] = replay.marker if sample['external_state_visible'] else None
                audit.write(json.dumps(sample, allow_nan=False)+'\n')
                movies['internal'].write(internal)
                movies['external'].write(external)
                side = np.hstack((external, internal))
                if 'side_by_side' in movies:
                    movies['side_by_side'].write(side)
                if i in posters:
                    samples.append(cv2.resize(side, (768, round(768*args.height/(2*args.width)))))
                if i == round((count-1)/2):
                    cv2.imwrite(str(out/'poster.png'), side)
                if i % max(1, round(args.fps*10)) == 0:
                    print(f'rendered {i}/{count} frames, host +{time-origin:.2f}s', file=sys.stderr, flush=True)
    finally:
        for movie in movies.values():
            movie.close()
        if replay:
            replay.close()
    while len(samples) % 3:
        samples.append(np.zeros_like(samples[0]))
    contact = np.vstack([np.hstack(samples[i:i+3]) for i in range(0, len(samples), 3)])
    cv2.imwrite(str(out/'contactsheet.png'), contact)
    decoded = {}
    for name, movie in movies.items():
        info = decoded_info(movie.path)
        if info['decoded_frames'] != count or abs(info['duration_s']-count/args.fps) > 1e-3:
            raise RuntimeError(f'decoded count/duration mismatch: {name}: {info}')
        decoded[name] = dict(info, codec=movie.codec, sha256=sha256(movie.path), encoder_command=movie.command)
    available = {i for i, f in enumerate(frames) if pairs[i]['status'] != 'pixels_unavailable'}
    unaligned = [i for i, d in enumerate(decisions) if timeline.keys[frame_key(d.get('frame'))] != 1]
    metadata = dict(schema=1, kind='mission2_offline_review_media', trial=str(trial_path),
        trial_status=trial.get('status'), trial_evidence=trial.get('evidence'), powered_success_claim=False,
        validation_only=args.validation_only, label='OFFLINE TOOLING SMOKE - EXISTING TRIAL' if args.validation_only else 'RECORDED TRIAL REVIEW',
        policy_execution=False, physics_execution=False, ports_opened=False,
        full_trial_host_interval=[origin, end], exported_host_interval=[begin, stop],
        full_trial_export=args.start == 0 and stop == end, requested_start_s=args.start,
        internal_mode=args.internal_mode,
        requested_duration_s=args.duration, frames=count, fps=args.fps, duration_s=count/args.fps,
        alignment=dict(master='recorded Linux host monotonic seconds',
            rationale='trial supervisor, host receiver/policy/sender and evaluation source record time.monotonic() on same launcher host',
            uncertainty='host identity inferred from per-trial paths and native launch provenance; kernel boot ID not retained',
            image_presentation=('latest completed judgment and its EXACT recorded input NPZ, held until next completed judgment; before first judgment blank'
                if args.internal_mode == 'decision' else 'latest retained image by actual received_monotonic, before first receipt blank'),
            temporal_quantization_bound_s=1/args.fps,
            decisions='exact session/sequence/GAP8-capture/host-receipt tuple, only after policy_completed_monotonic',
            displayed_fields='displayed_frame_index/displayed_decision_index describe the visible matched pair; frame_index/decision_index retain latest-receipt diagnostics independently',
            commands='latest actual sent_monotonic; source-frame match displayed independently of policy acceptance',
            external='preceding recorded GT host timestamp, no synthetic interpolation; blank if hold exceeds max_gt_hold_s',
            source_identity='unique raw pixel SHA256 + session from timing trace; never source/host sequence equality',
            independent_clocks=['GAP8 capture_tick_us', 'STM32 snapshot_tick_ms', 'source sim_time_s'],
            independent_clock_mapping='none; raw hash identity links source sim stamp without equating clock rates',
            max_gt_hold_s=args.max_gt_hold),
        source_trace_coverage=coverage, source_pair_counts=dict(Counter(p['status'] for p in pairs)),
        counts=dict(received_frames=len(frames), pixels_available=len(available), logged_decisions=len(decisions),
            logged_commands=len(commands), gt_rows=len(truth), unaligned_decisions=len(unaligned),
            received_frames_without_decision=sum(not timeline.by_frame.get(frame_key(f)) for f in frames)),
        unaligned_decision_indices=unaligned, parsing=parsing, input_sha256=inputs,
        recording_status=args.recording_status,
        external_status='REPLAY' if replay else 'UNAVAILABLE', external_error=external_error,
        replay_validation='replay_validation.json' if replay else None,
        preflight_assets_path=str(args.preflight_assets.resolve()) if args.preflight_assets is not None else None,
        versions=dict(python=sys.version, numpy=np.__version__, opencv=cv2.__version__), ffmpeg=ffmpeg,
        tooling_sha256={p.name: sha256(p) for p in (Path(__file__), Path(__file__).with_name('mission2_review_media.py'), Path(__file__).with_name('eval_timing.py'))},
        decoded=decoded, limitations=[
            'CFR preserves the host trial timeline; decision mode repeats each real input image until the next completed judgment; receipt mode repeats by actual arrivals',
            'short-lived decisions/images can fall between output samples; source rows are retained in sidecars, not fabricated into frames',
            'raw crosshairs require the exact completed input identity; minimal_navigation.current_center_px also requires fresh_full_annulus=true and matching source_sequence, otherwise only legacy current entry hints are supported',
            'minimal-navigation passed_count is labelled model cycles; independent scorer passes are NOT IMPORTED and collision clearance is never inferred',
            'level-frame fence geometry, when its observation identity matches, is shown on a separate labelled coordinate inset, never projected onto camera pixels',
            'NPZ recording is the host policy-consumed subset; latest-only queue drops may leave unretained arrivals, reported by whole-run counters',
            'logged ring detection count absent means NOT LOGGED, not zero; missing decisions remain explicit',
            'independent external render is reconstructed GT kinematics, not new flight or policy evidence',
            'video does not replace independent scoring or prove flight success'])
    metadata['presentation'] = dict(
        action_labels='MOVE/HOLD/BRAKE/STOP derived only from logged command fields/reason, not measured flight movement',
        risk_labels='FAILED from root_failure; explicit collision_checks_disabled shows COLLISION CHECKS OFF / SIM EXPERIMENT before any permit/legacy clear status; otherwise current action/release permit DENIED or ALLOWED for its scope, legacy foreground assessment or UNPROVEN',
        minimal_navigation='current ring center requires fresh_full_annulus=true and exact completed input/source identity; cached model, reference ray, estimated plane and commit are never projected onto raw pixels',
        minimal_counts='logged passed_count means model cycles in minimal mode; actual scorer pass count is unknown / NOT IMPORTED, never inferred from model cycles, completion, GT or image exit',
        input_acceptance='decision.accepted is runtime input/command gate acceptance, never motion permission; absent permit is NOT LOGGED, not implicit permission',
        external_marker='recorded drone GT origin projected with actual free-camera frustum; external-only, may mark an occluded body',
        original_image='no annotation is moved onto a different frame; all detailed clocks, quaternions, provenance and raw diagnostic rows remain in sidecars')
    metadata['sidecar_sha256'] = {p.name: sha256(p) for p in out.iterdir() if p.is_file() and p.suffix in ('.json', '.jsonl', '.png')}
    write_json(out/'metadata.json', metadata)
    print(json.dumps(dict(out_dir=str(out), movies=list(decoded), frames=count,
                          duration_s=count/args.fps, external_status=metadata['external_status'])))
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--external', choices=('auto', 'required', 'none'), default='auto')
    parser.add_argument('--internal-mode', choices=('decision', 'received'), default='decision',
                        help='decision: latest completed judgment and its exact NPZ input (default); received: latest retained arrival')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--sim-python', type=Path)
    parser.add_argument('--preflight-assets', type=Path,
                        help='optional pretrial JSON: baseline, captured_monotonic, sha256 absolute asset-path map')
    parser.add_argument('--ffmpeg', type=Path)
    parser.add_argument('--fps', type=float, default=30.)
    parser.add_argument('--start', type=float, default=0., help='explicit clip start, seconds after trial.started_monotonic')
    parser.add_argument('--duration', type=float, help='explicit clip duration; default whole remaining trial')
    parser.add_argument('--validation-only', action='store_true', help='identify export of an existing trial as tooling validation')
    parser.add_argument('--no-side-by-side', action='store_true')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--image-height', type=int, default=560)
    parser.add_argument('--height', type=int, default=800)
    parser.add_argument('--text-scale', type=float, default=.59)
    parser.add_argument('--max-gt-hold', type=float, default=.25)
    args = parser.parse_args(argv)
    if (not 1 <= args.fps <= 120 or not math.isfinite(args.start) or args.start < 0
            or args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0)
            or args.width < 640 or args.width % 2 or args.image_height < 244 or args.image_height % 2
            or args.height < args.image_height+240 or args.height % 2
            or not math.isfinite(args.max_gt_hold) or args.max_gt_hold <= 0):
        parser.error('invalid fps, clip interval, panel dimensions (even, width>=640, text area>=240), or GT hold limit')
    try:
        export(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'review export failed: {exc}\n')


if __name__ == '__main__':
    main()
