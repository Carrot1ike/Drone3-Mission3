"""Read-only camera identity and clock-rate diagnostics; never imported by policy.

Source-side BoundedCaptureTrace is opt-in. CLI analysis reads retained Mono8 pixels,
not PNG container hashes or host/GAP8 sequence-number equality.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import time
import uuid


def session_id(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def statistics(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return dict(count=0)
    def quantile(q):
        index = (len(values)-1)*q
        lower = int(index)
        return values[lower] + (values[min(lower+1, len(values)-1)]-values[lower])*(index-lower)
    return dict(count=len(values), min=values[0], median=quantile(.5), p95=quantile(.95),
                max=values[-1], mean=sum(values)/len(values))


class BoundedCaptureTrace:
    """A bounded prefix/thinned trace, separate from GT and native rotating logs.

    No pixel/history retention. At most max_frames hashes and fixed-size records.
    Buffered synchronous writes can still stall; this is not a real-time guarantee.
    """
    def __init__(self, path, *, max_frames, every=1, clock=time.monotonic):
        if (type(max_frames) is not int or type(every) is not int
                or not 1 <= max_frames <= 100000 or not 1 <= every <= 100000):
            raise ValueError('timing max_frames/every must be in 1..100000')
        self.clock = clock
        self.stream = Path(path).open('w', buffering=65536)
        self.trace_id = uuid.uuid4().hex
        self.max_frames, self.every = max_frames, every
        self.seen = self.written = self.thinned = self.capped = 0
        self.overhead_total_s = self.overhead_max_s = 0.
        self.closed = False
        self._write(dict(event='timing_start', schema=1, evaluation_only=True,
                         trace_id=self.trace_id, max_frames=max_frames, every=every,
                         host_monotonic=self.clock()))

    def _write(self, row):
        self.stream.write(json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n')

    def selected(self):
        """Called exactly once for each complete source frame, before transport."""
        ordinal = self.seen
        self.seen += 1
        if ordinal % self.every:
            self.thinned += 1
            return False
        if self.written >= self.max_frames:
            self.capped += 1
            return False
        return True

    def capture(self, raw, *, session, sequence, sim_ns, started, ready):
        if self.closed or self.written >= self.max_frames:
            raise ValueError('capture outside open trace capacity')
        before = self.clock()
        row = dict(event='camera_formed', schema=1, evaluation_only=True,
                   trace_id=self.trace_id, session=session_id(session), source_sequence=sequence,
                   sim_time_ns=sim_ns, width=324, height=244, raw_bytes=len(raw),
                   raw_sha256=hashlib.sha256(raw).hexdigest(),
                   host_monotonic=ready, pixels_ready_host_monotonic=ready,
                   render_call_begin_host_monotonic=started,
                   evidence='synchronous_pixels_before_native_transport')
        self._write(row)
        self.written += 1
        elapsed = self.clock()-before
        self.overhead_total_s += elapsed
        self.overhead_max_s = max(self.overhead_max_s, elapsed)

    def status(self):
        return dict(trace_id=self.trace_id, frames_seen=self.seen, records_written=self.written,
                    thinned_frames=self.thinned, capped_frames=self.capped,
                    max_frames=self.max_frames, every=self.every,
                    hash_and_buffer_write_total_host_s=self.overhead_total_s,
                    hash_and_buffer_write_max_host_s=self.overhead_max_s)

    def close(self):
        if self.closed:
            return
        self._write(dict(event='timing_close', schema=1, evaluation_only=True,
                         host_monotonic=self.clock(), **self.status()))
        self.stream.close()
        self.closed = True


def read_jsonl(path, *, max_rows=1000000):
    """Keep parsed prefix and explicitly report malformed/truncated input."""
    rows, malformed, limited = [], 0, False
    with Path(path).open() as stream:
        for line in stream:
            if not line.strip():
                continue
            if len(rows) >= max_rows:
                limited = True
                break
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('record is not an object')
                rows.append(row)
            except (ValueError, TypeError):
                malformed += 1
    return rows, dict(path=str(path), rows=len(rows), malformed_rows=malformed, row_limit_hit=limited)


def source_records(rows):
    """Normalize raw hashes only: legacy camera_capture.sha256 is a PNG hash."""
    output = []
    for row in rows:
        event = row.get('event')
        if event == 'source' and row.get('sha256'):
            output.append(dict(session=session_id(row['session']), sequence=row['seq'],
                digest=row['sha256'], sim_s=row['sim_ns']/1e9,
                host=row['host_monotonic'], kind='legacy_post_transport',
                render_and_pack_s=row.get('render_and_pack_s'), trace_id=None))
        elif event == 'camera_formed' and row.get('raw_sha256'):
            output.append(dict(session=session_id(row['session']), sequence=row['source_sequence'],
                digest=row['raw_sha256'], sim_s=row['sim_time_ns']/1e9,
                host=row['pixels_ready_host_monotonic'], kind='pre_transport',
                started=row.get('render_call_begin_host_monotonic'), trace_id=row.get('trace_id')))
    return output


def trace_coverage(rows, parse_status):
    starts = [r for r in rows if r.get('event') == 'timing_start']
    closes = [r for r in rows if r.get('event') == 'timing_close']
    captures = [r for r in rows if r.get('event') == 'camera_formed']
    complete = False
    if len(starts) == len(closes) == 1:
        start, close = starts[0], closes[0]
        progression = all(a['session'] == b['session']
                          and a['source_sequence']+1 == b['source_sequence']
                          and a['sim_time_ns'] < b['sim_time_ns']
                          and a['pixels_ready_host_monotonic'] < b['pixels_ready_host_monotonic']
                          for a, b in zip(captures, captures[1:]))
        complete = (start['trace_id'] == close['trace_id']
                    and all(r.get('trace_id') == start['trace_id'] for r in captures)
                    and close['frames_seen'] == close['records_written'] == len(captures)
                    and close['thinned_frames'] == close['capped_frames'] == 0
                    and close['every'] == 1 and rows[0] == start and rows[-1] == close
                    and progression and (not captures or captures[0]['source_sequence'] == 0)
                    and not parse_status['malformed_rows'] and not parse_status['row_limit_hit'])
    return dict(complete_capture_prefix_through_wrapper_close=complete,
                start_records=len(starts), close_records=len(closes), capture_records=len(captures),
                counters=closes[-1] if closes else None,
                limitation='observed wrapper lifetime only; not crash, pre-wrapper, or external physics coverage')


def pair_frames(frames, sources, load_pixels, *, same_host_monotonic=False,
                source_coverage_complete=False):
    """Only a globally unique recorded raw hash is a candidate identity.

    No nearest-receipt tie breaker, interpolation, or source/host sequence equality.
    A partial trace supports observed candidates, not an exhaustive age bound.
    """
    by_hash = defaultdict(list)
    identities = Counter((s['session'], s.get('trace_id'), s['sequence']) for s in sources)
    host_identities = Counter((f['session_id'], f['sequence'], f['capture_tick_us']) for f in frames)
    for source in sources:
        by_hash[source['session'], source['digest']].append(source)
    matches = []
    for index, frame in enumerate(frames):
        row = dict(frame_index=index, session_id=frame['session_id'], sequence=frame['sequence'],
                   capture_tick_us=frame['capture_tick_us'], received_monotonic=frame['received_monotonic'])
        if host_identities[frame['session_id'], frame['sequence'], frame['capture_tick_us']] != 1:
            row['status'] = 'duplicate_host_identity'
            matches.append(row)
            continue
        try:
            raw = load_pixels(frame)
            if len(raw) != 324*244:
                raise ValueError('expected 79056 received Mono8 bytes')
            digest = hashlib.sha256(raw).hexdigest()
        except (OSError, ValueError, KeyError, EOFError) as exc:
            row.update(status='pixels_unavailable', error=str(exc))
            matches.append(row)
            continue
        candidates = by_hash[session_id(frame['session_id']), digest]
        row.update(raw_sha256=digest, candidate_count=len(candidates),
                   candidate_set_exhaustive=source_coverage_complete)
        causal = [s for s in candidates if s['host'] <= frame['received_monotonic']]
        if same_host_monotonic and causal:
            ages = [frame['received_monotonic']-s['host'] for s in causal]
            row['observed_candidate_endpoint_age_interval_host_s'] = [min(ages), max(ages)]
            row['age_interval_is_exhaustive'] = source_coverage_complete
        if not candidates:
            row['status'] = 'unmatched'
        elif len(candidates) != 1:
            row['status'] = 'ambiguous_hash'
        else:
            source = candidates[0]
            identity = (source['session'], source.get('trace_id'), source['sequence'])
            if identities[identity] != 1:
                row['status'] = 'duplicate_source_identity'
            elif same_host_monotonic and not causal:
                row['status'] = 'noncausal_clock_or_trace'
            else:
                qualified = (source_coverage_complete and same_host_monotonic
                             and source['kind'] == 'pre_transport')
                row.update(status='qualified_unique' if qualified else 'unique_observed_hash',
                           source_sequence=source['sequence'], source_sim_time_s=source['sim_s'],
                           source_endpoint=source['kind'], source_host_monotonic=source['host'])
                if same_host_monotonic:
                    age = frame['received_monotonic']-source['host']
                    row['source_endpoint_to_receipt_host_s'] = age
                    if source['kind'] == 'pre_transport':
                        started = source.get('started')
                        if started is not None and 0 <= started <= source['host']:
                            row['render_call_to_receipt_age_bracket_host_s'] = [
                                age, frame['received_monotonic']-started]
                        sent = source.get('native_post_send_host_monotonic')
                        if sent is not None:
                            row['pixels_ready_to_native_post_send_host_s'] = sent-source['host']
                            row['native_post_send_to_receipt_host_s'] = frame['received_monotonic']-sent
                    elif source.get('render_and_pack_s') is not None:
                        # Native timestamps use separate monotonic() calls. This includes
                        # conversion/packing/send work; not a measured exposure instant.
                        row['approx_render_call_to_receipt_host_s'] = age+source['render_and_pack_s']
        matches.append(row)
    return matches


def attach_native_transport(sources, native_rows):
    """Join source-side records by source identity AND raw hash; never host seq."""
    native = defaultdict(list)
    for row in source_records(native_rows):
        if row['kind'] == 'legacy_post_transport':
            native[row['session'], row['sequence'], row['digest']].append(row)
    attached = []
    for source in sources:
        item = dict(source)
        candidates = native[source['session'], source['sequence'], source['digest']]
        if source['kind'] == 'pre_transport' and len(candidates) == 1 and candidates[0]['host'] >= source['host']:
            item['native_post_send_host_monotonic'] = candidates[0]['host']
        attached.append(item)
    return attached


def clock_windows(samples, *, window_s=1., max_gap_s=.5):
    """Rate from observed deltas within each identity/monotonic segment.

    samples: (identity, host monotonic, local seconds, optional sequence).
    Gaps are excluded rather than bridged; offsets never fitted or subtracted.
    """
    counts = Counter()
    windows, active, previous = [], None, None
    segment = 0
    def finish():
        if active is not None:
            elapsed = active['host_end']-active['host_start']
            windows.append(dict(**active, host_elapsed_s=elapsed,
                                local_per_host_rate=active['local_elapsed_s']/elapsed))
    for identity, host, local, sequence in samples:
        current = (identity, host, local, sequence)
        counts['samples'] += 1
        if previous is None:
            previous = current
            continue
        pi, ph, pl, ps = previous
        dh, dl = host-ph, local-pl
        reason = None
        if identity != pi:
            reason = 'identity_changes'
        elif dh <= 0:
            reason = 'nonmonotonic_host'
        elif dl < 0:
            reason = 'local_clock_resets_or_reorders'
        elif sequence is not None and ps is not None and sequence <= ps:
            reason = 'duplicate_or_backward_sequences'
        elif dh > max_gap_s:
            reason = 'host_gaps'
        if reason:
            counts[reason] += 1
            finish()
            active = None
            segment += 1
            previous = current
            continue
        if sequence is not None and ps is not None and sequence > ps+1:
            counts['missing_sequence_values'] += sequence-ps-1
        if dl == 0:
            counts['repeated_local_ticks'] += 1
        if active is not None and host-active['host_start'] > window_s:
            finish()
            active = None
        if active is None:
            active = dict(segment=segment, identity=list(identity), host_start=ph, host_end=host,
                          local_elapsed_s=0., intervals=0)
        active['host_end'] = host
        active['local_elapsed_s'] += dl
        active['intervals'] += 1
        previous = current
    finish()
    return dict(counts=dict(counts), windows=windows,
                rates=statistics(w['local_per_host_rate'] for w in windows),
                window_host_s=window_s, max_included_gap_host_s=max_gap_s)


def lifecycle_report(rows, lifecycle=None):
    truth = [r for r in rows if r.get('event') == 'ground_truth']
    sample_indices = [r.get('sample_index') for r in truth]
    indices_ok = bool(truth) and sample_indices == list(range(len(truth)))
    monotonic = all(a['session'] == b['session'] and a['host_monotonic'] < b['host_monotonic']
                    and a['sim_time_s'] < b['sim_time_s'] for a, b in zip(truth, truth[1:]))
    counts_ok = bool(lifecycle and lifecycle.get('gt_rows') == len(truth))
    return dict(gt_rows=len(truth), contiguous_sample_indices=indices_ok,
                strictly_monotonic_retained_truth=monotonic if truth else None,
                lifecycle=lifecycle, row_count_agrees=counts_ok,
                wrapper_close_observed=bool(lifecycle and lifecycle.get('wrapper_closed')),
                qualification='observed wrapper evidence only; no guaranteed crash or whole-flight coverage')


def raw_timing_statistics(frames):
    """Summarize only recorder-whitelisted raw integers; never subtract clocks.

    Counts include all retained frames, irrespective of association validity.
    Missing/null historical fields remain missing; malformed values are counted,
    not coerced into ages or included in statistics.
    """
    fields = [
        ('association', 'gap8_state_receive_tick_us', 64, 'us'),
        ('association', 'state_receive_age_at_frame_us', 64, 'us'),
        ('association', 'receiver_sequence', 64, 'sequence'),
        ('stm32', 'state_sequence', 64, 'sequence'),
        ('stm32', 'clock_domain_id', 32, 'domain_id'),
        ('stm32', 'imu_receive_tick', 64, 'ms'),
        ('stm32', 'tof_receive_tick', 64, 'ms'),
        ('stm32', 'imu_sequence', 32, 'sequence'),
        ('stm32', 'tof_sequence', 32, 'sequence'),
        ('stm32', 'imu_age_ms', 32, 'ms'),
        ('stm32', 'tof_age_ms', 32, 'ms'),
    ]
    result = dict(association={}, stm32={})
    for group, name, bits, unit in fields:
        values, missing, invalid = [], 0, 0
        for frame in frames:
            container = frame if group == 'association' else frame.get('state') or {}
            value = container.get(name)
            if value is None:
                missing += 1
            elif type(value) is not int or not 0 <= value < 1 << bits:
                invalid += 1
            else:
                values.append(value)
        result[group][name] = dict(statistics(values), missing_count=missing,
                                  invalid_count=invalid, unit=unit)
    return result


def analyze(source_rows, frames, truth, load_pixels, *, same_host_monotonic=False,
            source_coverage_complete=False, window_s=1., max_gap_s=.5, lifecycle=None,
            native_source_rows=(), hook_status=None):
    sources = attach_native_transport(source_records(source_rows), native_source_rows)
    matches = pair_frames(frames, sources, load_pixels,
                          same_host_monotonic=same_host_monotonic,
                          source_coverage_complete=source_coverage_complete)
    physics = [((session_id(r['session']),), r['host_monotonic'], r['sim_time_s'],
                r.get('sample_index')) for r in truth if r.get('event') == 'ground_truth']
    capture = [((session_id(f['session_id']),), f['received_monotonic'], f['capture_tick_us']/1e6,
                f['sequence']) for f in frames]
    snapshot = []
    missing_state = invalid_state = 0
    for frame in frames:
        state = frame.get('state')
        if not state:
            missing_state += 1
        elif not frame.get('association_valid') or not state.get('state_valid'):
            invalid_state += 1
        else:
            snapshot.append(((session_id(frame['session_id']), state['boot_id'], state['reset_generation']),
                frame['received_monotonic'], state['snapshot_tick_ms']/1000, frame['sequence']))
    rates = {name: clock_windows(samples, window_s=window_s, max_gap_s=max_gap_s)
             for name, samples in [('physics_seconds_at_dispatch', physics),
                                   ('gap8_seconds_at_host_receipt', capture),
                                   ('stm32_seconds_at_host_receipt', snapshot)]}
    source_rates = [((s['session'], s.get('trace_id')), s['host'], s['sim_s'], s['sequence']) for s in sources]
    rates['source_physics_seconds_at_camera_endpoint'] = clock_windows(
        source_rates, window_s=window_s, max_gap_s=max_gap_s)
    cpu = [((session_id(r['session']),), r['host_monotonic'], r['process_cpu_s'],
            r.get('sample_index')) for r in truth
           if r.get('event') == 'ground_truth' and 'process_cpu_s' in r]
    rates['source_process_cpu_seconds_at_dispatch'] = clock_windows(
        cpu, window_s=window_s, max_gap_s=max_gap_s)
    load_bins = defaultdict(list)
    if sources:
        anchor = sources[0]['host']
        for source in sources:
            duration = source.get('render_and_pack_s')
            if duration is not None and math.isfinite(duration) and duration >= 0:
                load_bins[(source['session'], math.floor((source['host']-anchor)/window_s))].append(duration)
    pacing_snapshot = (hook_status or {}).get('pacing_final_snapshot') or {}
    infrastructure_failure = bool((lifecycle or {}).get('infrastructure_failure')
                                  or pacing_snapshot.get('infrastructure_failure'))
    return dict(schema=1, evaluation_only=True,
        verdict='INVALID_INFRASTRUCTURE' if infrastructure_failure else 'DIAGNOSTIC_ONLY',
        optional_source_hooks=hook_status if hook_status is not None else dict(recorded=False,
            limitation='no retained hook status; absence does not prove adapters were disabled'),
        host_monotonic_comparison_asserted=same_host_monotonic,
        source_hash_records=len(sources), frames=len(frames),
        matches_by_status=dict(Counter(m['status'] for m in matches)), matches=matches,
        endpoint_to_receipt_host_s=statistics(m['source_endpoint_to_receipt_host_s'] for m in matches
                                             if 'source_endpoint_to_receipt_host_s' in m),
        source_render_and_pack_host_s=statistics(s['render_and_pack_s'] for s in sources
            if s.get('render_and_pack_s') is not None),
        source_load_windows=[dict(session=key[0], host_start=anchor+key[1]*window_s,
                                  render_and_pack_host_s=statistics(values))
                             for key, values in sorted(load_bins.items())],
        local_clock_rates=rates, lifecycle=lifecycle_report(truth, lifecycle),
        state_association=dict(missing_state=missing_state, invalid_state_or_association=invalid_state,
            absolute_snapshot_age_host_s=None, image_minus_snapshot_age_s=None,
            raw_timing_fields=raw_timing_statistics(frames),
            raw_timing_scope='all retained frames, including invalid associations; raw values only',
            limitation='Raw GAP8 association and STM32 fields are reported where recorded; '
                       'absent/null historical fields remain unknown. '
                       'valid association constrains receive age <=50000 GAP8 us, not source/exposure/state age at host'),
        limitations=[
            'Unique observed hashes in legacy/partial traces are conditional evidence, not retrospective exact alignment.',
            'Identical hover pixels can have multiple source identities; no nearest-receipt disambiguation.',
            'Legacy endpoint is after send; render-call estimate includes render/conversion/packing/send and call skew.',
            'Physics scene is frozen during synchronous rendering; no physical shutter/exposure model.',
            'Host receipt is image-end parsing time, before possible association/consumer waits.',
            'GAP8 capture is DMA completion; STM32 snapshot is committed state; epochs and offsets remain separate.',
            'Rate windows are sampled progress against host time, affected by queueing, load and gaps; no cross-clock offset fit.',
            'No RTF multiplier or correction applied. EKF/controller dt and command-application clock coupling remain unqualified.'
        ])


def load_received_pixels(directory, frame):
    import numpy as np
    directory = Path(directory).resolve()
    path = (directory/frame['image_file']).resolve()
    if not path.is_relative_to(directory):
        raise ValueError('image_file escapes host recording directory')
    with np.load(path, allow_pickle=False) as archive:
        image = archive['image']
        if image.dtype != np.uint8 or image.shape != (244, 324):
            raise ValueError('received image must be native 244x324 Mono8')
        return image.tobytes(order='C')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, help='archived trial.json; all inputs are read-only')
    parser.add_argument('--source-log', type=Path, help='timing.jsonl or legacy source.jsonl')
    parser.add_argument('--native-source-log', type=Path,
                        help='optional legacy source log to attach post-send boundary to pre-transport trace')
    parser.add_argument('--host-frames', type=Path)
    parser.add_argument('--truth-log', type=Path)
    parser.add_argument('--lifecycle', type=Path)
    parser.add_argument('--hook-status', type=Path, help='evaluation_hooks.json with installed/final adapter evidence')
    parser.add_argument('--same-host-monotonic', action='store_true',
                        help='assert records share one host monotonic epoch; required for endpoint ages')
    parser.add_argument('--window-s', type=float, default=1.)
    parser.add_argument('--max-gap-s', type=float, default=.5)
    parser.add_argument('--max-rows', type=int, default=1000000)
    args = parser.parse_args()
    if not (math.isfinite(args.window_s) and 0 < args.window_s <= 3600
            and math.isfinite(args.max_gap_s) and 0 < args.max_gap_s <= args.window_s
            and 1 <= args.max_rows <= 10000000):
        parser.error('require finite 0 < max-gap-s <= window-s <= 3600 and max-rows in 1..10000000')
    if args.trial:
        trial = json.loads(args.trial.read_text())
        runtime = Path(trial['runtime_log_dir'])
        timing = runtime/'evaluation/timing.jsonl'
        args.source_log = args.source_log or (timing if timing.is_file() else runtime/'source.jsonl')
        if args.source_log == timing and not args.native_source_log:
            args.native_source_log = runtime/'source.jsonl'
        args.host_frames = args.host_frames or (args.trial.parent/'host/frames.jsonl')
        args.truth_log = args.truth_log or (runtime/'evaluation/ground_truth.jsonl')
        candidate = runtime/'evaluation/evaluation_lifecycle.json'
        if not args.lifecycle and candidate.is_file():
            args.lifecycle = candidate
        candidate = runtime/'evaluation/evaluation_hooks.json'
        if not args.hook_status and candidate.is_file():
            args.hook_status = candidate
    if not args.source_log or not args.host_frames or not args.truth_log:
        parser.error('provide --trial or source-log, host-frames and truth-log')
    sources, source_parse = read_jsonl(args.source_log, max_rows=args.max_rows)
    frames, frame_parse = read_jsonl(args.host_frames, max_rows=args.max_rows)
    truth, truth_parse = read_jsonl(args.truth_log, max_rows=args.max_rows)
    native, native_parse = (read_jsonl(args.native_source_log, max_rows=args.max_rows)
                            if args.native_source_log else ([], None))
    coverage = trace_coverage(sources, source_parse)
    lifecycle = json.loads(args.lifecycle.read_text()) if args.lifecycle else None
    hooks = json.loads(args.hook_status.read_text()) if args.hook_status else None
    result = analyze(sources, frames, truth, lambda f: load_received_pixels(args.host_frames.parent, f),
                     same_host_monotonic=args.same_host_monotonic,
                     source_coverage_complete=coverage['complete_capture_prefix_through_wrapper_close'],
                     window_s=args.window_s, max_gap_s=args.max_gap_s, lifecycle=lifecycle,
                     native_source_rows=native, hook_status=hooks)
    result['input_coverage'] = dict(source=source_parse, host_frames=frame_parse, truth=truth_parse,
                                    native_transport=native_parse,
                                    source_trace=coverage,
                                    legacy_source_lifetime_coverage='unverified; rotations/start/end may be absent')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
