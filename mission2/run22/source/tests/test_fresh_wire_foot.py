"""Current-pixel counterexamples; optional bounded native347 image-only replay."""
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import pytest

from mission_host.geometry import CameraModel, OPTICAL_TO_BODY
from mission_host.perception import fresh_wire_foot as fw
from mission_host.perception.passages import (
    FenceEvidence, PassageConfig, PassageDetector, PassageResult, _level_rotation,
)
from mission_host.types import EgoState, FrameObservation


CAM = CameraModel()
CONFIG = fw.FreshWireFootConfig(assume_flat_floor=True, assume_upright_wires=True)


@pytest.fixture(autouse=True)
def one_opencv_thread():
    previous = cv2.getNumThreads()
    cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(previous)


def scene(*, foot=180, floor=145, x=100, q=(0., 0., 0., 1.)):
    yy, xx = np.indices((244, 324))
    image = np.full((244, 324), 10, np.uint8)
    texture = 140+14*np.sin(yy/3.5)+10*np.sin(xx/12.)
    image[yy >= floor] = texture[yy >= floor].astype(np.uint8)
    image[18:foot+1, x] = 42
    state = EgoState(1200, (0., 0., 0.), (0., 0., 0.), q, .75, .75,
                     True, 8, 2, True, state_sequence=9, clock_domain_id=3,
                     imu_age_ms=0, tof_age_ms=10)
    level = _level_rotation(q)
    h = CAM.matrix @ OPTICAL_TO_BODY.T @ level @ OPTICAL_TO_BODY @ np.linalg.inv(CAM.matrix)
    raw = cv2.warpPerspective(image, np.linalg.inv(h), (324, 244))
    frame = FrameObservation(raw, 347, 12, 20., 2_000_000, state, True,
                             state_receive_age_at_frame_us=2000)
    fence = FenceEvidence((x-3., 18., x+3., 85.), 1, 0, None, .9,
        segments_level_px=((float(x), 18., float(x), 85.),),
        segment_depth_lower_m=(None,), segment_depth_source=('unranged',))
    result = PassageResult((), (), (fence,), (), frame.sequence, frame.session_id,
        True, 'synthetic_current_seed', tuple(map(tuple, level)), CAM,
        frame.capture_tick_us, state.snapshot_tick_ms)
    return frame, result


def observe(monkeypatch, frame, result, *, source=None, config=CONFIG):
    monkeypatch.setattr(fw.time, 'monotonic', lambda: frame.received_monotonic+.001)
    return fw.reobserve_wire_foot(frame, result, source or fw.current_wire_source(frame, 0, 0), config=config)


def with_image(frame, image):
    return replace(frame, image=image)


def test_continuous_same_wire_extends_hough_seed(monkeypatch):
    f, r = scene()
    before = r.as_dict()
    o = observe(monkeypatch, f, r)
    assert o.evidence is not None, o
    e = o.evidence
    assert e.pixel_level == (100., 180.5)
    assert e.source == fw.current_wire_source(f, 0, 0)
    assert e.ridge_coverage == 1.
    assert e.junction_rows == ()
    assert min(e.floor_contrast_dn) > 20 and min(e.floor_spread_dn) > 7
    assert e.coordinate_uncertainty_px >= 2.5
    assert e.pixel_bounds_level_px[1] <= 180 <= e.pixel_bounds_level_px[3]
    assert all(abs(u-100) <= 1 for u, v in e.ridge_pixels_level)
    assert r.as_dict() == before  # no core mutation / inferred range injection
    assert not any('permit' in key or 'depth' in key for key in asdict(e))


def test_small_grid_junction_gap_requires_transverse_pixel_proof(monkeypatch):
    f, r = scene()
    im = f.image.copy()
    im[60:62, 87:114] = 85
    good = observe(monkeypatch, with_image(f, im), r)
    assert good.evidence is not None, good
    assert good.evidence.junction_rows == (60, 61)
    assert .97 <= good.evidence.ridge_coverage < 1.
    assert observe(monkeypatch, with_image(f, im), r,
                   config=replace(CONFIG, max_junction_gap_px=0)).evidence is None
    im = f.image.copy()
    im[60:62, 100] = 10
    bad = observe(monkeypatch, with_image(f, im), r)
    assert bad.evidence is None and bad.reason == 'unsupported_ridge_gap'


@pytest.mark.parametrize('gap', [(90, 140), (150, 166)])
def test_no_unsupported_bridge_above_or_on_floor(monkeypatch, gap):
    f, r = scene()
    background, _ = scene(foot=17)
    im = f.image.copy()
    im[gap[0]:gap[1], 100] = background.image[gap[0]:gap[1], 100]
    o = observe(monkeypatch, with_image(f, im), r)
    assert o.evidence is None
    assert o.reason in ('unsupported_ridge_gap', 'ridge_resumes_below_candidate')


@pytest.mark.parametrize('neighbour_x,brightness', [(104, 55), (103, 55), (102, 100)])
def test_cannot_slide_onto_nearby_wire_or_borrow_its_foot(monkeypatch, neighbour_x, brightness):
    f, r = scene(foot=96)
    im = f.image.copy()
    im[18:181, neighbour_x] = brightness
    # Even an already ranged neighbour in the result is irrelevant to this API.
    neighbour = replace(r.fences[0], base_pixels_level=((float(neighbour_x), 180.),),
        observed_base_count=1, nearest_depth_lower_m=2.,
        segments_level_px=r.fences[0].segments_level_px+((float(neighbour_x), 18., float(neighbour_x), 180.),),
        segment_depth_lower_m=(None, 2.), segment_depth_source=('unranged', 'observed_wire_foot'))
    o = observe(monkeypatch, with_image(f, im), replace(r, fences=(neighbour,)))
    assert o.evidence is None


@pytest.mark.parametrize('foot', [235, 243])
def test_full_image_bottom_clipping_is_unknown(monkeypatch, foot):
    f, r = scene(foot=foot)
    assert observe(monkeypatch, f, r).evidence is None
    clipped = replace(r.fences[0], segments_level_px=((100., 18., 100., 243.),))
    assert observe(monkeypatch, f, replace(r, fences=(clipped,))).reason == 'bottom_clipped'


def test_hough_bottom_and_sky_contrast_loss_are_not_floor(monkeypatch):
    f, r = scene(foot=85, floor=200)
    assert observe(monkeypatch, f, r).evidence is None
    f, r = scene(foot=180, floor=244)
    assert observe(monkeypatch, f, r).evidence is None


def test_floor_transition_does_not_end_a_continuing_wire(monkeypatch):
    f, r = scene(foot=243)
    assert observe(monkeypatch, f, r).evidence is None


def test_ridge_resuming_below_false_floor_end_is_unknown(monkeypatch):
    f, r = scene(foot=243)
    background, _ = scene(foot=17)
    im = f.image.copy()
    im[181:202, 100] = background.image[181:202, 100]
    o = observe(monkeypatch, with_image(f, im), r)
    assert o.evidence is None and o.reason == 'ridge_resumes_below_candidate'


def test_uniform_below_region_is_not_supported_floor(monkeypatch):
    f, r = scene()
    im = f.image.copy()
    im[145:] = 140
    im[145:181, 100] = 42
    assert observe(monkeypatch, with_image(f, im), r).evidence is None


def test_floor_like_occluder_at_apparent_tip_has_no_ridge_on_floor(monkeypatch):
    f, r = scene(foot=243, floor=244)
    textured_face, _ = scene(foot=17, floor=181)
    im = f.image.copy()
    # A textured opaque face covers the wire from row 181 downward. Its
    # grayscale resembles floor, but no ridge was visible ON that surface.
    im[181:] = textured_face.image[181:]
    o = observe(monkeypatch, with_image(f, im), r)
    assert o.evidence is None and o.reason == 'no_supported_floor_endpoint'


@pytest.mark.parametrize('field', ['session_id', 'boot_id', 'reset_generation', 'sequence',
                                  'capture_tick_us', 'snapshot_tick_ms'])
def test_current_source_identity_is_exact(monkeypatch, field):
    f, r = scene()
    source = fw.current_wire_source(f, 0, 0)
    old = replace(source, **{field: getattr(source, field)-1})
    o = observe(monkeypatch, f, r, source=old)
    assert o.evidence is None and o.reason == 'source_identity_mismatch'


@pytest.mark.parametrize('field', ['source_sequence', 'source_session_id',
                                  'source_capture_tick_us', 'source_snapshot_tick_ms'])
def test_stale_result_rejected_even_with_current_source(monkeypatch, field):
    f, r = scene()
    old = replace(r, **{field: getattr(r, field)-1})
    assert observe(monkeypatch, f, old).reason == 'result_source_mismatch'


def test_freshness_association_and_priors_are_required(monkeypatch):
    f, r = scene()
    assert observe(monkeypatch, f, r, config=fw.FreshWireFootConfig()).reason == 'priors_not_accepted'
    assert observe(monkeypatch, replace(f, association_valid=False), r).reason == 'invalid_association'
    monkeypatch.setattr(fw.time, 'monotonic', lambda: f.received_monotonic+.5)
    assert fw.reobserve_wire_foot(f, r, fw.current_wire_source(f, 0, 0), config=CONFIG).reason == 'stale_or_future_frame'
    monkeypatch.setattr(fw.time, 'monotonic', lambda: f.received_monotonic-.01)
    assert fw.reobserve_wire_foot(f, r, fw.current_wire_source(f, 0, 0), config=CONFIG).reason == 'stale_or_future_frame'
    stale = replace(f, state_receive_age_at_frame_us=50_001)
    assert observe(monkeypatch, stale, r).reason == 'stale_associated_state'


def test_actual_gravity_transform_handles_tilt_and_yaw(monkeypatch):
    # Composition Rz(yaw) Ry(pitch) Rx(roll), with no world position.
    roll, pitch, yaw = map(math.radians, (9., -5., 31.))
    from scipy.spatial.transform import Rotation
    q = tuple(Rotation.from_euler('xyz', (roll, pitch, yaw)).as_quat())
    f, r = scene(q=q)
    o = observe(monkeypatch, f, r)
    assert o.evidence is not None, o
    e = o.evidence
    assert abs(e.pixel_level[0]-100) <= 1 and abs(e.pixel_level[1]-180.5) <= 2
    assert abs(e.pixel_image[0]-e.pixel_level[0]) > 3
    h = CAM.matrix @ OPTICAL_TO_BODY.T @ np.asarray(e.body_to_level) @ OPTICAL_TO_BODY @ np.linalg.inv(CAM.matrix)
    mapped = h @ (*e.pixel_image, 1.)
    assert np.allclose(mapped[:2]/mapped[2], e.pixel_level)
    wrong = replace(r, body_to_level=tuple(map(tuple, np.eye(3))))
    assert observe(monkeypatch, f, wrong).reason == 'level_transform_mismatch'


@pytest.mark.parametrize('kwargs', [dict(max_extension_px=201), dict(max_junction_gap_px=4),
                                   dict(pixel_error_px=.5), dict(max_received_age_s=float('nan')),
                                   dict(assume_flat_floor=1)])
def test_compute_and_uncertainty_budgets_are_bounded(kwargs):
    with pytest.raises(ValueError):
        fw.FreshWireFootConfig(**kwargs)


@pytest.mark.skipif(os.environ.get('FRESH_WIRE_FOOT_REPLAY') != '1',
                    reason='opt-in bounded native347 current image/state replay')
def test_optional_actual347_current_pixels(monkeypatch):
    root = Path(__file__).resolve().parents[1]/'.local/evaluation/entry_bypass_sweep_20261026/round08/cases/h075_lat006/workspace/.local/evaluation/trial/host'
    if not (root/'frames.jsonl').is_file():
        pytest.skip('round08 actual347 host files unavailable')
    record = next(json.loads(line) for line in (root/'frames.jsonl').open()
                  if json.loads(line)['sequence'] == 347)
    with np.load(root/record['image_file'], allow_pickle=False) as saved:
        image = saved['image']
    f = FrameObservation(image, record['sequence'], record['session_id'], record['received_monotonic'],
        record['capture_tick_us'], EgoState(**record['state']), record['association_valid'],
        **{k: record[k] for k in ('gap8_state_receive_tick_us', 'state_receive_age_at_frame_us', 'receiver_sequence')})
    r = PassageDetector(PassageConfig(assume_flat_floor=True, assume_upright_opaque_walls=True)).detect(f)
    assert r.fences[0].segments_level_px[1] == (32., 4., 32., 88.)
    assert r.fences[0].segment_depth_lower_m[1] is None
    # Only the replay clock is substituted. No recorded decision, GT, metric
    # scene position, neighbouring foot or known dimensions enter the helper.
    o = observe(monkeypatch, f, r, source=fw.current_wire_source(f, 0, 1))
    assert o.evidence is not None, o
    assert o.evidence.pixel_level == (32., 174.5)
    assert o.evidence.ridge_coverage == 1. and not o.evidence.junction_rows
    assert o.evidence.source.sequence == 347


@pytest.mark.skipif(os.environ.get('FRESH_WIRE_FOOT_REPLAY') != '1',
                    reason='opt-in bounded source52/55 unknown regressions')
@pytest.mark.parametrize('recording,sequence,indices', [
    ('cycle52_course_workspace', 193, (10, 11, 12)),
    ('cycle52_course_workspace', 241, (13, 14)),
    ('cycle55_course_workspace', 184, (8,)),
])
def test_optional_known52_55_unknowns_remain_unknown(monkeypatch, recording, sequence, indices):
    root = Path(__file__).resolve().parents[1]/'.local/evaluation'/recording/'.local/evaluation/official_course/host'
    if not (root/'frames.jsonl').is_file():
        pytest.skip('optional native host recording unavailable')
    record = next(json.loads(line) for line in (root/'frames.jsonl').open()
                  if json.loads(line)['sequence'] == sequence)
    with np.load(root/record['image_file'], allow_pickle=False) as saved:
        image = saved['image']
    f = FrameObservation(image, record['sequence'], record['session_id'], record['received_monotonic'],
        record['capture_tick_us'], EgoState(**record['state']), record['association_valid'],
        **{k: record[k] for k in ('gap8_state_receive_tick_us', 'state_receive_age_at_frame_us', 'receiver_sequence')
           if k in record})
    r = PassageDetector(PassageConfig(assume_flat_floor=True, assume_upright_opaque_walls=True)).detect(f)
    for index in indices:
        assert r.fences[0].segment_depth_lower_m[index] is None
        o = observe(monkeypatch, f, r, source=fw.current_wire_source(f, 0, index))
        assert o.evidence is None, o
