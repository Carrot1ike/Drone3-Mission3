"""미션3 실행 종료 후 독립 평가기.

계약 문서: docs/mission3/evaluation-contract.md (실행 로그 mission3-run/1, 리뷰 mission3-review/1)

이 모듈은 완료된 실행의 로그만 읽는다. 출력은 사후 평가이며 제어기, 추정기,
plant의 입력이나 실행 중 튜닝 신호로 쓰지 않는다. 평가 결과로 제어를 고치는 것은
실행이 끝난 뒤 오프라인에서만 하고, 고친 코드는 새 해시와 새 holdout seed로만 집계한다.

평가기는 성공을 승인하지 않는다. qualified_mission_success는 평가기 단계에서 최대
pending_review이고, 독립 리뷰 기록이 검증될 때 ledger에서만 집계된다. 평가기는
승인된 리뷰를 만들 수 없다. 제어 모듈(controller, estimator, vision, scene)을
import하지 않는다.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

CONTRACT_VERSION = "mission3-evaluation/1"
RUN_SCHEMA = "mission3-run/1"
REVIEW_SCHEMA = "mission3-review/1"
CONSUMER_RESTRICTION = (
    "post-run review only: never an input to controller, estimator, plant or runtime tuning; "
    "offline improvements require a new code hash and new holdout seeds")
TEMPLATE_MARKER = "evaluate_mission3 review template (unsigned)"

PASS, FAIL, INELIGIBLE, PENDING = "pass", "fail", "ineligible", "pending_review"

# 평가 계약 상수. 실행 설정으로 바꿀 수 없다. 바꾸려면 CONTRACT_VERSION을 올린다.
HOLD_S = 3.0
SCORE_MAX = 10
SCORE_STEP_NM = 10_000_000        # 0.01 m
SCORE_RADIUS_NM = 100_000_000     # 0.10 m, 0.20 m 상판의 내접원
MAX_TRUTH_GAP_S = 0.05
MAX_MOTOR_GAP_S = 0.05
VEHICLE_SPEED_TOLERANCE_M_S = 0.02
TIME_EPS_S = 1e-9
FEET = ("foot_1", "foot_2", "foot_3", "foot_4")
PAD = "pad"
FLOOR = "floor"
CAMERA_CONTRACT = {"width": 320, "height": 320, "encoding": "mono8", "channels": 1}
EVIDENCE_LEVELS = ("L0", "L1", "L2", "L3")
PHYSICS_LEVELS = ("L2", "L3")
RUN_KINDS = ("mission3_closed_loop", "contact_fixture", "optical_geometry")
SPLITS = ("training", "holdout")
ENTRY_ORDER = tuple(range(1, 9))
SUCCESSES_PER_ENTRY = 5
REQUIRED_PROFILE_FIELDS = (
    "assembled_mass_kg", "assembled_center_of_mass_m", "assembled_inertia_kg_m2",
    "pwm_to_thrust_model", "motor_response_model", "aerodynamic_model",
    "camera_K", "camera_distortion", "R_body_camera", "camera_position_body_m",
    "camera_capture_timestamp", "flow_deck_position_body_m", "flow_fov_deg",
    "flow_min_height_m", "tof_fov_deg", "tof_min_distance_m", "landing_contact_geometry",
)
PROFILE_SOURCE_ACCEPTED = ("manufacturer",)
FORBIDDEN_CONTROLLER_KEYS = (
    "vehicle_plant_only", "initial_track_arc_m", "initial_track_arc_m_plant_only",
    "initial_arc", "initial_arc_m", "seed", "truth", "in_turn", "vehicle_in_turn", "plant",
)
AUTOMATED_IDENTITIES = ("", "evaluator", "evaluate_mission3", "auto", "automatic", "self", "controller", "template")


def evaluator_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def evaluation_digest(evaluation):
    """리뷰가 묶이는 평가 결과 해시. 키 정렬 JSON의 SHA-256."""
    blob = json.dumps(evaluation, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def score_points(distance_m):
    """첫 접촉 시 기체 중심 수평 투영과 상판 중심 거리의 내부 점수.

    거리를 1nm로 반올림한 뒤 [0, 0.01) → 10, [0.01, 0.02) → 9, ..., [0.09, 0.10) → 1,
    0.10m 이상 → 0. 경계값은 낮은 점수 쪽이다. 상판 모서리(약 0.141m)는 0점이다.
    대회 공식 점수가 아니다.
    """
    if isinstance(distance_m, bool) or not isinstance(distance_m, (int, float)):
        raise ValueError("distance must be a number")
    if not math.isfinite(distance_m) or distance_m < 0:
        raise ValueError("distance must be finite and non-negative")
    nanometres = int(round(distance_m * 1e9))
    if nanometres >= SCORE_RADIUS_NM:
        return 0
    return SCORE_MAX - nanometres // SCORE_STEP_NM


def _verdict(fail=(), ineligible=(), **details):
    fail, ineligible = list(dict.fromkeys(fail)), list(dict.fromkeys(ineligible))
    status = FAIL if fail else INELIGIBLE if ineligible else PASS
    return {"status": status, "fail_reasons": fail, "ineligible_reasons": ineligible, **details}


def _combine(*verdicts, **details):
    fail, ineligible = [], []
    for v in verdicts:
        fail += v["fail_reasons"]
        ineligible += v["ineligible_reasons"]
    return _verdict(fail, ineligible, **details)


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("finite number required")
    return float(value)


def _vec3(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("3-vector required")
    return tuple(_finite(v) for v in value)


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _read_jsonl(path):
    rows = []
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as error:
                    raise ValueError(f"line {number}: {error.msg}") from error
    return rows


def load_run(run_dir):
    """완료된 실행 디렉터리를 읽는다. 누락·손상은 기록만 하고 판정은 evaluate_loaded가 한다."""
    run_dir = Path(run_dir)
    data = {"run_dir": str(run_dir), "missing": [], "corrupt": []}
    try:
        data["manifest"] = _read_json(run_dir / "run_manifest.json")
    except FileNotFoundError:
        data["manifest"] = None
        data["missing"].append("run_manifest.json")
    except (ValueError, OSError) as error:
        data["manifest"] = None
        data["corrupt"].append(f"run_manifest.json:{error}")
    for name, key in (("truth.jsonl", "truth"), ("motors.jsonl", "motors"),
                      ("commands.jsonl", "commands"), ("camera_frames.jsonl", "camera")):
        try:
            data[key] = _read_jsonl(run_dir / name)
        except FileNotFoundError:
            data[key] = None
            data["missing"].append(name)
        except (ValueError, OSError) as error:
            data[key] = None
            data["corrupt"].append(f"{name}:{error}")
    settings_name = "controller-settings.json"
    if isinstance(data["manifest"], dict):
        settings_name = data["manifest"].get("controller_settings_file", settings_name)
    path = Path(str(settings_name))
    if path.is_absolute() or ".." in path.parts:
        data["controller_settings"] = None
        data["corrupt"].append(f"controller_settings_file outside run directory: {settings_name}")
    else:
        try:
            data["controller_settings"] = _read_json(run_dir / path)
        except FileNotFoundError:
            data["controller_settings"] = None
            data["missing"].append(str(path))
        except (ValueError, OSError) as error:
            data["controller_settings"] = None
            data["corrupt"].append(f"{path}:{error}")
    return data


def _file_problem(data, name):
    for item in data["missing"]:
        if item == name:
            return f"missing_log:{name}"
    for item in data["corrupt"]:
        if item.startswith(name):
            return f"corrupt_log:{item}"
    return None


def check_manifest(m):
    if not isinstance(m, dict):
        return _verdict(ineligible=["manifest_unavailable"])
    ineligible = []
    if m.get("schema") != RUN_SCHEMA:
        ineligible.append(f"schema:{m.get('schema')}")
    if m.get("completed") is not True:
        ineligible.append("run_not_completed")
    if m.get("run_kind") not in RUN_KINDS:
        ineligible.append(f"run_kind:{m.get('run_kind')}")
    if m.get("evidence_level") not in EVIDENCE_LEVELS:
        ineligible.append(f"evidence_level:{m.get('evidence_level')}")
    for key in ("run_id", "author", "config_sha256"):
        if not isinstance(m.get(key), str) or not m.get(key):
            ineligible.append(f"manifest_field:{key}")
    if not isinstance(m.get("code_sha256"), dict) or not m.get("code_sha256"):
        ineligible.append("manifest_field:code_sha256")
    return _verdict(ineligible=ineligible)


def check_camera(frames, problem=None):
    if problem:
        return _verdict(ineligible=[problem])
    if not frames:
        return _verdict(ineligible=["no_camera_frames"])
    fail, count = [], 0
    for row in frames:
        bad = [f"{k}={row.get(k)!r}" for k, v in CAMERA_CONTRACT.items() if row.get(k) != v]
        bad += [k for k in ("resized", "color_converted") if row.get(k) is True]
        if bad:
            count += 1
            if not fail:
                fail.append(f"camera_contract:{','.join(bad)}@t={row.get('t')}")
    if count:
        fail.append(f"camera_contract_violations:{count}/{len(frames)}")
    return _verdict(fail, frames=len(frames))


def _forbidden_paths(node, path=()):
    found = []
    if isinstance(node, dict):
        for key, value in node.items():
            key_l = str(key).lower()
            here = path + (key_l,)
            if key_l in FORBIDDEN_CONTROLLER_KEYS or (
                    key_l == "speed_m_s" and any("vehicle" in p for p in path)):
                found.append(".".join(here))
            found += _forbidden_paths(value, here)
    elif isinstance(node, list):
        for index, value in enumerate(node):
            found += _forbidden_paths(value, path + (str(index),))
    return found


def check_isolation(settings, problem=None):
    if problem:
        return _verdict(ineligible=[problem])
    if not isinstance(settings, dict):
        return _verdict(ineligible=["controller_settings_unavailable"])
    found = _forbidden_paths(settings)
    return _verdict([f"forbidden_controller_key:{p}" for p in found])


def check_physics_evidence(m):
    level = m.get("evidence_level")
    if level in PHYSICS_LEVELS and m.get("physics_contact") is True:
        return _verdict()
    return _verdict([f"no_physics_contact_evidence:{level}"])


def check_profile(m):
    profile = m.get("aircraft_profile")
    if not isinstance(profile, dict):
        return _verdict(ineligible=["aircraft_profile_unavailable"])
    fail = []
    for field in REQUIRED_PROFILE_FIELDS:
        item = profile.get(field)
        if not isinstance(item, dict):
            fail.append(f"profile_missing:{field}")
        elif item.get("value") is None:
            fail.append(f"profile_null:{field}")
        elif item.get("source_class") not in PROFILE_SOURCE_ACCEPTED:
            fail.append(f"profile_source:{field}={item.get('source_class')}")
        elif not isinstance(item.get("source_ref"), str) or not item["source_ref"].strip():
            fail.append(f"profile_source_ref:{field}")
    return _verdict(fail, profile_id=profile.get("profile_id"))


def check_closed_loop(m):
    kind = m.get("run_kind")
    if kind == "contact_fixture":
        return _verdict(["contact_fixture_not_mission3"])
    if kind == "optical_geometry":
        return _verdict(["optical_geometry_not_physical_mission"])
    fail = []
    if kind != "mission3_closed_loop":
        fail.append(f"run_kind:{kind}")
    if m.get("entry_id") not in ENTRY_ORDER:
        fail.append(f"entry_id:{m.get('entry_id')}")
    if m.get("controller_in_loop") is not True:
        fail.append("controller_not_in_loop")
    if m.get("started_from") != "HANDOFF_IN":
        fail.append(f"started_from:{m.get('started_from')}")
    if m.get("evidence_level") not in PHYSICS_LEVELS:
        fail.append(f"evidence_level:{m.get('evidence_level')}")
    return _verdict(fail)


def _parse_truth(rows):
    parsed, bad = [], []
    for number, row in enumerate(rows, 1):
        try:
            turn = row["vehicle"]["in_turn"]
            if not isinstance(turn, bool):
                raise ValueError("in_turn must be bool")
            contacts = []
            for c in row["contacts"]:
                part, other = c["aircraft_part"], c["other"]
                if not isinstance(part, str) or not isinstance(other, str):
                    raise ValueError("contact labels must be strings")
                contacts.append((part, other))
            parsed.append({"t": _finite(row["t"]),
                           "aircraft": _vec3(row["aircraft"]["center_world_m"]),
                           "pad": _vec3(row["pad"]["center_world_m"]),
                           "vehicle_v": _vec3(row["vehicle"]["velocity_world_m_s"]),
                           "in_turn": turn, "contacts": contacts})
        except (KeyError, TypeError, ValueError):
            bad.append(number)
    return parsed, bad


def _parse_motors(rows):
    parsed, bad = [], []
    for number, row in enumerate(rows, 1):
        try:
            out = row["motor_output"]
            if not isinstance(out, (list, tuple)) or len(out) != 4:
                raise ValueError("four motor outputs required")
            parsed.append((_finite(row["t"]), tuple(_finite(v) for v in out)))
        except (KeyError, TypeError, ValueError):
            bad.append(number)
    return parsed, bad


def _timeline_problem(times, max_gap, name):
    if len(times) < 2:
        return f"{name}_too_short"
    for a, b in zip(times, times[1:]):
        if b <= a:
            return f"{name}_time_not_increasing@t={b}"
        if b - a > max_gap + TIME_EPS_S:
            return f"{name}_gap:{b - a:.4f}s@t={a}"
    return None


def evaluate_loaded(data):
    m = data.get("manifest")
    manifest = check_manifest(m)
    m = m if isinstance(m, dict) else {}
    result = {
        "contract_version": CONTRACT_VERSION,
        "evaluator_sha256": evaluator_sha256(),
        "consumer_restriction": CONSUMER_RESTRICTION,
        "official_competition_score": False,
        "run_dir": data.get("run_dir"),
        "run_id": m.get("run_id"), "run_kind": m.get("run_kind"), "evidence_level": m.get("evidence_level"),
        "run_author": m.get("author"), "entry_id": m.get("entry_id"), "seed": m.get("seed"),
        "split": m.get("split"), "series_id": m.get("series_id"),
        "code_sha256": m.get("code_sha256"), "config_sha256": m.get("config_sha256"),
        "missing_logs": list(data.get("missing", [])), "corrupt_logs": list(data.get("corrupt", [])),
    }
    checks = {"manifest": manifest}
    checks["camera_contract"] = check_camera(data.get("camera"), _file_problem(data, "camera_frames.jsonl"))
    settings_name = str(m.get("controller_settings_file", "controller-settings.json"))
    settings_problem = _file_problem(data, settings_name) or next(
        (f"corrupt_log:{c}" for c in data.get("corrupt", []) if c.startswith("controller_settings_file")), None)
    checks["controller_isolation"] = check_isolation(data.get("controller_settings"), settings_problem)
    manifest_ok = manifest["status"] == PASS
    # manifest를 신뢰할 수 없으면 manifest 파생 판정은 실패가 아니라 판정 불가다.
    unknown = _verdict(ineligible=["manifest_not_valid"])
    checks["physics_evidence"] = check_physics_evidence(m) if manifest_ok else unknown
    checks["closed_loop_mission"] = check_closed_loop(m) if manifest_ok else unknown

    # 참값·모터 로그 무결성
    integrity_inel = []
    truth, motors = [], []
    problem = _file_problem(data, "truth.jsonl")
    if problem:
        integrity_inel.append(problem)
    else:
        truth, bad = _parse_truth(data["truth"])
        if bad:
            integrity_inel.append(f"malformed_truth_lines:{bad[:5]}")
        gap = _timeline_problem([r["t"] for r in truth], MAX_TRUTH_GAP_S, "truth")
        if gap:
            integrity_inel.append(gap)
    problem = _file_problem(data, "motors.jsonl")
    if problem:
        integrity_inel.append(problem)
    else:
        motors, bad = _parse_motors(data["motors"])
        if bad:
            integrity_inel.append(f"malformed_motor_lines:{bad[:5]}")
        gap = _timeline_problem([t for t, _ in motors], MAX_MOTOR_GAP_S, "motor")
        if gap:
            integrity_inel.append(gap)
    checks["log_integrity"] = _verdict(ineligible=integrity_inel)
    usable = not integrity_inel

    touchdown = None
    if usable:
        touchdown = next((r for r in truth if any(o == PAD for _, o in r["contacts"])), None)
    t_td = touchdown["t"] if touchdown else None

    # 모터 off: 첫 접촉 이후 4개 출력이 모두 0이고 HOLD_S 동안 유지되는 가장 이른 시각
    t_off, motor_fail, motor_inel = None, [], []
    if not usable:
        motor_inel.append("log_integrity")
    elif t_td is None:
        motor_fail.append("no_pad_contact")
    else:
        before = [out for t, out in motors if t < t_td - TIME_EPS_S]
        if before and all(v == 0 for v in before[-1]):
            motor_fail.append("motors_off_before_touchdown")
        after = [(t, out) for t, out in motors if t >= t_td - TIME_EPS_S]
        index, open_window = 0, False
        while index < len(after):
            t, out = after[index]
            if not all(v == 0 for v in out):
                index += 1
                continue
            end = t + HOLD_S
            window = [(tt, oo) for tt, oo in after[index:] if tt <= end + TIME_EPS_S]
            broken = next((tt for tt, oo in window if not all(v == 0 for v in oo)), None)
            if broken is not None:
                index = next(i for i, (tt, _) in enumerate(after) if tt >= broken)
                continue
            if window[-1][0] < end - TIME_EPS_S:
                open_window = True
                break
            t_off = t
            break
        if t_off is None and not motor_fail:
            if open_window:
                motor_inel.append("motor_log_ends_before_hold_complete")
            else:
                motor_fail.append("motors_not_off_for_hold")
    checks["motors_off"] = _verdict(motor_fail, motor_inel, motor_off_time_s=t_off)
    hold_end = None if t_off is None else t_off + HOLD_S

    # 충돌: 로그 시작부터 유지 종료(없으면 로그 끝)까지 상판 외 접촉 금지, 바닥 접촉 금지
    collision_fail = []
    if usable:
        horizon = hold_end if hold_end is not None else truth[-1]["t"]
        for r in truth:
            if r["t"] > horizon + TIME_EPS_S:
                break
            for part, other in r["contacts"]:
                if other == FLOOR:
                    collision_fail.append(f"floor_contact:{part}")
                elif other != PAD:
                    collision_fail.append(f"collision:{part}->{other}")
                elif part not in FEET:
                    collision_fail.append(f"non_foot_pad_contact:{part}")
    checks["collision"] = _verdict(collision_fail, [] if usable else ["log_integrity"])
    checks["touchdown"] = _verdict([] if (t_td is not None or not usable) else ["no_pad_contact"],
                                   [] if usable else ["log_integrity"], first_touchdown_time_s=t_td)

    # 3초 지지: 모터 off 구간 내 모든 참값 표본에서 4발 모두 상판 접촉, 다른 접촉 없음
    hold_fail, hold_inel = [], []
    if hold_end is None:
        (hold_inel if (motor_inel or not usable) else hold_fail).append("no_motor_off_window")
    else:
        window = [r for r in truth if t_off - TIME_EPS_S <= r["t"] <= hold_end + TIME_EPS_S]
        if truth[-1]["t"] < hold_end - TIME_EPS_S:
            hold_inel.append("truth_log_ends_before_hold_complete")
        for r in window:
            feet = {p for p, o in r["contacts"] if o == PAD and p in FEET}
            missing = [f for f in FEET if f not in feet]
            if missing:
                hold_fail.append(f"foot_not_on_pad:{','.join(missing)}@t={r['t']:.3f}")
                break
        for r in window:
            others = [f"{p}->{o}" for p, o in r["contacts"] if o != PAD]
            if others:
                hold_fail.append(f"other_contact_during_hold:{others[0]}@t={r['t']:.3f}")
                break
    checks["hold_support"] = _verdict(hold_fail, hold_inel, hold_s=HOLD_S, hold_end_s=hold_end)

    # 차량 참값: 접촉부터 유지 종료까지 상판이 선언된 속도로 계속 움직였는가
    nominal = m.get("vehicle_nominal_speed_m_s")
    vm_fail, vm_inel = [], []
    if not isinstance(nominal, (int, float)) or isinstance(nominal, bool) or not math.isfinite(nominal):
        vm_inel.append("vehicle_nominal_speed_unavailable")
    elif t_td is None or hold_end is None:
        vm_inel.append("no_touchdown_hold_window")
    else:
        for r in truth:
            if t_td - TIME_EPS_S <= r["t"] <= hold_end + TIME_EPS_S:
                speed = math.hypot(r["vehicle_v"][0], r["vehicle_v"][1])
                if abs(speed - nominal) > VEHICLE_SPEED_TOLERANCE_M_S:
                    vm_fail.append(f"vehicle_speed_deviation:{speed:.4f}@t={r['t']:.3f}")
                    break
    checks["vehicle_motion"] = _verdict(vm_fail, vm_inel, nominal_speed_m_s=nominal,
                                        static_pad=(nominal == 0) if not vm_inel else None)

    # 하강 직선: 하강 시작부터 유지 종료까지 차량 곡선 금지(보수 해석)
    ds_fail, ds_inel, descent_start = [], [], None
    phases = m.get("descent_phases")
    problem = _file_problem(data, "commands.jsonl")
    if problem:
        ds_inel.append(problem)
    elif not isinstance(phases, list) or not phases:
        ds_inel.append("descent_phases_unavailable")
    elif t_td is None or hold_end is None:
        ds_inel.append("no_touchdown_hold_window")
    else:
        try:
            commands = [(_finite(c["t"]), c["phase"]) for c in data["commands"]]
        except (KeyError, TypeError, ValueError):
            commands = None
            ds_inel.append("malformed_commands")
        if commands is not None:
            descent_start = next((t for t, p in commands if p in phases), None)
            if descent_start is None or descent_start > t_td + TIME_EPS_S:
                ds_fail.append("touchdown_without_declared_descent")
            else:
                if touchdown["in_turn"]:
                    ds_fail.append("touchdown_in_turn")
                turn = next((r["t"] for r in truth
                             if descent_start - TIME_EPS_S <= r["t"] <= hold_end + TIME_EPS_S and r["in_turn"]), None)
                if turn is not None:
                    ds_fail.append(f"vehicle_in_turn_between_descent_and_hold_end@t={turn:.3f}")
    checks["descent_straight"] = _verdict(ds_fail, ds_inel, descent_start_s=descent_start)

    timeout = m.get("mission_timeout_s")
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
        checks["mission_time"] = _verdict(ineligible=["mission_timeout_unavailable"])
    elif hold_end is None or not truth:
        checks["mission_time"] = _verdict(ineligible=["no_touchdown_hold_window"])
    else:
        t0 = m.get("handoff_time_s", truth[0]["t"])
        elapsed = hold_end - t0
        checks["mission_time"] = _verdict([] if elapsed <= timeout + TIME_EPS_S else
                                          [f"mission_time_exceeded:{elapsed:.2f}>{timeout}"], elapsed_s=elapsed)

    score = None
    if touchdown is not None:
        distance = math.hypot(touchdown["aircraft"][0] - touchdown["pad"][0],
                              touchdown["aircraft"][1] - touchdown["pad"][1])
        score = {"points": score_points(distance), "distance_m": distance,
                 "time_s": t_td, "point": m.get("aircraft_center_definition"),
                 "rule": "1nm 반올림 후 [0,0.01)=10 ... [0.09,0.10)=1, 0.10 이상=0",
                 "official_competition_score": False}
    result["score"] = score
    result["checks"] = checks

    contact = _combine(manifest, checks["physics_evidence"], checks["log_integrity"], checks["collision"],
                       checks["touchdown"], checks["motors_off"], checks["hold_support"], checks["vehicle_motion"],
                       scope=m.get("run_kind"),
                       note="시뮬레이션 접촉 판정. 기체 파라미터가 조건부이면 제조사 일치 성공이 아니다")
    profile = _combine(manifest, check_profile(m) if manifest_ok else unknown)
    qualified = _combine(contact, profile, checks["closed_loop_mission"], checks["camera_contract"],
                         checks["controller_isolation"], checks["descent_straight"], checks["mission_time"])
    if qualified["status"] == PASS:
        qualified["status"] = PENDING
        qualified["note"] = "평가기 조건 통과. 독립 리뷰 기록이 검증되기 전에는 성공이 아니다"
    result["simulation_contact_success"] = contact
    result["manufacturer_profile_matches"] = profile
    result["qualified_mission_success"] = qualified
    result["physical_success_counted"] = False
    return result


def evaluate_run(run_dir):
    return evaluate_loaded(load_run(run_dir))


def review_template(evaluation, reviewer_id):
    """서명 전 리뷰 양식. 승인 플래그는 모두 False이며 이 양식으로는 승인할 수 없다."""
    return {"schema": REVIEW_SCHEMA, "generated_by": TEMPLATE_MARKER,
            "review_id": None, "run_id": evaluation.get("run_id"),
            "reviewer": {"id": reviewer_id, "role": "independent_reviewer"},
            "run_author": evaluation.get("run_author"),
            "code_sha256": evaluation.get("code_sha256"), "config_sha256": evaluation.get("config_sha256"),
            "evaluator_sha256": evaluation.get("evaluator_sha256"),
            "evaluation_sha256": evaluation_digest(evaluation),
            "approved": {"simulation_contact_success": False, "qualified_mission_success": False},
            "findings": [], "reviewed_at_kst": None}


def validate_review(review, evaluation):
    """리뷰 기록의 독립성·해시 결합·승인 범위를 검사한다."""
    reasons = []
    if not isinstance(review, dict) or review.get("schema") != REVIEW_SCHEMA:
        return {"valid": False, "reasons": ["review_schema"], "approves_simulation_contact": False,
                "approves_qualified": False}
    reviewer = review.get("reviewer") or {}
    reviewer_id = str(reviewer.get("id") or "").strip()
    author = str(evaluation.get("run_author") or "").strip()
    if reviewer.get("role") != "independent_reviewer":
        reasons.append("reviewer_role_not_independent")
    if reviewer_id.lower() in AUTOMATED_IDENTITIES:
        reasons.append("automated_or_empty_reviewer")
    if author and reviewer_id.lower() == author.lower():
        reasons.append("self_review")
    if review.get("generated_by") == TEMPLATE_MARKER:
        reasons.append("unsigned_template")
    if not review.get("review_id") or not review.get("reviewed_at_kst"):
        reasons.append("review_id_or_time_missing")
    if review.get("run_id") != evaluation.get("run_id"):
        reasons.append("run_id_mismatch")
    if review.get("run_author") != evaluation.get("run_author"):
        reasons.append("run_author_mismatch")
    if review.get("code_sha256") != evaluation.get("code_sha256"):
        reasons.append("code_sha256_mismatch")
    if review.get("config_sha256") != evaluation.get("config_sha256"):
        reasons.append("config_sha256_mismatch")
    if review.get("evaluator_sha256") != evaluation.get("evaluator_sha256"):
        reasons.append("evaluator_sha256_mismatch")
    if review.get("evaluation_sha256") != evaluation_digest(evaluation):
        reasons.append("evaluation_sha256_mismatch")
    approved = review.get("approved") or {}
    sim = approved.get("simulation_contact_success") is True
    qual = approved.get("qualified_mission_success") is True
    if sim and evaluation["simulation_contact_success"]["status"] != PASS:
        reasons.append("approves_failed_simulation_contact")
    if qual and evaluation["qualified_mission_success"]["status"] != PENDING:
        reasons.append("approves_unqualified_mission")
    if qual and not sim:
        reasons.append("qualified_without_contact_approval")
    valid = not reasons
    return {"valid": valid, "reasons": reasons, "reviewer": reviewer_id,
            "approves_simulation_contact": valid and sim, "approves_qualified": valid and qual}


def build_ledger(attempts):
    """실행 순서대로 받은 시도 목록에서 집계한다. 입력의 임의 집계 필드는 읽지 않는다.

    attempts: [{"evaluation": dict, "reviews": [dict, ...]}, ...]
    """
    rows, training_seeds, frozen = [], {}, {}
    for item in attempts:
        ev = item["evaluation"]
        if ev.get("split") == "training":
            training_seeds.setdefault((ev.get("entry_id"), ev.get("seed")), []).append(ev.get("run_id"))
    holdout_seen = {}
    current_index, counts, random_successes = 0, {e: 0 for e in ENTRY_ORDER}, 0
    tallies = {e: {"attempts": 0, "training": 0, "holdout": 0, "counted_successes": 0,
                   "qualified_status": {PENDING: 0, FAIL: 0, INELIGIBLE: 0}} for e in ENTRY_ORDER}
    other = {"attempts": 0, "qualified_status": {PENDING: 0, FAIL: 0, INELIGIBLE: 0}}
    for order, item in enumerate(attempts):
        ev = item["evaluation"]
        reviews = [validate_review(r, ev) for r in item.get("reviews", [])]
        approved_q = any(r["approves_qualified"] for r in reviews)
        approved_c = any(r["approves_simulation_contact"] for r in reviews)
        entry, seed, split = ev.get("entry_id"), ev.get("seed"), ev.get("split")
        key = (entry, seed)
        status = ev["qualified_mission_success"]["status"]
        bucket = tallies.get(entry, other)
        bucket["attempts"] += 1
        bucket["qualified_status"][status] = bucket["qualified_status"].get(status, 0) + 1
        if entry in tallies and split in SPLITS:
            tallies[entry][split] += 1
        reasons = []
        if status != PENDING:
            reasons.append(f"evaluation_{status}")
        if not approved_q:
            reasons.append("no_valid_independent_approval")
        if split != "holdout":
            reasons.append(f"split:{split}")
        if key in training_seeds:
            reasons.append("seed_used_in_training:" + ",".join(map(str, training_seeds[key])))
        if split == "holdout":
            if key in holdout_seen:
                reasons.append(f"holdout_seed_reused_after:{holdout_seen[key]}")
            else:
                holdout_seen[key] = ev.get("run_id")
            series = ev.get("series_id")
            hashes = (json.dumps(ev.get("code_sha256"), sort_keys=True), ev.get("config_sha256"))
            frozen.setdefault(series, hashes)
            if frozen[series] != hashes:
                reasons.append("code_or_config_changed_within_series")
        phase = "random" if current_index >= len(ENTRY_ORDER) else ENTRY_ORDER[current_index]
        if phase != "random" and entry != phase:
            reasons.append(f"out_of_protocol_order:current_entry={phase}")
        counted = not reasons
        if counted:
            if phase == "random":
                random_successes += 1
            else:
                counts[entry] += 1
                tallies[entry]["counted_successes"] += 1
                if counts[entry] >= SUCCESSES_PER_ENTRY:
                    current_index += 1
        rows.append({"order": order, "run_id": ev.get("run_id"), "entry_id": entry, "seed": seed,
                     "split": split, "series_id": ev.get("series_id"),
                     "code_sha256": ev.get("code_sha256"), "config_sha256": ev.get("config_sha256"),
                     "simulation_contact_success": ev["simulation_contact_success"]["status"],
                     "manufacturer_profile_matches": ev["manufacturer_profile_matches"]["status"],
                     "qualified_mission_success": status,
                     "review_approved_contact": approved_c, "review_approved_qualified": approved_q,
                     "reviews": reviews, "counted": counted, "not_counted_reasons": reasons})
    phase = "random" if current_index >= len(ENTRY_ORDER) else ENTRY_ORDER[current_index]
    return {"contract_version": CONTRACT_VERSION, "consumer_restriction": CONSUMER_RESTRICTION,
            "attempts": rows, "entries": {str(k): v for k, v in tallies.items()}, "unassigned_entry": other,
            "protocol": {"current_entry": phase, "successes_per_entry": SUCCESSES_PER_ENTRY,
                         "counted_by_entry": {str(k): v for k, v in counts.items()},
                         "random_phase_successes": random_successes},
            "note": "집계는 평가 결과와 검증된 독립 리뷰에서만 계산한다. 실패·판정불가 시도를 모두 보존한다"}
