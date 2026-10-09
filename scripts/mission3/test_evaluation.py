"""미션3 평가기 골든 사례. 합성 실행 로그를 임시 디렉터리에 쓰고 실제 로더로 읽는다.

실행: cd scripts && python -m unittest mission3.test_evaluation
"""
from copy import deepcopy
import json
import re
import tempfile
import unittest
from pathlib import Path

from .evaluation import (FAIL, INELIGIBLE, PASS, PENDING, REQUIRED_PROFILE_FIELDS, build_ledger,
                         evaluate_run, evaluation_digest, review_template, score_points, validate_review)

FULL_PROFILE = {f: {"value": 1.0, "source_class": "manufacturer", "source_ref": "synthetic test reference"}
                for f in REQUIRED_PROFILE_FIELDS}
NULL_PROFILE = {f: {"value": None, "source_class": "unresolved", "source_ref": ""} for f in REQUIRED_PROFILE_FIELDS}
FEET = ["foot_1", "foot_2", "foot_3", "foot_4"]
T_TOUCHDOWN, T_OFF, T_END, DT = 10.0, 10.5, 20.0, 0.01


def write_run(root, name="run", manifest=None, truth_fn=None, motors_fn=None, frames=None,
              settings=None, drop=(), offset=(0.012, 0.0), end=T_END):
    run = Path(root) / name
    run.mkdir(parents=True)
    base = {"schema": "mission3-run/1", "run_id": name, "run_kind": "mission3_closed_loop",
            "evidence_level": "L2", "physics_contact": True, "completed": True, "author": "main",
            "entry_id": 1, "seed": 101, "split": "holdout", "series_id": "S1",
            "code_sha256": {"scripts/mission3/controller.py": "c0de"}, "config_sha256": "cf9",
            "mission_timeout_s": 240.0, "vehicle_nominal_speed_m_s": 0.25, "descent_phases": ["DESCEND"],
            "controller_in_loop": True, "started_from": "HANDOFF_IN",
            "aircraft_center_definition": "synthetic body origin", "aircraft_profile": NULL_PROFILE}
    base.update(manifest or {})
    files = {"run_manifest.json": json.dumps(base)}
    truth, motors = [], []
    for i in range(int(round(end / DT)) + 1):
        t = round(i * DT, 6)
        pad = [5.0 + 0.25 * t, 6.25, 0.30]
        landed = t >= T_TOUCHDOWN
        row = {"t": t, "pad": {"center_world_m": pad},
               "aircraft": {"center_world_m": [pad[0] + offset[0], pad[1] + offset[1], 0.33 if landed else 0.6]},
               "vehicle": {"velocity_world_m_s": [0.25, 0.0, 0.0], "in_turn": False},
               "contacts": [{"aircraft_part": f, "other": "pad"} for f in FEET] if landed else []}
        row = truth_fn(t, row) if truth_fn else row
        if row is not None:
            truth.append(json.dumps(row))
        out = [30000, 30000, 30000, 30000] if t < T_OFF else [0, 0, 0, 0]
        out = motors_fn(t, out) if motors_fn else out
        motors.append(json.dumps({"t": t, "motor_output": out}))
    files["truth.jsonl"] = "\n".join(truth)
    files["motors.jsonl"] = "\n".join(motors)
    files["commands.jsonl"] = "\n".join(json.dumps({"t": round(i * 0.1, 3), "phase":
                                                    "WATCH" if i < 80 else "DESCEND" if i < 105 else "MOTOR_OFF"})
                                         for i in range(int(end * 10) + 1))
    frames = frames if frames is not None else [{"t": round(i * 0.05, 3), "width": 320, "height": 320,
                                                 "encoding": "mono8", "channels": 1} for i in range(200)]
    files["camera_frames.jsonl"] = "\n".join(json.dumps(f) for f in frames)
    files["controller-settings.json"] = json.dumps(settings if settings is not None else
                                                   {"control": {"max_speed_m_s": 0.7, "descent_speed_m_s": 0.035}})
    for file_name, text in files.items():
        if file_name not in drop:
            (run / file_name).write_text(text + "\n")
    return run


def signed_review(evaluation, reviewer="dirac-independent", contact=True, qualified=True):
    review = review_template(evaluation, reviewer)
    review.pop("generated_by")
    review.update(review_id=f"rv-{evaluation['run_id']}", reviewed_at_kst="2026-10-08T03:00:00+09:00",
                  approved={"simulation_contact_success": contact, "qualified_mission_success": qualified})
    return review


def with_identity(evaluation, run_id, seed, entry=1, split="holdout", code=None):
    copy = deepcopy(evaluation)
    copy.update(run_id=run_id, seed=seed, entry_id=entry, split=split)
    if code is not None:
        copy["code_sha256"] = code
    return copy


class Evaluation(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def run_eval(self, name, **kwargs):
        return evaluate_run(write_run(self.root, name, **kwargs))

    def assertReason(self, verdict, pattern):
        reasons = verdict["fail_reasons"] + verdict["ineligible_reasons"]
        self.assertTrue(any(re.search(pattern, r) for r in reasons), reasons)

    # 정상 접촉과 세 플래그 분리
    def test_contact_pass_but_null_profile_is_not_qualified(self):
        ev = self.run_eval("nominal")
        self.assertEqual(ev["simulation_contact_success"]["status"], PASS)
        self.assertEqual(ev["manufacturer_profile_matches"]["status"], FAIL)
        self.assertReason(ev["manufacturer_profile_matches"], r"profile_null:assembled_mass_kg")
        self.assertEqual(ev["qualified_mission_success"]["status"], FAIL)
        self.assertEqual(ev["score"]["points"], 9)
        self.assertAlmostEqual(ev["checks"]["motors_off"]["motor_off_time_s"], T_OFF)
        self.assertFalse(ev["physical_success_counted"])
        self.assertIn("never an input to controller", ev["consumer_restriction"])

    def test_full_manufacturer_profile_reaches_only_pending_review(self):
        ev = self.run_eval("full", manifest={"aircraft_profile": FULL_PROFILE})
        self.assertEqual(ev["manufacturer_profile_matches"]["status"], PASS)
        self.assertEqual(ev["qualified_mission_success"]["status"], PENDING)
        self.assertNotEqual(ev["qualified_mission_success"]["status"], PASS)

    def test_conditional_sensitivity_parameters_are_not_manufacturer_matched(self):
        profile = deepcopy(FULL_PROFILE)
        profile["assembled_inertia_kg_m2"]["source_class"] = "conditional_sensitivity"
        ev = self.run_eval("conditional", manifest={"aircraft_profile": profile})
        self.assertEqual(ev["simulation_contact_success"]["status"], PASS)
        self.assertEqual(ev["manufacturer_profile_matches"]["status"], FAIL)
        self.assertReason(ev["manufacturer_profile_matches"], r"profile_source:assembled_inertia")
        self.assertEqual(ev["qualified_mission_success"]["status"], FAIL)

    # 접촉 부정·실패 골든 사례
    def test_floor_contact_during_approach_fails(self):
        def floor(t, row):
            if abs(t - 5.0) < 1e-9:
                row["contacts"] = [{"aircraft_part": "foot_2", "other": "floor"}]
            return row
        ev = self.run_eval("floor", truth_fn=floor)
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)
        self.assertReason(ev["checks"]["collision"], r"floor_contact:foot_2")

    def test_marker_board_collision_fails(self):
        def board(t, row):
            if abs(t - 9.9) < 1e-9:
                row["contacts"] = [{"aircraft_part": "body", "other": "marker_board"}]
            return row
        ev = self.run_eval("board", truth_fn=board)
        self.assertReason(ev["checks"]["collision"], r"collision:body->marker_board")
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)

    def test_three_feet_on_pad_fails_hold(self):
        def three(t, row):
            if 11.0 <= t <= 11.2:
                row["contacts"] = [c for c in row["contacts"] if c["aircraft_part"] != "foot_4"]
            return row
        ev = self.run_eval("three_feet", truth_fn=three)
        self.assertReason(ev["checks"]["hold_support"], r"foot_not_on_pad:foot_4")
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)

    def test_body_touching_pad_fails(self):
        def body(t, row):
            if t >= 12.0:
                row["contacts"].append({"aircraft_part": "body", "other": "pad"})
            return row
        ev = self.run_eval("body_on_pad", truth_fn=body)
        self.assertReason(ev["checks"]["collision"], r"non_foot_pad_contact:body")

    def test_motor_drop_before_touchdown_fails(self):
        ev = self.run_eval("drop", motors_fn=lambda t, out: [0, 0, 0, 0] if t >= 9.5 else out)
        self.assertReason(ev["checks"]["motors_off"], r"motors_off_before_touchdown")
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)

    def test_motors_restarted_and_kept_running_fails(self):
        ev = self.run_eval("rethrottle", motors_fn=lambda t, out: [0, 0, 0, 1] if t >= 12.0 else out)
        self.assertReason(ev["checks"]["motors_off"], r"motors_not_off_for_hold")

    def test_log_ending_before_three_seconds_is_ineligible(self):
        ev = self.run_eval("short", end=13.0)
        self.assertEqual(ev["simulation_contact_success"]["status"], INELIGIBLE)
        self.assertReason(ev["checks"]["motors_off"], r"motor_log_ends_before_hold_complete")

    def test_vehicle_stopping_under_aircraft_fails(self):
        def stop(t, row):
            if t >= 11.0:
                row["vehicle"]["velocity_world_m_s"] = [0.0, 0.0, 0.0]
            return row
        ev = self.run_eval("vehicle_stop", truth_fn=stop)
        self.assertReason(ev["checks"]["vehicle_motion"], r"vehicle_speed_deviation")
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)

    def test_turn_during_descent_fails_mission_but_not_contact(self):
        def turn(t, row):
            if 8.5 <= t < 9.0:
                row["vehicle"]["in_turn"] = True
            return row
        ev = self.run_eval("turn", manifest={"aircraft_profile": FULL_PROFILE}, truth_fn=turn)
        self.assertEqual(ev["simulation_contact_success"]["status"], PASS)
        self.assertReason(ev["checks"]["descent_straight"], r"vehicle_in_turn_between_descent_and_hold_end")
        self.assertEqual(ev["qualified_mission_success"]["status"], FAIL)

    def test_mission_time_budget(self):
        ev = self.run_eval("late", manifest={"aircraft_profile": FULL_PROFILE, "mission_timeout_s": 12.0})
        self.assertReason(ev["checks"]["mission_time"], r"mission_time_exceeded")
        self.assertEqual(ev["qualified_mission_success"]["status"], FAIL)

    # 카메라·격리·로그 누락
    def test_resized_or_wrong_camera_frames_fail(self):
        frames = [{"t": 0.0, "width": 320, "height": 320, "encoding": "mono8", "channels": 1},
                  {"t": 0.05, "width": 324, "height": 244, "encoding": "mono8", "channels": 1},
                  {"t": 0.10, "width": 320, "height": 320, "encoding": "mono8", "channels": 1, "resized": True}]
        ev = self.run_eval("camera", frames=frames)
        self.assertEqual(ev["checks"]["camera_contract"]["status"], FAIL)
        self.assertReason(ev["checks"]["camera_contract"], r"camera_contract_violations:2/3")

    def test_plant_parameters_in_controller_settings_fail_isolation(self):
        for i, settings in enumerate(({"vehicle_plant_only": {"speed_m_s": 0.25}},
                                      {"vehicle": {"speed_m_s": 0.25}},
                                      {"control": {"initial_arc_m": 3.0}})):
            ev = self.run_eval(f"leak{i}", manifest={"aircraft_profile": FULL_PROFILE}, settings=settings)
            self.assertEqual(ev["checks"]["controller_isolation"]["status"], FAIL)
            self.assertEqual(ev["qualified_mission_success"]["status"], FAIL)
        ok = self.run_eval("no_leak", settings={"control": {"max_speed_m_s": 0.7, "coarse_approach_speed_m_s": 0.4}})
        self.assertEqual(ok["checks"]["controller_isolation"]["status"], PASS)

    def test_missing_motor_log_is_ineligible(self):
        ev = self.run_eval("no_motors", drop=("motors.jsonl",))
        self.assertEqual(ev["simulation_contact_success"]["status"], INELIGIBLE)
        self.assertReason(ev["checks"]["log_integrity"], r"missing_log:motors.jsonl")

    def test_truth_gap_is_ineligible(self):
        ev = self.run_eval("gap", truth_fn=lambda t, row: None if 11.0 < t < 11.1 else row)
        self.assertReason(ev["checks"]["log_integrity"], r"truth_gap")
        self.assertEqual(ev["simulation_contact_success"]["status"], INELIGIBLE)

    def test_incomplete_run_and_missing_manifest_are_ineligible(self):
        ev = self.run_eval("running", manifest={"completed": False})
        self.assertReason(ev["checks"]["manifest"], r"run_not_completed")
        self.assertEqual(ev["simulation_contact_success"]["status"], INELIGIBLE)
        empty = self.root / "legacy_l1"
        empty.mkdir()
        legacy = evaluate_run(empty)
        self.assertEqual(legacy["qualified_mission_success"]["status"], INELIGIBLE)
        self.assertFalse(legacy["physical_success_counted"])

    # 증거 등급과 fixture
    def test_optical_geometry_run_is_never_contact_success(self):
        ev = self.run_eval("l1", manifest={"run_kind": "optical_geometry", "evidence_level": "L1",
                                           "physics_contact": False},
                           truth_fn=lambda t, row: {**row, "contacts": []}, drop=("motors.jsonl",))
        self.assertEqual(ev["simulation_contact_success"]["status"], FAIL)
        self.assertReason(ev["simulation_contact_success"], r"no_physics_contact_evidence:L1")

    def test_contact_fixture_pass_is_not_mission3_success(self):
        ev = self.run_eval("fixture", manifest={"run_kind": "contact_fixture", "entry_id": None,
                                                "aircraft_profile": FULL_PROFILE})
        self.assertEqual(ev["simulation_contact_success"]["status"], PASS)
        self.assertReason(ev["qualified_mission_success"], r"contact_fixture_not_mission3")
        ledger = build_ledger([{"evaluation": ev, "reviews": [signed_review(ev)]}])
        self.assertFalse(ledger["attempts"][0]["counted"])

    # 점수 경계
    def test_score_boundaries(self):
        cases = [(0.0, 10), (0.0099999, 10), (0.01, 9), (0.015, 9), (0.03, 7), (0.09, 1), (0.0999, 1),
                 (0.1, 0), (0.1000001, 0), (2 ** 0.5 * 0.1, 0)]
        for distance, points in cases:
            self.assertEqual(score_points(distance), points, distance)
        for bad in (-0.001, float("nan"), float("inf"), True, None):
            with self.assertRaises(ValueError):
                score_points(bad)
        corner = self.run_eval("corner", offset=(0.1, 0.1))
        self.assertEqual(corner["score"]["points"], 0)

    # 리뷰
    def test_review_rules(self):
        ev = self.run_eval("reviewed", manifest={"aircraft_profile": FULL_PROFILE})
        self.assertTrue(validate_review(signed_review(ev), ev)["approves_qualified"])
        self.assertIn("self_review", validate_review(signed_review(ev, reviewer="main"), ev)["reasons"])
        self.assertIn("automated_or_empty_reviewer", validate_review(signed_review(ev, reviewer="evaluator"), ev)["reasons"])
        template = review_template(ev, "dirac-independent")
        template["approved"] = {"simulation_contact_success": True, "qualified_mission_success": True}
        self.assertIn("unsigned_template", validate_review(template, ev)["reasons"])
        wrong = signed_review(ev)
        wrong["config_sha256"] = "other"
        self.assertIn("config_sha256_mismatch", validate_review(wrong, ev)["reasons"])
        review = signed_review(ev)
        tampered = deepcopy(ev)
        tampered["score"]["points"] = 10
        self.assertIn("evaluation_sha256_mismatch", validate_review(review, tampered)["reasons"])
        failed = self.run_eval("failed", truth_fn=lambda t, row: row if t < 12 else {**row, "contacts": []})
        self.assertIn("approves_unqualified_mission", validate_review(signed_review(failed), failed)["reasons"])
        self.assertEqual(review_template(ev, "x")["approved"],
                         {"simulation_contact_success": False, "qualified_mission_success": False})

    # 집계
    def test_ledger_protocol_denominators_and_no_override(self):
        base = self.run_eval("ledger_base", manifest={"aircraft_profile": FULL_PROFILE})
        failed = self.run_eval("ledger_fail", manifest={"aircraft_profile": FULL_PROFILE},
                               truth_fn=lambda t, row: row if abs(t - 5.0) > 1e-9 else
                               {**row, "contacts": [{"aircraft_part": "foot_1", "other": "floor"}]})
        attempts = []

        def add(ev, approve=True, **extra):
            attempts.append({"evaluation": ev, "reviews": [signed_review(ev)] if approve else [], **extra})

        add(with_identity(base, "train-1", 300, split="training"))
        add(with_identity(failed, "h-fail", 199))
        add(with_identity(base, "h-unreviewed", 198), approve=False, counted=True, success_count=99)
        add(with_identity(base, "h-train-seed", 300))
        for seed in range(201, 206):
            add(with_identity(base, f"h-{seed}", seed))
        add(with_identity(base, "h-dup", 201, entry=2))
        add(with_identity(base, "h-205-entry2", 205, entry=2))
        add(with_identity(base, "h-entry3", 401, entry=3))
        add(with_identity(base, "h-code", 402, entry=2, code={"scripts/mission3/controller.py": "changed"}))
        add(with_identity(base, "h-entry2", 403, entry=2))
        add(with_identity(base, "h-403-again", 403, entry=2))
        ledger = build_ledger(attempts)
        rows = {r["run_id"]: r for r in ledger["attempts"]}
        self.assertEqual(len(ledger["attempts"]), len(attempts))
        self.assertEqual(ledger["protocol"]["counted_by_entry"]["1"], 5)
        # 같은 seed라도 진입점이 다르면 다른 시나리오다. 같은 (진입점, seed)의 두 번째 holdout은 세지 않는다.
        self.assertEqual(ledger["protocol"]["counted_by_entry"]["2"], 3)
        self.assertIn("holdout_seed_reused_after:h-entry2", rows["h-403-again"]["not_counted_reasons"])
        self.assertEqual(ledger["protocol"]["current_entry"], 2)
        self.assertFalse(rows["h-unreviewed"]["counted"])
        self.assertIn("no_valid_independent_approval", rows["h-unreviewed"]["not_counted_reasons"])
        self.assertTrue(any(r.startswith("seed_used_in_training") for r in rows["h-train-seed"]["not_counted_reasons"]))
        self.assertIn("evaluation_fail", rows["h-fail"]["not_counted_reasons"])
        self.assertIn("out_of_protocol_order:current_entry=2", rows["h-entry3"]["not_counted_reasons"])
        self.assertIn("code_or_config_changed_within_series", rows["h-code"]["not_counted_reasons"])
        self.assertTrue(rows["h-dup"]["counted"])
        entry1 = ledger["entries"]["1"]
        self.assertEqual(entry1["attempts"], 9)
        self.assertEqual(entry1["qualified_status"][FAIL], 1)
        self.assertEqual(entry1["training"], 1)

    def test_entries_advance_in_order_then_random(self):
        base = self.run_eval("order_base", manifest={"aircraft_profile": FULL_PROFILE})
        attempts, seed = [], 1000
        for entry in range(1, 9):
            for _ in range(5):
                seed += 1
                ev = with_identity(base, f"e{entry}-{seed}", seed, entry=entry)
                attempts.append({"evaluation": ev, "reviews": [signed_review(ev)]})
        ev = with_identity(base, "random-1", 9999, entry=4)
        attempts.append({"evaluation": ev, "reviews": [signed_review(ev)]})
        ledger = build_ledger(attempts)
        self.assertEqual(ledger["protocol"]["current_entry"], "random")
        self.assertEqual(ledger["protocol"]["random_phase_successes"], 1)
        self.assertTrue(all(v == 5 for v in ledger["protocol"]["counted_by_entry"].values()))

    def test_evaluator_does_not_import_control_modules(self):
        source = (Path(__file__).with_name("evaluation.py")).read_text(encoding="utf-8")
        for module in ("controller", "estimator", "vision", "scene", "settings", "track", "firmware"):
            self.assertIsNone(re.search(rf"^\s*(from\s+\.{module}|import\s+.*{module})", source, re.M), module)

    def test_digest_is_stable(self):
        ev = self.run_eval("digest")
        self.assertEqual(evaluation_digest(ev), evaluation_digest(json.loads(json.dumps(ev))))


if __name__ == "__main__":
    unittest.main()
