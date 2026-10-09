#!/usr/bin/env python3
"""읽기 전용 series aggregator 독립 검사. 발견한 계약 위반은 실패로 보존한다.

합성 offline 로그/리뷰만 own artifact에 쓴다. 실제 미션/승인/제조사 근거가 아니다.
Aggregator source는 실행 시작에 복사해 고정하고 shared 파일은 수정하지 않는다.
"""
from copy import deepcopy
from datetime import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import types
import unittest
from zoneinfo import ZoneInfo

from mission3.evaluation import (REQUIRED_PROFILE_FIELDS, evaluation_digest, review_template)

ROOT=Path(__file__).resolve().parents[1]
OUT=Path(os.environ.get("MISSION3_SERIES_SUMMARY_TEST_OUT",str(ROOT/"artifacts/mission3/series-summary-review"/
    datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f-tests")))).resolve()
FROZEN_SOURCE=None
INPUT_HASHES={}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def json_bytes(value):return (json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode()
def dump(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(json_bytes(value))


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.case=OUT/self._testMethodName;self.case.mkdir(parents=True,exist_ok=False)
        self.root=self.case/"fake-project";self.root.mkdir()
        self.module=types.ModuleType("frozen_summary_under_review")
        self.module.__file__=str(OUT/"source-snapshot/scripts/summarize_mission3_series.py")
        exec(compile(FROZEN_SOURCE,self.module.__file__,"exec"),self.module.__dict__)
        self.module.ROOT=self.root
        self.code={"scripts/mission3/controller.py":hashlib.sha256(b"synthetic controller source\n").hexdigest()}
        self.libraries={"build/mission3-flight-controller/libmission3_controller.so":hashlib.sha256(b"synthetic library; never loaded").hexdigest()}
        self.settings={"mission":{"camera":{"width":320,"height":320,"encoding":"mono8"},
                       "arena":{},"track":{},"marker":{},"landing_geometry":{},"entry":{},
                       "observer":{},"control":{"max_speed_m_s":.7}},
                       "physical":{"test_double":True},"strict_focus":True}
        self.config_sha=hashlib.sha256(json_bytes(self.settings)).hexdigest()
        self.plan_path=self.case/"plan.json"
        self.plan={"schema":"mission3-series/1","series_id":"synthetic-summary-review","entry_id":1,
            "split":"holdout","seeds_unused_at_entry":True,"seeds":[201,202,203,204,205],
            "runtime_identity":{"code_sha256":self.code,"runtime_library_sha256":self.libraries},
            "config_sha256":self.config_sha,"config_identity_file":"frozen-experiment-settings.json",
            "root":str(self.root),"output":str(self.case),"created_at_utc":"2026-10-07T22:35:00+00:00",
            "profile_frozen":str(self.case/"frozen-profile.json"),"jobs":[]}
        dump(self.case/"frozen-experiment-settings.json",self.settings)
        dump(self.case/"frozen-profile.json",self.settings["physical"])
        self.plan["frozen_profile_sha256"]=sha(self.case/"frozen-profile.json")
        coordinator=self.case/"management-source/run_mission3_series.py"
        coordinator.parent.mkdir();coordinator.write_text("# synthetic frozen coordinator; never executed\n")
        self.plan["coordinator_sha256"]=sha(coordinator)
        for name,data in (("scripts/mission3/controller.py",b"synthetic controller source\n"),
                          ("build/mission3-flight-controller/libmission3_controller.so",b"synthetic library; never loaded")):
            p=self.case/"source-snapshot"/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        self.runs=[];self.reviews=[]
        dump(self.root/"fixture-provenance.json",{"schema":"mission3-summary-test-fixture/1",
            "test_double":True,"scope":"synthetic offline evaluator logs only; no physical flight",
            "producer_sha256":sha(Path(__file__)),"fixture_root":str(self.root),"test_case":self._testMethodName})

    def write_run(self,seed,*,split="holdout",qualified=False,contact=True):
        run=self.root/"artifacts/mission3/physical"/f"synthetic-entry1-seed{seed}"
        run.mkdir(parents=True,exist_ok=False)
        profile={name:{"value":1. if qualified else None,
            "source_class":"manufacturer" if qualified else "unresolved",
            "source_ref":"SYNTHETIC test reference only" if qualified else ""} for name in REQUIRED_PROFILE_FIELDS}
        manifest={"schema":"mission3-run/1","run_id":run.name,"test_double":True,"author":"fixture-author",
            "run_kind":"mission3_closed_loop","evidence_level":"L2","physics_contact":True,"completed":True,
            "entry_id":1,"seed":seed,"split":split,"series_id":self.plan["series_id"],
            "code_sha256":self.code,"runtime_library_sha256":self.libraries,"config_sha256":self.config_sha,
            "config_identity_file":"frozen-experiment-settings.json","mission_timeout_s":10.,
            "vehicle_nominal_speed_m_s":.25,"descent_phases":["DESCEND"],"controller_in_loop":True,
            "started_from":"HANDOFF_IN","aircraft_center_definition":"synthetic offline fixture origin",
            "aircraft_profile":profile,
            "termination_reason":"hold_window_elapsed" if contact else "synthetic_completed_landing_failure"}
        dump(run/"adapter-summary.json",{"failure":None if contact else "synthetic_completed_landing_failure",
                                         "test_double":True})
        dump(run/"frozen-experiment-settings.json",self.settings)
        trial={**self.settings,"entry_id":1,"seed":seed};dump(run/"experiment.json",trial)
        manifest["trial_metadata_sha256"]=sha(run/"experiment.json")
        dump(run/"run_manifest.json",manifest)
        for name,data in (("scripts/mission3/controller.py",b"synthetic controller source\n"),
                          ("build/mission3-flight-controller/libmission3_controller.so",b"synthetic library; never loaded")):
            path=run/"source-snapshot"/name;path.parent.mkdir(parents=True);path.write_bytes(data)
        public=deepcopy(self.settings["mission"]);public["control"]["terminal_policy"]="last_fix_bridge"
        dump(run/"controller-settings.json",public)
        truth=[];motors=[];commands=[];frames=[]
        for i in range(401):
            t=i/100.;pad=[5.+.25*t,6.25,.30];landed=t>=.5 and contact
            truth.append({"t":t,"pad":{"center_world_m":pad},
                "aircraft":{"center_world_m":[pad[0]+.012,pad[1],.33 if landed else .7]},
                "vehicle":{"velocity_world_m_s":[.25,0.,0.],"in_turn":False},
                "contacts":[{"aircraft_part":f"foot_{f}","other":"pad"} for f in range(1,5)] if landed else []})
            motors.append({"t":t,"motor_output":[30000]*4 if t<.75 else [0]*4})
            commands.append({"t":t,"phase":"WATCH" if t<.25 else "DESCEND" if t<.75 else "MOTOR_OFF"})
            if i%5==0:frames.append({"t":t,"width":320,"height":320,"encoding":"mono8","channels":1})
        for name,rows in (("truth.jsonl",truth),("motors.jsonl",motors),("commands.jsonl",commands),("camera_frames.jsonl",frames)):
            (run/name).write_text("\n".join(json.dumps(row) for row in rows)+"\n")
        self.plan["jobs"].append({"job_id":f"{len(self.runs)+1:03d}-entry1-seed{seed}",
            "seed":seed,"entry_id":1,"output":str(run),
            "trial_metadata_sha256":manifest["trial_metadata_sha256"]})
        self.runs.append(run);return run

    def sign_fixture(self,run,*,qualified=False):
        evaluation=self.module.canonical_run(run)
        dump(run/"fixture-evaluation.json",evaluation)
        review=review_template(evaluation,"synthetic-independent-reviewer")
        review.pop("generated_by")
        review.update(review_id="synthetic-"+run.name,reviewed_at_kst="2026-10-08T07:35:00+09:00",
            approved={"simulation_contact_success":evaluation["simulation_contact_success"]["status"]=="pass",
                      "qualified_mission_success":qualified},
            findings=["SYNTHETIC TEST DATA; never an actual independent approval"],test_double=True)
        path=self.root/"artifacts/mission3/independent-review"/run.name/"signedreview.json"
        dump(path,review);self.reviews.append(path);return path

    def make_series(self,*,split="holdout",qualified=False,fail_seed=None):
        for seed in self.plan["seeds"]:
            run=self.write_run(seed,split=split,qualified=qualified,contact=seed!=fail_seed)
            self.sign_fixture(run,qualified=qualified and seed!=fail_seed)
        self.save_plan()

    def save_plan(self):
        dump(self.plan_path,self.plan)
        self.plan_path.with_name("plan.sha256").write_text(sha(self.plan_path)+"\n")
        states=[]
        for job in self.plan["jobs"]:
            manifest=json.loads((Path(job["output"])/"run_manifest.json").read_text())
            failed=manifest.get("termination_reason")!="hold_window_elapsed"
            states.append({**deepcopy(job),"status":"worker_failed" if failed else "worker_succeeded",
                "returncode":2 if failed else 0,"manifest_check":{"valid":True,
                "termination_reason":manifest.get("termination_reason"),"manifest_sha256":sha(Path(job["output"])/"run_manifest.json")}})
        dump(self.case/"state.json",{"schema":"mission3-series/1","series_id":self.plan["series_id"],
            "plan_sha256":sha(self.plan_path),"complete":True,"stop_reason":None,"events":[],
            "jobs":states})

    def mutate_manifest(self,index,**values):
        path=self.runs[index]/"run_manifest.json";data=json.loads(path.read_text());data.update(values);dump(path,data)

    def summarize(self):
        old_cwd=Path.cwd()
        try:result=self.module.summarize(self.plan_path)
        except Exception as exc:
            dump(self.case/"observed-exception.json",{"type":type(exc).__name__,"error":str(exc)})
            raise
        finally:os.chdir(old_cwd)
        dump(self.case/"observed-report.json",result);return result

    def test_five_conditional_reviews_gate_true_formal_zero(self):
        self.make_series();result=self.summarize()
        self.assertEqual(result["scheduled"],5);self.assertEqual(result["completed"],5)
        self.assertEqual(result["conditional_contact_passes"],5)
        self.assertEqual(result["independently_approved_contact_passes"],5)
        self.assertEqual(result["formal_qualified_count"],0)
        self.assertTrue(result["conditional_research_gate_passed"])

    def test_pending_and_missing_attempts_preserve_scheduled_denominator(self):
        self.make_series();self.mutate_manifest(3,completed=False)
        (self.runs[4]/"run_manifest.json").unlink()
        result=self.summarize()
        self.assertEqual((result["scheduled"],result["completed"],len(result["pending"])),(5,3,2))
        self.assertEqual(len(result["immutable_ledger"]["attempts"]),3)
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_completed_contact_failure_stays_in_denominator(self):
        self.make_series(fail_seed=205);result=self.summarize()
        self.assertEqual((result["scheduled"],result["completed"],result["conditional_contact_passes"]),(5,5,4))
        self.assertEqual(result["completed_attempts"][-1]["contact_status"],"fail")
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_training_manifest_never_promoted(self):
        self.make_series(split="training",qualified=True);result=self.summarize()
        self.assertEqual(result["formal_qualified_count"],0)
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_reused_plan_flag_blocks_conditional_gate(self):
        self.make_series();self.plan["seeds_unused_at_entry"]=False;self.save_plan()
        self.assertFalse(self.summarize()["conditional_research_gate_passed"])

    def test_duplicate_scheduled_seed_blocks_gate(self):
        self.make_series();self.plan["jobs"][-1]=deepcopy(self.plan["jobs"][0]);self.save_plan()
        result=self.summarize();self.assertEqual(result["scheduled"],5)
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_missing_review_blocks_gate(self):
        self.make_series();self.reviews[-1].unlink();result=self.summarize()
        self.assertEqual(result["independently_approved_contact_passes"],4)
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_manifest_code_config_library_mismatch_blocks_gate(self):
        self.make_series();original=json.loads((self.runs[0]/"run_manifest.json").read_text())
        for key,value in (("code_sha256",{"other":"different"}),("config_sha256","different"),
                          ("runtime_library_sha256",{"other":"different"})):
            with self.subTest(key=key):
                dump(self.runs[0]/"run_manifest.json",{**original,key:value})
                self.sign_fixture(self.runs[0]);result=self.summarize()
                dump(self.case/f"declared-{key}-mismatch-report.json",result)
                self.assertFalse(result["conditional_research_gate_passed"])
                self.assertIn(key,result["identity_mismatches"][0]["keys"])

    def test_declared_entry_and_seed_mismatch_blocks_gate(self):
        self.make_series();original=json.loads((self.runs[0]/"run_manifest.json").read_text())
        for key,value in (("entry_id",2),("seed",999)):
            with self.subTest(key=key):
                dump(self.runs[0]/"run_manifest.json",{**original,key:value})
                self.sign_fixture(self.runs[0]);result=self.summarize()
                dump(self.case/f"declared-{key}-mismatch-report.json",result)
                self.assertFalse(result["conditional_research_gate_passed"])
                self.assertIn(key,result["identity_mismatches"][0]["keys"])

    def test_unsigned_self_review_and_hash_mismatches_are_rejected(self):
        self.make_series();path=self.reviews[0];original=json.loads(path.read_text())
        changes=[{"generated_by":"evaluate_mission3 review template (unsigned)"},
            {"reviewer":{"id":"fixture-author","role":"independent_reviewer"}},
            {"reviewer":{"id":"evaluator","role":"independent_reviewer"}},
            {"code_sha256":{"wrong":"hash"}},{"config_sha256":"wrong"},
            {"evaluator_sha256":"wrong"},{"evaluation_sha256":"wrong"},
            {"review_id":None},{"approved":{"simulation_contact_success":True,"qualified_mission_success":True}}]
        for i,change in enumerate(changes):
            with self.subTest(change=change):
                dump(path,{**original,**change});result=self.summarize()
                dump(self.case/f"review-negative-{i}.json",result)
                self.assertGreaterEqual(result["invalid_reviews"],1)
                self.assertFalse(result["conditional_research_gate_passed"])

    def test_extended_review_fields_must_be_validated_when_present(self):
        self.make_series();path=self.reviews[0];review=json.loads(path.read_text())
        review.update(trial_metadata_sha256="wrong",evaluation_file_sha256="wrong")
        dump(path,review);result=self.summarize()
        self.assertFalse(result["conditional_research_gate_passed"],"extra claimed trial/evaluation-file binding is ignored")

    def test_modified_frozen_config_bytes_must_block_gate(self):
        self.make_series();dump(self.runs[0]/"frozen-experiment-settings.json",{"changed":True})
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"manifest hash declaration is trusted without config bytes")

    def test_modified_source_snapshot_bytes_must_block_gate(self):
        self.make_series();(self.runs[0]/"source-snapshot/scripts/mission3/controller.py").write_text("changed snapshot\n")
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"source snapshot bytes are not verified")

    def test_modified_library_snapshot_bytes_must_block_gate(self):
        self.make_series();(self.runs[0]/"source-snapshot/build/mission3-flight-controller/libmission3_controller.so").write_bytes(b"changed library")
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"library snapshot bytes are not verified")

    def test_modified_trial_metadata_bytes_must_block_gate(self):
        self.make_series();dump(self.runs[0]/"experiment.json",{"seed":999})
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"trial bytes and scheduled trial hash are not verified")

    def test_modified_plan_digest_must_block_gate(self):
        self.make_series();plan=json.loads(self.plan_path.read_text());plan["changed_after_freeze"]=True
        dump(self.plan_path,plan)
        self.assertNotEqual(sha(self.plan_path),self.plan_path.with_name("plan.sha256").read_text().strip())
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"existing plan.sha256 is not checked")

    def test_external_training_seed_history_must_block_gate(self):
        self.make_series();training=self.root/"artifacts/mission3/physical/prior-training/run_manifest.json"
        dump(training,{"schema":"mission3-run/1","run_id":"prior-training","entry_id":1,"seed":201,
            "split":"training","completed":True,"test_double":True})
        self.plan["training_runs"]=[str(training.parent)];self.save_plan()
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"external prior training is absent from local ledger")

    def test_plan_training_split_must_match_run_split(self):
        self.make_series();self.plan["split"]="training";self.save_plan()
        self.assertFalse(self.summarize()["conditional_research_gate_passed"],"plan split is not compared with manifests")

    def test_manifest_series_id_must_match_plan(self):
        self.make_series();self.mutate_manifest(0,series_id="different-series")
        self.sign_fixture(self.runs[0]);result=self.summarize()
        self.assertFalse(result["conditional_research_gate_passed"],"series_id mismatch is ignored")

    def test_coordinator_identity_changed_blocks_gate_after_bytes_restored(self):
        self.make_series()
        # Same final identities and positive offline evaluations/reviews do
        # not erase a coordinator-observed midrun source-change failure.
        for i,job in enumerate(self.plan["jobs"]):
            job["job_id"]=f"{i+1:03d}-entry1-seed{job['seed']}"
        self.save_plan()
        state={"schema":"mission3-series/1","series_id":self.plan["series_id"],
            "plan_sha256":sha(self.plan_path),"stop_reason":"identity_change","complete":True,
            "events":[{"event":"future_launches_aborted","reason":"identity_change",
                       "detail":{"source":"changed during worker; final bytes restored"}},
                      {"event":"worker_finished","job_id":self.plan["jobs"][0]["job_id"],
                       "status":"identity_changed","returncode":0}],
            "jobs":[{**deepcopy(job),"status":"identity_changed" if i==0 else "worker_succeeded",
                     "returncode":0,"manifest_check":{"valid":True},
                     **({"identity_changed_during_run":True} if i==0 else {})}
                    for i,job in enumerate(self.plan["jobs"])]}
        dump(self.case/"state.json",state)
        self.assertEqual(sha(self.runs[0]/"source-snapshot/scripts/mission3/controller.py"),
                         self.code["scripts/mission3/controller.py"])
        result=self.summarize()
        self.assertEqual((result["scheduled"],result["completed"]),(5,5))
        self.assertFalse(result["conditional_research_gate_passed"],
                         "coordinator identity_changed record is ignored after final hashes match")

    def test_formal_count_must_not_promote_known_reused_seeds(self):
        self.make_series(qualified=True);self.plan["seeds_unused_at_entry"]=False;self.save_plan()
        result=self.summarize();self.assertFalse(result["conditional_research_gate_passed"])
        self.assertEqual(result["formal_qualified_count"],0,"local ledger formal count ignores plan's known seed reuse")

    def test_formal_count_must_not_promote_library_mismatch(self):
        self.make_series(qualified=True);self.mutate_manifest(0,runtime_library_sha256={"other":"different"})
        result=self.summarize();self.assertFalse(result["conditional_research_gate_passed"])
        self.assertEqual(result["formal_qualified_count"],0,"local ledger does not bind runtime library identity")

    def test_unrelated_corrupt_review_must_not_erase_report_denominator(self):
        self.make_series();path=self.root/"artifacts/mission3/independent-review/unrelated/signedreview.json"
        path.parent.mkdir();path.write_text("{broken unrelated review")
        result=self.summarize()
        self.assertEqual(result["scheduled"],5)
        self.assertTrue(result["conditional_research_gate_passed"])
        self.assertEqual(len(result["review_read_errors"]),1)

    def test_corrupt_related_review_blocks_gate_and_keeps_denominator(self):
        self.make_series();self.reviews[0].write_text("{broken related review")
        result=self.summarize()
        self.assertEqual(result["scheduled"],5)
        self.assertGreaterEqual(result["invalid_reviews"],1)
        self.assertFalse(result["conditional_research_gate_passed"])

    def test_coordinator_plan_state_and_frozen_inputs_required(self):
        self.make_series();baseline=self.summarize()
        self.assertTrue(baseline["conditional_research_gate_passed"])
        targets=[self.case/"plan.sha256",self.case/"state.json",self.case/"frozen-profile.json",
                 self.case/"frozen-experiment-settings.json",self.case/"source-snapshot/scripts/mission3/controller.py",
                 self.case/"source-snapshot/build/mission3-flight-controller/libmission3_controller.so"]
        for i,path in enumerate(targets):
            with self.subTest(path=path.name):
                original=path.read_bytes();path.unlink()
                try:
                    result=self.summarize();dump(self.case/f"missing-provenance-{i}.json",result)
                    self.assertFalse(result["conditional_research_gate_passed"])
                    self.assertEqual(result["scheduled"],5)
                finally:path.write_bytes(original)

    def test_coordinator_status_job_identity_and_state_digest_bound(self):
        self.make_series();path=self.case/"state.json";original=json.loads(path.read_text())
        variants=[]
        for status in ("timed_out","worker_failed","cancelled_at_cutoff"):
            changed=deepcopy(original);changed["jobs"][0]["status"]=status;variants.append(changed)
        changed=deepcopy(original);changed["plan_sha256"]="wrong";variants.append(changed)
        changed=deepcopy(original);changed["jobs"][0]["output"]="wrong";variants.append(changed)
        changed=deepcopy(original);changed["jobs"][0]["identity_changed_during_run"]=True;variants.append(changed)
        for i,state in enumerate(variants):
            with self.subTest(variant=i):
                dump(path,state);result=self.summarize();dump(self.case/f"state-negative-{i}.json",result)
                self.assertFalse(result["conditional_research_gate_passed"])
                self.assertEqual(result["scheduled"],5)

    def test_coordinator_schema_cannot_be_removed_to_bypass_state(self):
        self.make_series();self.plan.pop("schema")
        self.save_plan();(self.case/"state.json").unlink()
        result=self.summarize()
        self.assertFalse(result["conditional_research_gate_passed"])
        self.assertIn("unsupported_coordinator_plan_schema",result["plan_state_errors"])

    def test_controller_public_settings_bound_to_frozen_trial(self):
        self.make_series();path=self.runs[0]/"controller-settings.json";d=json.loads(path.read_text())
        d["control"]["max_speed_m_s"]=1.1;dump(path,d)
        result=self.summarize()
        self.assertFalse(result["conditional_research_gate_passed"])
        self.assertIn("controller_settings_projection",result["identity_mismatches"][0]["keys"])

    def test_legacy_direct_plan_remains_valid_with_run_snapshot_proof(self):
        self.make_series();coordinated=deepcopy(self.plan)
        legacy={"created_kst":"2026-10-08T07:35:00+09:00","series":self.plan["series_id"],
            "entry":1,"seeds":self.plan["seeds"],"split":"holdout","holdout_seed_previously_used":False,
            "config_sha256":self.config_sha,"code_sha256":self.code,"runtime_library_sha256":self.libraries,
            "runs":[{"seed":j["seed"],"entry":1,"run_dir":j["output"]} for j in coordinated["jobs"]]}
        dump(self.plan_path,legacy);self.plan_path.with_name("plan.sha256").unlink()
        (self.case/"state.json").unlink()
        import shutil
        shutil.rmtree(self.case/"management-source")
        result=self.summarize()
        self.assertTrue(result["conditional_research_gate_passed"])
        self.assertEqual(result["provenance_mode"],"legacy_direct_plan")
        self.assertEqual(result["formal_qualified_count"],0)

    def test_coordinator_planned_job_id_and_trial_digest_are_required(self):
        self.make_series();original=deepcopy(self.plan)
        for key in ("job_id","trial_metadata_sha256"):
            with self.subTest(key=key):
                self.plan=deepcopy(original);self.plan["jobs"][0].pop(key);self.save_plan()
                result=self.summarize();dump(self.case/f"missing-job-{key}.json",result)
                self.assertFalse(result["conditional_research_gate_passed"])
                self.assertEqual(result["scheduled"],5)

    def test_ten_completed_with_five_approved_and_five_preserved_landing_failures_pass_gate(self):
        self.plan["seeds"]=list(range(201,211))
        for seed in self.plan["seeds"]:
            run=self.write_run(seed,contact=seed<=205);self.sign_fixture(run)
        self.save_plan();result=self.summarize()
        self.assertEqual((result["scheduled"],result["completed"]),(10,10))
        self.assertEqual(result["conditional_contact_passes"],5)
        self.assertEqual(result["independently_approved_contact_passes"],5)
        self.assertEqual(sum(a["contact_status"]=="fail" for a in result["completed_attempts"]),5)
        self.assertEqual(result["plan_state_errors"],[])
        self.assertTrue(result["conditional_research_gate_passed"])
        self.assertEqual(result["formal_qualified_count"],0)

    def test_exit_two_without_matching_completed_failure_output_blocks_gate(self):
        self.make_series();path=self.case/"state.json";d=json.loads(path.read_text())
        d["jobs"][0].update(status="worker_failed",returncode=2);dump(path,d)
        result=self.summarize()
        self.assertFalse(result["conditional_research_gate_passed"])
        self.assertEqual(result["scheduled"],5)

    def test_metadata_prior_reuse_and_later_use_are_distinguished(self):
        self.make_series();metadata=self.root/"prior-history.json"
        # Inject only the metadata registry lookup, never a truth stream. The
        # alternate basename avoids poisoning the actual project's registry.
        self.module.used_seeds=lambda root:{(1,201):[str(metadata)]}
        for stamp,expected in (("20261008-070000",False),("20261008-090000",True)):
            with self.subTest(stamp=stamp):
                dump(metadata,{"schema":"mission3-run/1","run_id":stamp+"-entry1-seed201", "test_double":True,
                    "entry_id":1,"seed":201,"split":"training","completed":False})
                result=self.summarize();dump(self.case/f"history-{stamp}.json",result)
                self.assertEqual(result["conditional_research_gate_passed"],expected)
                self.assertEqual(len(result["known_prior_seed_reuse"]),int(not expected))
                self.assertEqual(len(result["later_seed_uses"]),int(expected))

    def test_empty_plan_refused(self):
        self.save_plan()
        with self.assertRaisesRegex(ValueError,"no scheduled"):self.summarize()


def main():
    global FROZEN_SOURCE,INPUT_HASHES
    OUT.mkdir(parents=True,exist_ok=False)
    sources=["scripts/summarize_mission3_series.py","scripts/evaluate_mission3.py",
             "scripts/mission3/evaluation.py","scripts/mission3/settings.py","scripts/mission3/controller.py",
             "scripts/test_mission3_series_summary.py"]
    for name in sources:
        path=ROOT/name;data=path.read_bytes();INPUT_HASHES[name]=hashlib.sha256(data).hexdigest()
        target=OUT/"source-snapshot"/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    FROZEN_SOURCE=(OUT/"source-snapshot/scripts/summarize_mission3_series.py").read_bytes()
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(SummaryTests))
    text=stream.getvalue();(OUT/"test-output.txt").write_text(text);print(text,end="")
    changed=[name for name,h in INPUT_HASHES.items() if sha(ROOT/name)!=h]
    report={"schema":"mission3-series-summary-test-review/1","tests_run":result.testsRun,
        "passed":result.testsRun-len(result.failures)-len(result.errors),
        "failures":[{"test":str(t),"traceback":s} for t,s in result.failures],
        "errors":[{"test":str(t),"traceback":s} for t,s in result.errors],
        "all_safety_expectations_passed":result.wasSuccessful(),"read_only_source_sha256":INPUT_HASHES,
        "shared_source_changed_during_tests":changed,"source_frozen_for_tests":True,
        "scope":"offline synthetic fixtures only; failures preserve independent review findings; no actual mission or approval",
        "python_executable":sys.executable,"python_version":sys.version,"artifacts":str(OUT)}
    dump(OUT/"report.json",report);print(OUT)
    return 0 if result.wasSuccessful() and not changed else 1


if __name__=="__main__":raise SystemExit(main())
