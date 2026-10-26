#!/usr/bin/env python3
"""관리 coordinator의 synthetic subprocess 검사. 실제 미션은 실행하지 않는다."""
import ast
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from zoneinfo import ZoneInfo

import run_mission3_series as series

ROOT=Path(__file__).resolve().parents[1]
OUT=Path(os.environ.get("MISSION3_SERIES_TEST_OUT",str(ROOT/"artifacts/mission3/series-runner"/
    datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f-tests"))))


FAKE_WORKER=r'''#!/usr/bin/env python3
import argparse,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
from datetime import datetime,timezone
def frozen_experiment_settings(cfg,physical,strict_focus):
    return {'mission':cfg,'physical':physical,'strict_focus':strict_focus}
p=argparse.ArgumentParser()
for name in ('entry','seed'): p.add_argument('--'+name,type=int,required=True)
for name in ('profile','out','split','series'): p.add_argument('--'+name,required=True)
p.add_argument('--timeout',type=float)
for name in ('respect-session-deadline','video','allow-near-focus','perturb-entry'):
    p.add_argument('--'+name,action='store_true')
a=p.parse_args();root=Path(__file__).resolve().parents[1];out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
control=json.loads((root/'fake-controls.json').read_text()).get(str(a.seed),{})
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,data): Path(path).write_text(json.dumps(data,indent=2)+'\n')
cfg=json.loads((root/'config/mission3.json').read_text());physical=json.loads(Path(a.profile).read_text())
cfg['entry']['inject_initial_uncertainty']=a.perturb_entry
if a.timeout is not None: cfg['control']['mission_timeout_s']=a.timeout
frozen={'mission':cfg,'physical':physical,'strict_focus':not a.allow_near_focus}
trial={**frozen,'entry_id':a.entry,'seed':a.seed}
save(out/'frozen-experiment-settings.json',frozen);save(out/'experiment.json',trial)
paths=[root/'scripts/run_mission3_physical.py',*sorted((root/'scripts/mission3').glob('*.py')),
       *sorted((root/'scripts/mission3').glob('*_host.c')),root/'scripts/yaw_core.py',root/'scripts/aruco_pose.py']
rpyt=physical.get('control_architecture',{}).get('mode')=='pc_velocity_rpyt'
if rpyt: paths.append(root/'scripts/mission3_rpyt_protocol.py')
folders=[('mission3-flight-controller','libmission3_controller.so'),('mission3-firmware-host','libmission3_estimator.so')]
if rpyt: folders.append(('mission3-position-bridge','libmission3_position_bridge.so'))
libraries={}
for folder,filename in folders:
    for name in (filename,'source-manifest.json','autoconf.h','FreeRTOS.h'):
        file=root/'build'/folder/name
        if file.exists(): libraries[str(file.relative_to(root))]=sha(file)
manifest={'schema':'mission3-run/1','test_double':True,'entry_id':a.entry,'seed':a.seed,'split':a.split,
          'series_id':a.series,'completed':False,'code_sha256':{str(f.relative_to(root)):sha(f) for f in paths},
          'runtime_library_sha256':libraries,'config_sha256':sha(out/'frozen-experiment-settings.json'),
          'config_identity_file':'frozen-experiment-settings.json','trial_metadata_sha256':sha(out/'experiment.json'),
          'termination_reason':'fake_started','hardware_matched':False}
manifest.update(control.get('manifest_overrides',{}));save(out/'run_manifest.json',manifest)
save(out/'fake-start.json',{'pid':os.getpid(),'pgid':os.getpgrp(),'t_utc':datetime.now(timezone.utc).isoformat(),'argv':sys.argv})
if control.get('ignore_signals'):
    signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
if control.get('spawn_child'):
    child=subprocess.Popen([sys.executable,'-c','import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)'])
    save(out/'fake-child.json',{'pid':child.pid,'pgid':os.getpgid(child.pid)})
try: time.sleep(control.get('sleep_s',.15))
except KeyboardInterrupt:
    save(out/'fake-interrupted.json',{'pid':os.getpid()});sys.exit(130)
manifest['completed']=True;manifest['termination_reason']='fake_failure' if control.get('exit_code',0) else 'fake_complete'
save(out/'run_manifest.json',manifest)
print('synthetic worker only',a.seed,flush=True)
sys.exit(control.get('exit_code',0))
'''


def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+"\n")


class SeriesTests(unittest.TestCase):
    def setUp(self):
        self.case=OUT/self._testMethodName;self.case.mkdir(parents=True,exist_ok=False)
        self.root=self.case/"fake-project";self.root.mkdir()
        dump(self.root/"fixture-provenance.json",{
            "schema":series.FIXTURE_SCHEMA,"test_double":True,"scope":series.FIXTURE_SCOPE,
            "producer":"scripts/test_mission3_series.py","producer_sha256":series.sha(Path(__file__)),
            "test_case":self._testMethodName,"fixture_root":str(self.root.resolve())})
        (self.root/"scripts/mission3").mkdir(parents=True)
        (self.root/"scripts/run_mission3_physical.py").write_text(FAKE_WORKER)
        for name in ("controller.py","test_contracts.py","settings.py","flight_controller_host.c","firmware_host.c"):
            (self.root/"scripts/mission3"/name).write_text("# synthetic source\n" if name.endswith('.py') else "/* synthetic C */\n")
        for name in ("yaw_core.py","aruco_pose.py","mission3_rpyt_protocol.py"):
            (self.root/"scripts"/name).write_text("# synthetic source\n")
        folders=[("mission3-flight-controller","libmission3_controller.so"),
                 ("mission3-firmware-host","libmission3_estimator.so"),
                 ("mission3-position-bridge","libmission3_position_bridge.so")]
        for folder,filename in folders:
            directory=self.root/"build"/folder;directory.mkdir(parents=True)
            (directory/filename).write_bytes(b"not a real loaded library; subprocess fixture only")
            dump(directory/"source-manifest.json",{"test_double":True})
            (directory/"autoconf.h").write_text("/* synthetic */\n")
            (directory/"FreeRTOS.h").write_text("/* synthetic */\n")
        self.cfg={"camera":{"width":320,"height":320,"encoding":"mono8"},
                  "entry":{"positions":[{"id":1,"xy_m":[2,3],"heading_deg":90}],"height_m":1,
                           "inject_initial_uncertainty":False},
                  "control":{"mission_timeout_s":300.},
                  "deadline_kst":(series.utc_now()+timedelta(minutes=5)).isoformat()}
        self.profile=self.root/"config/profile.json"
        dump(self.root/"config/mission3.json",self.cfg)
        dump(self.root/"config/provided-hardware.json",{"test_double":True})
        dump(self.profile,{"scope":"synthetic subprocess fixture; no physical flight",
                           "control_architecture":{"mode":"pc_velocity_rpyt"}})
        self.controls={};dump(self.root/"fake-controls.json",self.controls)
        python=self.root/"src/crazyflie-ros2-sil/.venv/bin/python";python.parent.mkdir(parents=True)
        python.symlink_to(sys.executable)

    def make_plan(self,**kwargs):
        output_name=kwargs.pop("output_name","series")
        options={"root":self.root,"profile":self.profile,"entry_id":1,"seeds":[201,202,203],
                 "series_id":"synthetic-series","worker_timeout_s":3.,"signal_grace_s":.07,
                 "terminate_grace_s":.07,"poll_interval_s":.015,"launch_margin_s":.02}
        options.update(kwargs)
        return series.create_plan(self.case/output_name,**options)

    def control(self,seed,**values):
        self.controls[str(seed)]=values;dump(self.root/"fake-controls.json",self.controls)

    def assert_denominator(self,state,expected):
        self.assertEqual(state["counts"]["scheduled"],expected)
        self.assertEqual(sum(state["counts"]["status_counts"].values()),expected)
        self.assertTrue(state["complete"])
        self.assertEqual(state["counts"]["qualified_mission_success"],0)

    def mutate_after_start(self,plan,mutation):
        first=json.loads(plan.read_text())["jobs"][0]
        def worker():
            limit=time.monotonic()+3
            while not (Path(first["output"])/"fake-start.json").exists():
                if time.monotonic()>limit: return
                time.sleep(.005)
            mutation()
        thread=threading.Thread(target=worker);thread.start()
        return thread

    def test_plan_dry_run_no_workers_and_native_schema(self):
        path=self.make_plan(mission_timeout_s=250,perturb_entry=True,allow_near_focus=True)
        plan=series.read_plan(path);report=series.dry_run(path)
        self.assertTrue(report["valid"]);self.assertEqual(report["workers_started"],0)
        self.assertFalse((self.case/"series/workers").exists())
        self.assertTrue(plan["jobs"][0]["argv"][0].endswith(".venv/bin/python"))
        expected=series.frozen_settings({**self.cfg,"entry":{**self.cfg["entry"],"inject_initial_uncertainty":True},
            "control":{"mission_timeout_s":250.}},json.loads(self.profile.read_text()),False)
        self.assertEqual(plan["config_sha256"],hashlib.sha256(series.json_bytes(expected)).hexdigest())
        self.assertNotEqual(plan["jobs"][0]["trial_metadata_sha256"],plan["jobs"][1]["trial_metadata_sha256"])
        self.assertIn("scripts/mission3/controller.py",plan["runtime_identity"]["code_sha256"])
        self.assertIn("scripts/mission3/test_contracts.py",plan["runtime_identity"]["code_sha256"])
        self.assertIn("scripts/mission3/flight_controller_host.c",plan["runtime_identity"]["code_sha256"])
        self.assertIn("build/mission3-position-bridge/libmission3_position_bridge.so",plan["runtime_identity"]["runtime_library_sha256"])

    def test_current_runner_frozen_function_read_only_contract(self):
        source=ROOT/"scripts/run_mission3_physical.py"
        function=next(node for node in ast.parse(source.read_text()).body if isinstance(node,ast.FunctionDef) and node.name=="frozen_experiment_settings")
        namespace={};exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),"exec"),namespace)
        cfg=json.loads((ROOT/"config/mission3.json").read_text())
        physical=json.loads((ROOT/"config/mission3-pc-rpyt-sensitivity.json").read_text())
        expected=namespace["frozen_experiment_settings"](cfg,physical,True)
        self.assertEqual(series.json_bytes(expected),series.json_bytes(series.frozen_settings(cfg,physical,True)))
        dump(self.case/"read-only-schema-evidence.json",{"runner_sha256":series.sha(source),
            "physical_config_sha256":series.sha(ROOT/"config/mission3-pc-rpyt-sensitivity.json"),
            "schema_bytes_match":True,"physical_worker_started":False})

    def test_nonempty_output_refused_without_overwrite(self):
        directory=self.case/"series";directory.mkdir();file=directory/"keep.txt";file.write_text("keep failure")
        with self.assertRaisesRegex(ValueError,"nonempty"): self.make_plan()
        self.assertEqual(file.read_text(),"keep failure")

    def test_unique_seed_and_max_workers_constraints(self):
        for options in ({"seeds":[201,201]},{"seeds":[]},{"seeds":[-1]},{"max_workers":4},{"max_workers":0}):
            with self.subTest(options=options),self.assertRaises(ValueError): self.make_plan(**options)

    def test_used_holdout_seed_rejected_including_active_attempt(self):
        dump(self.root/"artifacts/mission3/physical/old/run_manifest.json",{
            "schema":"mission3-run/1","entry_id":1,"seed":201,"completed":False})
        with self.assertRaisesRegex(ValueError,"already used"): self.make_plan()
        with self.assertRaisesRegex(ValueError,"holdout"): self.make_plan(allow_training_reuse=True)

    def test_explicit_training_reuse_preserves_history(self):
        dump(self.root/"artifacts/mission3/physical/old/run_manifest.json",{
            "schema":"mission3-run/1","entry_id":1,"seed":201,"completed":True})
        path=self.make_plan(split="training",allow_training_reuse=True)
        plan=series.read_plan(path);self.assertFalse(plan["seeds_unused_at_entry"])
        self.assertIn("201",plan["training_reuse_history"])

    def test_corrupt_seed_registry_refuses_unused_claim(self):
        path=self.root/"artifacts/mission3/physical/broken/run_manifest.json";path.parent.mkdir(parents=True)
        path.write_text("{broken")
        with self.assertRaisesRegex(ValueError,"registry unreadable"): self.make_plan()

    def nested_fixture(self,*,legacy=False):
        batch=self.root/"artifacts/mission3/series-runner/nested-tests"
        case=self._testMethodName;fixture=batch/case/"fake-project"
        dump(fixture/"config/provided-hardware.json",{"test_double":True})
        dump(fixture/"config/profile.json",{"scope":series.FIXTURE_SCOPE})
        path=fixture/"artifacts/mission3/physical/broken/run_manifest.json"
        path.parent.mkdir(parents=True);path.write_text("{intentional malformed fixture")
        if legacy:
            source=batch/"handoff-source/test_mission3_series.py"
            source.parent.mkdir();source.write_bytes(Path(__file__).read_bytes())
            dump(batch/"report.json",{"scope":series.TEST_REPORT_SCOPE,"artifacts":str(batch),
                "source_sha256":{"scripts/test_mission3_series.py":series.sha(source)}})
        else:
            dump(fixture/"fixture-provenance.json",{
                "schema":series.FIXTURE_SCHEMA,"test_double":True,"scope":series.FIXTURE_SCOPE,
                "producer":"scripts/test_mission3_series.py","producer_sha256":series.sha(Path(__file__)),
                "test_case":case,"fixture_root":str(fixture.resolve())})
        return fixture,path

    def test_nested_explicit_fixture_corrupt_manifest_does_not_block_plan(self):
        fixture,path=self.nested_fixture()
        dump(fixture/"artifacts/mission3/physical/unmarked/run_manifest.json",{
            "schema":"mission3-run/1","entry_id":1,"seed":201,"completed":False})
        plan=self.make_plan();self.assertTrue(series.dry_run(plan)["valid"])
        self.assertEqual(series.used_seeds(self.root),{})
        self.assertEqual(path.read_text(),"{intentional malformed fixture")

    def test_nested_legacy_fixture_requires_report_archived_hash_and_case(self):
        fixture,path=self.nested_fixture(legacy=True)
        self.assertEqual(series.used_seeds(self.root),{})
        source=fixture.parent.parent/"handoff-source/test_mission3_series.py"
        source.write_text("# hash mismatch\n")
        with self.assertRaisesRegex(ValueError,"registry unreadable"): series.used_seeds(self.root)

    def test_nested_fake_directory_without_provenance_still_refuses_corrupt(self):
        fixture,path=self.nested_fixture();(fixture/"fixture-provenance.json").unlink()
        with self.assertRaisesRegex(ValueError,"registry unreadable"): self.make_plan()

    def test_corrupt_actual_registry_beside_nested_fixture_still_refuses(self):
        self.nested_fixture()
        actual=self.root/"artifacts/mission3/physical/actual/run_manifest.json"
        actual.parent.mkdir(parents=True);actual.write_text("{corrupt actual attempt")
        with self.assertRaisesRegex(ValueError,"registry unreadable"): self.make_plan()
        self.assertEqual(actual.read_text(),"{corrupt actual attempt")

    def test_actual_series_workers_under_series_runner_are_counted(self):
        self.nested_fixture()
        actual=self.root/"artifacts/mission3/series-runner/real-series/workers/001-entry1-seed201/run_manifest.json"
        dump(actual,{"schema":"mission3-run/1","entry_id":1,"seed":201,"completed":False})
        self.assertEqual(series.used_seeds(self.root)[(1,201)],[str(actual)])
        with self.assertRaisesRegex(ValueError,"already used"): self.make_plan()
        actual.write_text("{corrupt real series worker")
        with self.assertRaisesRegex(ValueError,"registry unreadable"): self.make_plan()

    def test_max_three_real_fake_processes_and_all_outcomes_kept(self):
        self.control(202,exit_code=2)
        self.control(205,exit_code=7)
        path=self.make_plan(seeds=[201,202,203,204,205,206]);state=series.execute_plan(path)
        self.assert_denominator(state,6)
        self.assertEqual(state["peak_active"],3)
        self.assertEqual(state["counts"]["started"],6)
        self.assertEqual(state["counts"]["worker_succeeded"],4)
        self.assertEqual(state["counts"]["worker_failed"],2)
        self.assertEqual([j["seed"] for j in state["jobs"]],[201,202,203,204,205,206])
        self.assertTrue(all(j["pid"]==j["pgid"] for j in state["jobs"]))
        self.assertTrue(all(j["manifest_check"]["valid"] for j in state["jobs"]))

    def test_worker_output_conflict_aborts_before_any_start(self):
        path=self.make_plan();plan=series.read_plan(path)
        output=Path(plan["jobs"][1]["output"]);output.mkdir(parents=True);(output/"failure.log").write_text("retain")
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],0)
        self.assertEqual(state["stop_reason"],"output_conflict")
        self.assertEqual((output/"failure.log").read_text(),"retain")

    def test_source_changed_before_launch_aborts_all_scheduled(self):
        path=self.make_plan();(self.root/"scripts/mission3/controller.py").write_text("# changed\n")
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],0)
        self.assertEqual(state["counts"]["not_launched"],3)
        self.assertEqual(state["stop_reason"],"identity_change")

    def test_source_changed_during_worker_stops_future_and_marks_active(self):
        self.control(201,sleep_s=.35)
        path=self.make_plan(max_workers=1)
        thread=self.mutate_after_start(path,lambda:(self.root/"scripts/mission3/controller.py").write_text("# mutated\n"))
        state=series.execute_plan(path);thread.join()
        self.assert_denominator(state,3);self.assertEqual(state["counts"]["started"],1)
        self.assertEqual(state["jobs"][0]["status"],"identity_changed")
        self.assertEqual(state["counts"]["not_launched"],2)

    def test_invalid_runner_source_during_edit_blocks_future_launches(self):
        self.control(201,sleep_s=.35);path=self.make_plan(max_workers=1)
        thread=self.mutate_after_start(path,lambda:(self.root/"scripts/run_mission3_physical.py").write_text("def temporarily_unfinished(\n"))
        state=series.execute_plan(path);thread.join();self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1)
        self.assertEqual(state["stop_reason"],"identity_change")
        self.assertEqual(state["jobs"][0]["status"],"identity_changed")

    def test_config_mutation_during_worker_prevents_future(self):
        self.control(201,sleep_s=.35);path=self.make_plan(max_workers=1)
        thread=self.mutate_after_start(path,lambda:dump(self.root/"config/mission3.json",{**self.cfg,"changed":True}))
        state=series.execute_plan(path);thread.join();self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1);self.assertEqual(state["stop_reason"],"identity_change")

    def test_library_mutation_during_worker_prevents_future(self):
        self.control(201,sleep_s=.35);path=self.make_plan(max_workers=1)
        library=self.root/"build/mission3-flight-controller/libmission3_controller.so"
        thread=self.mutate_after_start(path,lambda:library.write_bytes(b"changed synthetic library"))
        state=series.execute_plan(path);thread.join();self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1);self.assertEqual(state["stop_reason"],"identity_change")

    def test_new_host_source_inventory_detected(self):
        path=self.make_plan();(self.root/"scripts/mission3/new_host.c").write_text("/* added */")
        self.assertFalse(series.dry_run(path)["valid"])
        state=series.execute_plan(path);self.assertEqual(state["counts"]["started"],0)

    def test_frozen_profile_mutation_during_worker_prevents_future(self):
        self.control(201,sleep_s=.35);path=self.make_plan(max_workers=1)
        frozen=self.case/"series/frozen-profile.json"
        thread=self.mutate_after_start(path,lambda:dump(frozen,{"changed":True}))
        state=series.execute_plan(path);thread.join();self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1);self.assertEqual(state["stop_reason"],"identity_change")

    def test_new_seed_conflict_after_plan_preserves_unstarted_denominator(self):
        path=self.make_plan(max_workers=1)
        dump(self.root/"artifacts/mission3/physical/manual/run_manifest.json",{
            "schema":"mission3-run/1","entry_id":1,"seed":202,"completed":False})
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1)
        self.assertEqual(state["stop_reason"],"seed_conflict")
        self.assertEqual(state["counts"]["not_launched"],2)

    def test_manifest_identity_mismatch_is_failed_even_exit_zero(self):
        self.control(201,manifest_overrides={"config_sha256":"wrong"})
        path=self.make_plan(seeds=[201]);state=series.execute_plan(path)
        self.assert_denominator(state,1);self.assertEqual(state["jobs"][0]["returncode"],0)
        self.assertEqual(state["jobs"][0]["status"],"worker_failed")
        self.assertIn("config_sha256",state["jobs"][0]["manifest_check"]["mismatched_fields"])

    def test_deadline_no_future_jobs_and_graceful_only_owned(self):
        self.control(201,sleep_s=5)
        path=self.make_plan(max_workers=1,cutoff=series.utc_now()+timedelta(seconds=.45))
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],1)
        self.assertEqual(state["jobs"][0]["status"],"cancelled_at_cutoff")
        self.assertEqual(state["counts"]["not_launched"],2)
        plan=series.read_plan(path)
        self.assertLess(series.aware(state["jobs"][0]["start_requested_at_utc"]),series.aware(plan["cutoff_utc"]))
        self.assertTrue((Path(state["jobs"][0]["output"])/"fake-interrupted.json").exists())
        signals=[e for e in state["events"] if e["event"]=="signal_owned_group"]
        self.assertEqual({e["pid"] for e in signals},{state["jobs"][0]["pid"]})

    def test_expired_deadline_starts_zero(self):
        path=self.make_plan(cutoff=series.utc_now()-timedelta(seconds=1))
        self.assertFalse(series.dry_run(path)["valid"])
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["counts"]["started"],0)
        self.assertEqual(state["stop_reason"],"cutoff")

    def test_worker_timeout_escalation_and_unrelated_process_survives(self):
        unrelated=subprocess.Popen([sys.executable,"-c","import time; time.sleep(10)"],start_new_session=True)
        self.control(201,sleep_s=5,ignore_signals=True,spawn_child=True)
        try:
            path=self.make_plan(seeds=[201],worker_timeout_s=.35)
            state=series.execute_plan(path);self.assert_denominator(state,1)
            self.assertEqual(state["jobs"][0]["status"],"timed_out")
            self.assertEqual(state["jobs"][0]["returncode"],-signal_number("SIGKILL"))
            self.assertIsNone(unrelated.poll())
            actions=[e for e in state["events"] if e["event"]=="signal_owned_group"]
            self.assertEqual([e["signal"] for e in actions],["SIGINT","SIGTERM","SIGKILL"])
            self.assertEqual({e["pid"] for e in actions},{state["jobs"][0]["pid"]})
            child=json.loads((Path(state["jobs"][0]["output"])/"fake-child.json").read_text())
            stat=Path(f"/proc/{child['pid']}/stat")
            limit=time.monotonic()+2
            while stat.exists() and stat.read_text().split()[2]!="Z" and time.monotonic()<limit: time.sleep(.01)
            self.assertTrue(not stat.exists() or stat.read_text().split()[2]=="Z")
            dump(self.case/"unrelated-process-evidence.json",{"unrelated_pid":unrelated.pid,
                "unrelated_survived":True,"owned_worker_pid":state["jobs"][0]["pid"],"owned_child":child})
        finally:
            unrelated.terminate();unrelated.wait(timeout=3)

    def test_launch_error_preserves_denominator(self):
        bad=self.case/"not-executable";bad.write_text("not executable")
        path=self.make_plan(python_executable=bad)
        state=series.execute_plan(path);self.assert_denominator(state,3)
        self.assertEqual(state["jobs"][0]["status"],"launch_failed")
        self.assertEqual(state["counts"]["not_launched"],2)

    def test_cli_argv_paths_with_spaces_and_shell_metacharacters_are_literal(self):
        old=self.profile;self.profile=old.with_name("profile ; touch INJECTION_SENTINEL.json")
        self.profile.write_bytes(old.read_bytes())
        path=self.make_plan(seeds=[201],output_name="series ; touch INJECTION_SENTINEL.json");state=series.execute_plan(path)
        self.assert_denominator(state,1);self.assertEqual(state["counts"]["worker_succeeded"],1)
        self.assertFalse((self.root/"INJECTION_SENTINEL.json").exists())
        args=json.loads((Path(state["jobs"][0]["output"])/"fake-start.json").read_text())["argv"]
        self.assertEqual(args[args.index("--profile")+1],str(path.parent/"frozen-profile.json"))

    def test_cli_plan_dry_run_and_fake_run_roundtrip(self):
        coordinator=ROOT/"scripts/run_mission3_series.py";output=self.case/"cli series"
        prefix=[sys.executable,str(coordinator)]
        actions=[prefix+["plan","--root",str(self.root),"--profile",str(self.profile),"--out",str(output),
            "--entry","1","--seeds","201","202","--series","cli-synthetic","--workers","2"],
            prefix+["dry-run","--plan",str(output/"plan.json")],
            prefix+["run","--plan",str(output/"plan.json")]]
        for i,argv in enumerate(actions):
            result=subprocess.run(argv,capture_output=True,text=True,timeout=10,shell=False)
            (self.case/f"cli-{i}.log").write_text(result.stdout+result.stderr)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        state=json.loads((output/"state.json").read_text());self.assert_denominator(state,2)
        self.assertEqual(state["counts"]["worker_succeeded"],2)

    def assert_cli_interrupt_drains(self,interrupt_signal):
        self.control(201,sleep_s=5,ignore_signals=True)
        path=self.make_plan(max_workers=1)
        log=(self.case/"coordinator-interrupt.log").open("w")
        proc=subprocess.Popen([sys.executable,str(ROOT/"scripts/run_mission3_series.py"),"run","--plan",str(path)],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True,shell=False)
        try:
            output=Path(series.read_plan(path)["jobs"][0]["output"])
            limit=time.monotonic()+3
            while not (output/"fake-start.json").exists() and time.monotonic()<limit: time.sleep(.01)
            self.assertTrue((output/"fake-start.json").exists())
            time.sleep(.02)
            import signal
            proc.send_signal(interrupt_signal)
            self.assertEqual(proc.wait(timeout=5),2)
            state=json.loads((path.parent/"state.json").read_text());self.assert_denominator(state,3)
            self.assertEqual(state["jobs"][0]["status"],"interrupted")
            self.assertEqual(state["counts"]["not_launched"],2)
        finally:
            if proc.poll() is None:
                proc.terminate();proc.wait(timeout=3)
            log.close()

    def test_coordinator_sigint_drains_only_its_workers(self):
        import signal
        self.assert_cli_interrupt_drains(signal.SIGINT)

    def test_coordinator_sigterm_drains_only_its_workers(self):
        import signal
        self.assert_cli_interrupt_drains(signal.SIGTERM)

    def test_two_coordinators_cannot_claim_same_plan(self):
        self.control(201,sleep_s=.35);path=self.make_plan(seeds=[201])
        argv=[sys.executable,str(ROOT/"scripts/run_mission3_series.py"),"run","--plan",str(path)]
        log=(self.case/"first-coordinator.log").open("w")
        proc=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,shell=False)
        try:
            marker=path.parent/".series-run-owner";limit=time.monotonic()+3
            while not marker.exists() and time.monotonic()<limit: time.sleep(.005)
            self.assertTrue(marker.exists())
            result=subprocess.run(argv,capture_output=True,text=True,timeout=5,shell=False)
            (self.case/"second-coordinator.log").write_text(result.stdout+result.stderr)
            self.assertEqual(result.returncode,2)
            self.assertIn("already started",result.stdout)
            self.assertEqual(proc.wait(timeout=5),0)
            state=json.loads((path.parent/"state.json").read_text());self.assert_denominator(state,1)
            self.assertEqual(state["counts"]["started"],1)
            self.assertEqual(state["counts"]["worker_succeeded"],1)
        finally:
            if proc.poll() is None:
                proc.terminate();proc.wait(timeout=3)
            log.close()

    def test_changed_runner_settings_schema_refused_before_launch(self):
        file=self.root/"scripts/run_mission3_physical.py"
        file.write_text(file.read_text().replace("'strict_focus':strict_focus","'focus':strict_focus",1))
        with self.assertRaisesRegex(ValueError,"schema changed"): self.make_plan()

    def test_modified_plan_digest_refused(self):
        path=self.make_plan();data=json.loads(path.read_text());data["max_workers"]=100
        dump(path,data)
        with self.assertRaisesRegex(ValueError,"digest mismatch"): series.execute_plan(path)

    def test_existing_state_never_restarts_completed_workers(self):
        path=self.make_plan(seeds=[201]);state=series.execute_plan(path)
        with self.assertRaisesRegex(ValueError,"already started"): series.execute_plan(path)
        self.assertEqual(state["counts"]["started"],1)


def signal_number(name):
    import signal
    return int(getattr(signal,name))


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    stream=io.StringIO()
    runner=unittest.TextTestRunner(stream=stream,verbosity=2)
    result=runner.run(unittest.defaultTestLoader.loadTestsFromTestCase(SeriesTests))
    log=stream.getvalue();(OUT/"test-output.txt").write_text(log);print(log,end="")
    report={"tests_run":result.testsRun,"failures":[{"test":str(t),"traceback":s} for t,s in result.failures],
        "errors":[{"test":str(t),"traceback":s} for t,s in result.errors],"successful":result.wasSuccessful(),
        "scope":"fake subprocess management only; no full mission or hardware commands",
        "source_sha256":{str(p.relative_to(ROOT)):series.sha(p) for p in (Path(__file__),ROOT/"scripts/run_mission3_series.py")},
        "python_executable":sys.executable,"python_version":sys.version,"artifacts":str(OUT)}
    dump(OUT/"report.json",report);print(OUT)
    return 0 if result.wasSuccessful() else 1


if __name__=="__main__": raise SystemExit(main())
