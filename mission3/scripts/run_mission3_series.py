#!/usr/bin/env python3
"""동결 Mission3 series의 계획·실행 관리. 미션/접촉 검증·승격은 하지 않는다.

stdlib만 사용한다. argv list/shell=False로 자체 시작한 worker의 PID/process
group만 관리한다. source/settings/library 변경 뒤 새 worker를 시작하지 않는다.
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
SCHEMA="mission3-series/1"
FIXTURE_SCHEMA="mission3-series-test-fixture/1"
FIXTURE_SCOPE="synthetic subprocess fixture; no physical flight"
TEST_REPORT_SCOPE="fake subprocess management only; no full mission or hardware commands"


def utc_now(): return datetime.now(timezone.utc)
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def json_bytes(value): return (json.dumps(value,indent=2)+"\n").encode()


def save(path,value):
    path=Path(path);temporary=path.with_name(path.name+".tmp")
    temporary.write_bytes(json_bytes(value));temporary.replace(path)


def aware(value):
    result=datetime.fromisoformat(value) if isinstance(value,str) else value
    if result.tzinfo is None: raise ValueError("deadline must include a timezone")
    return result.astimezone(timezone.utc)


def frozen_settings(cfg,physical,strict_focus):
    # run_mission3_physical.frozen_experiment_settings와 같은 key/order/byte schema.
    return {"mission":cfg,"physical":physical,"strict_focus":strict_focus}


def verify_settings_contract(runner):
    try: tree=ast.parse(Path(runner).read_text())
    except SyntaxError as exc: raise ValueError("runner source temporarily invalid; freeze identity unavailable") from exc
    function=next((n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="frozen_experiment_settings"),None)
    if function is None: raise ValueError("runner frozen-settings function missing")
    body=[n for n in function.body if not (isinstance(n,ast.Expr) and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str))]
    expected=ast.parse("{'mission': cfg, 'physical': physical, 'strict_focus': strict_focus}",mode="eval").body
    if len(body)!=1 or not isinstance(body[0],ast.Return) or ast.dump(body[0].value)!=ast.dump(expected):
        raise ValueError("runner frozen-settings schema changed; verify coordinator compatibility first")


def runtime_identity(root,profile):
    root=Path(root);profile=Path(profile)
    verify_settings_contract(root/"scripts/run_mission3_physical.py")
    physical=json.loads(profile.read_text())
    rpyt=physical.get("control_architecture",{}).get("mode")=="pc_velocity_rpyt"
    paths=[root/"scripts/run_mission3_physical.py",*sorted((root/"scripts/mission3").glob("*.py")),
           *sorted((root/"scripts/mission3").glob("*_host.c")),root/"scripts/yaw_core.py",root/"scripts/aruco_pose.py"]
    if rpyt: paths.append(root/"scripts/mission3_rpyt_protocol.py")
    code={str(p.relative_to(root)):sha(p) for p in paths}
    folders=[("mission3-flight-controller","libmission3_controller.so"),
             ("mission3-firmware-host","libmission3_estimator.so")]
    if rpyt: folders.append(("mission3-position-bridge","libmission3_position_bridge.so"))
    libraries={}
    for folder,filename in folders:
        if not (root/"build"/folder/filename).is_file(): raise ValueError(f"missing required library: {folder}/{filename}")
        for name in (filename,"source-manifest.json","autoconf.h","FreeRTOS.h"):
            p=root/"build"/folder/name
            if p.is_file(): libraries[str(p.relative_to(root))]=sha(p)
    extra={str(p.resolve()):sha(p) for p in (root/"config/mission3.json",profile,root/"config/provided-hardware.json")}
    return {"code_sha256":code,"runtime_library_sha256":libraries,"input_file_sha256":extra}


def is_nested_test_fixture(path,base,cache):
    """정확한 fake-project subtree만 제외한다. series의 실제 workers는 포함한다.

    새 fixture는 명시한 provenance marker, marker 이전 archive는 report와
    hash가 맞는 보존 test source/case 및 fake config metadata로 구별한다.
    scan root 자체의 fixture는 제외하지 않아 corrupt-registry 검사도 유효하다.
    """
    parts=path.relative_to(base).parts
    if len(parts)<8 or parts[0]!="series-runner" or parts[3]!="fake-project": return False
    fixture=base.joinpath(*parts[:4])
    if fixture in cache: return cache[fixture]
    cache[fixture]=False
    batch=fixture.parent.parent;case=fixture.parent.name
    try:
        marker=fixture/"fixture-provenance.json"
        if marker.exists():
            provenance=json.loads(marker.read_text())
            valid=(isinstance(provenance,dict) and provenance.get("schema")==FIXTURE_SCHEMA
                and provenance.get("test_double") is True and provenance.get("scope")==FIXTURE_SCOPE
                and provenance.get("producer")=="scripts/test_mission3_series.py"
                and re.fullmatch(r"[0-9a-f]{64}",provenance.get("producer_sha256","")) is not None
                and provenance.get("test_case")==case
                and Path(provenance.get("fixture_root","")).resolve()==fixture.resolve())
        else:
            # 기존 24/27/30-test archive는 수정하지 않는다. report가 기록한
            # test hash와 실제 보존 source에서 확인되는 case만 허용한다.
            report=json.loads((batch/"report.json").read_text())
            expected=report["source_sha256"]["scripts/test_mission3_series.py"]
            valid=(report.get("scope")==TEST_REPORT_SCOPE and Path(report["artifacts"]).resolve()==batch.resolve())
            archived=next((p for p in batch.rglob("test_mission3_series.py") if sha(p)==expected),None) if valid else None
            cases=set()
            if archived:
                tree=ast.parse(archived.read_text())
                cases={n.name for c in tree.body if isinstance(c,ast.ClassDef) and c.name=="SeriesTests"
                       for n in c.body if isinstance(n,ast.FunctionDef) and n.name.startswith("test_")}
            valid=bool(valid and case in cases)
        hardware=json.loads((fixture/"config/provided-hardware.json").read_text())
        profile=json.loads((fixture/"config/profile.json").read_text())
        cache[fixture]=bool(valid and hardware.get("test_double") is True and profile.get("scope")==FIXTURE_SCOPE)
    except (OSError,ValueError,KeyError,TypeError,AttributeError,SyntaxError): pass
    return cache[fixture]


def used_seeds(root,exclude=None):
    """metadata-only 조회. 실제 attempt는 completed=false도 사용 seed로 센다."""
    root=Path(root);base=root/"artifacts/mission3";seen={};fixture_cache={}
    if not base.exists(): return seen
    for path in sorted(base.rglob("run_manifest.json")):
        if exclude and path.resolve().is_relative_to(Path(exclude).resolve()): continue
        if is_nested_test_fixture(path,base,fixture_cache): continue
        try: manifest=json.loads(path.read_text())
        except (OSError,ValueError) as exc: raise ValueError(f"seed registry unreadable: {path}") from exc
        if not isinstance(manifest,dict): raise ValueError(f"seed registry metadata is not an object: {path}")
        if manifest.get("schema")!="mission3-run/1" or manifest.get("test_double",False): continue
        if not isinstance(manifest.get("entry_id"),int) or not isinstance(manifest.get("seed"),int):
            raise ValueError(f"seed registry metadata incomplete: {path}")
        seen.setdefault((manifest["entry_id"],manifest["seed"]),[]).append(str(path))
    return seen


def check_identity(plan):
    try: current=runtime_identity(Path(plan["root"]),Path(plan["profile_origin"]))
    except (OSError,ValueError,KeyError) as exc: return {"identity_read_error":str(exc)}
    differences={}
    for category,expected in plan["runtime_identity"].items():
        actual=current[category]
        changed=[p for p in sorted(set(expected)|set(actual)) if expected.get(p)!=actual.get(p)]
        if changed: differences[category]=changed
    if sha(Path(__file__))!=plan["coordinator_sha256"]:
        differences["coordinator"]=[str(Path(__file__).resolve())]
    for key,path in (("frozen_profile_sha256",Path(plan["profile_frozen"])),
                     ("config_sha256",Path(plan["output"])/plan["config_identity_file"])):
        try: actual=sha(path)
        except OSError: actual=None
        if actual!=plan[key]: differences[key]=[str(path)]
    return differences


def create_plan(out,*,root=ROOT,profile=None,entry_id=1,seeds,split="holdout",series_id,
                max_workers=3,cutoff=None,worker_timeout_s=900.,signal_grace_s=5.,
                terminate_grace_s=1.,poll_interval_s=.1,launch_margin_s=.05,
                mission_timeout_s=None,video=False,allow_near_focus=False,perturb_entry=False,
                allow_training_reuse=False,python_executable=None):
    root=Path(root).resolve();out=Path(out).resolve()
    profile=Path(profile or root/"config/mission3-physical-sensitivity.json").resolve()
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError("refuse existing nonempty output")
    if not 1<=max_workers<=3: raise ValueError("workers must be 1..3")
    if split not in ("training","holdout"): raise ValueError("invalid split")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}",series_id): raise ValueError("explicit safe series id required")
    if allow_training_reuse and split!="training": raise ValueError("holdout may not reuse seeds")
    seeds=list(seeds)
    if not seeds or any(type(s) is not int or s<0 or s>=2**32 for s in seeds) or len(set(seeds))!=len(seeds):
        raise ValueError("unique nonnegative uint32 seeds required")
    for value in (worker_timeout_s,signal_grace_s,terminate_grace_s,poll_interval_s,launch_margin_s):
        if not math.isfinite(value) or value<=0: raise ValueError("positive finite timing limits required")
    cfg_bytes=(root/"config/mission3.json").read_bytes();profile_bytes=profile.read_bytes()
    cfg=json.loads(cfg_bytes);physical=json.loads(profile_bytes)
    entry=next((e for e in cfg["entry"]["positions"] if e["id"]==entry_id),None)
    if entry is None: raise ValueError("entry must exist in known static entry map")
    if (cfg["camera"]["width"],cfg["camera"]["height"],cfg["camera"]["encoding"])!=(320,320,"mono8"):
        raise ValueError("native320 mono8 configuration required")
    cfg["entry"]["inject_initial_uncertainty"]=bool(perturb_entry)
    if mission_timeout_s is not None:
        if not math.isfinite(mission_timeout_s) or mission_timeout_s<=0: raise ValueError("invalid mission timeout")
        mission_timeout_s=float(mission_timeout_s)
        cfg["control"]["mission_timeout_s"]=mission_timeout_s
    session_cutoff=aware(cfg["deadline_kst"])
    cutoff=session_cutoff if cutoff is None else aware(cutoff)
    if cutoff>session_cutoff: raise ValueError("cutoff may not exceed project deadline")
    history=used_seeds(root)
    reused={str(seed):history[(entry_id,seed)] for seed in seeds if (entry_id,seed) in history}
    if reused and not allow_training_reuse: raise ValueError(f"seeds already used at entry {entry_id}: {list(reused)}")
    identity=runtime_identity(root,profile)
    for path,data in ((root/"config/mission3.json",cfg_bytes),(profile,profile_bytes)):
        if identity["input_file_sha256"][str(path.resolve())]!=hashlib.sha256(data).hexdigest():
            raise ValueError("config changed while reading plan inputs")
    settings=frozen_settings(cfg,physical,not allow_near_focus)
    # venv symlink를 realpath로 바꾸면 system Python이 선택된다. 경로를 유지한다.
    python_executable=Path(python_executable or root/"src/crazyflie-ros2-sil/.venv/bin/python").absolute()
    if not python_executable.is_file(): raise ValueError("team python executable missing")
    out.mkdir(parents=True,exist_ok=True)
    with (out/".series-plan-owner").open("x") as file: file.write(f"planner_pid={os.getpid()}\n")
    # exclusive plan write 아래에서 수행; 기존 파일은 수정하지 않는다.
    frozen_profile=out/"frozen-profile.json"
    frozen_profile.write_bytes(json_bytes(physical))
    (out/"frozen-experiment-settings.json").write_bytes(json_bytes(settings))
    for category in ("code_sha256","runtime_library_sha256"):
        for relative in identity[category]:
            target=out/"source-snapshot"/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(root/relative,target)
            if sha(target)!=identity[category][relative]: raise ValueError(f"source changed during snapshot: {relative}")
    for name,path in (("mission-source.json",root/"config/mission3.json"),("profile-source.json",profile),
                      ("provided-hardware-source.json",root/"config/provided-hardware.json")):
        shutil.copyfile(path,out/name)
        if sha(out/name)!=identity["input_file_sha256"][str(path.resolve())]:
            raise ValueError(f"input changed during snapshot: {path}")
    management=out/"management-source";management.mkdir()
    shutil.copyfile(Path(__file__),management/Path(__file__).name)
    test=Path(__file__).with_name("test_mission3_series.py")
    if test.exists(): shutil.copyfile(test,management/test.name)
    jobs=[]
    for i,seed in enumerate(seeds):
        job_id=f"{i+1:03d}-entry{entry_id}-seed{seed}"
        output=out/"workers"/job_id
        argv=[str(python_executable),str(root/"scripts/run_mission3_physical.py"),"--entry",str(entry_id),
              "--seed",str(seed),"--profile",str(frozen_profile),"--out",str(output),
              "--respect-session-deadline","--split",split,"--series",series_id]
        if mission_timeout_s is not None: argv += ["--timeout",str(mission_timeout_s)]
        if video: argv.append("--video")
        if allow_near_focus: argv.append("--allow-near-focus")
        if perturb_entry: argv.append("--perturb-entry")
        trial={"mission":cfg,"physical":physical,"strict_focus":not allow_near_focus,"entry_id":entry_id,"seed":seed}
        jobs.append({"job_id":job_id,"entry_id":entry_id,"seed":seed,"output":str(output),"argv":argv,
                     "trial_metadata_sha256":hashlib.sha256(json_bytes(trial)).hexdigest()})
    plan={"schema":SCHEMA,"created_at_utc":utc_now().isoformat(),"root":str(root),"output":str(out),
          "series_id":series_id,"split":split,"entry_id":entry_id,"known_static_entry":deepcopy(entry),
          "seeds":seeds,"seeds_unused_at_entry":not bool(reused),"training_reuse_history":reused,
          "allow_training_reuse":allow_training_reuse,"profile_origin":str(profile),
          "profile_frozen":str(frozen_profile),"runtime_identity":identity,
          "coordinator_sha256":sha(Path(__file__)),"frozen_profile_sha256":sha(frozen_profile),
          "config_identity_file":"frozen-experiment-settings.json",
          "config_sha256":hashlib.sha256(json_bytes(settings)).hexdigest(),
          "cutoff_utc":cutoff.isoformat(),"project_deadline_utc":session_cutoff.isoformat(),
          "max_workers":max_workers,"worker_timeout_s":worker_timeout_s,"signal_grace_s":signal_grace_s,
          "terminate_grace_s":terminate_grace_s,"poll_interval_s":poll_interval_s,
          "launch_margin_s":launch_margin_s,"jobs":jobs,
          "claims":{"hardware_matched":False,"qualified_promotion":False,"independent_review":"manual",
                    "truth_files_read":False,"worker_success_is_process_manifest_status_only":True}}
    with (out/"plan.json").open("xb") as file: file.write(json_bytes(plan))
    (out/"plan.sha256").write_text(sha(out/"plan.json")+"\n")
    differences=check_identity(plan)
    if differences:
        save(out/"plan-creation-error.json",differences)
        raise ValueError(f"source changed while freezing plan: {differences}")
    return out/"plan.json"


def read_plan(path):
    path=Path(path).resolve()
    if sha(path)!=path.with_name("plan.sha256").read_text().strip(): raise ValueError("plan digest mismatch")
    plan=json.loads(path.read_text())
    if plan.get("schema")!=SCHEMA or not 1<=plan["max_workers"]<=3: raise ValueError("unsupported plan")
    if Path(plan["output"]).resolve()!=path.parent: raise ValueError("plan output location mismatch")
    if sha(plan["profile_frozen"])!=plan["frozen_profile_sha256"]: raise ValueError("frozen profile changed")
    if sha(path.parent/plan["config_identity_file"])!=plan["config_sha256"]: raise ValueError("frozen settings changed")
    if aware(plan["cutoff_utc"])>aware(plan["project_deadline_utc"]): raise ValueError("invalid cutoff")
    return plan


def manifest_check(plan,job):
    out=Path(job["output"]);path=out/"run_manifest.json"
    try:
        manifest=json.loads(path.read_text())
        expected={"schema":"mission3-run/1","entry_id":job["entry_id"],"seed":job["seed"],
                  "split":plan["split"],"series_id":plan["series_id"],"completed":True,
                  "code_sha256":plan["runtime_identity"]["code_sha256"],
                  "runtime_library_sha256":plan["runtime_identity"]["runtime_library_sha256"],
                  "config_sha256":plan["config_sha256"],"config_identity_file":plan["config_identity_file"],
                  "trial_metadata_sha256":job["trial_metadata_sha256"]}
        mismatched=[key for key,value in expected.items() if manifest.get(key)!=value]
        if sha(out/plan["config_identity_file"])!=plan["config_sha256"]: mismatched.append("frozen_settings_bytes")
        if sha(out/"experiment.json")!=job["trial_metadata_sha256"]: mismatched.append("trial_metadata_bytes")
        return {"valid":not mismatched,"mismatched_fields":mismatched,
                "termination_reason":manifest.get("termination_reason"),"manifest_sha256":sha(path)}
    except (OSError,ValueError,KeyError) as exc:
        return {"valid":False,"error":str(exc)}


def summarize(state):
    counts=dict(Counter(j["status"] for j in state["jobs"]))
    state["counts"]={"scheduled":len(state["jobs"]),"started":sum(j.get("pid") is not None for j in state["jobs"]),
        "worker_succeeded":counts.get("worker_succeeded",0),"worker_failed":counts.get("worker_failed",0),
        "not_launched":sum(n for name,n in counts.items() if name.startswith("not_launched")),"status_counts":counts,
        "qualified_mission_success":0,"independent_reviews_performed":0}
    state["complete"]=all(j["status"] not in ("planned","running") for j in state["jobs"])
    return state


def dry_run(path):
    plan=read_plan(path);differences=check_identity(plan)
    history=used_seeds(plan["root"],plan["output"])
    conflicts=[] if plan["allow_training_reuse"] else [s for s in plan["seeds"] if (plan["entry_id"],s) in history]
    expired=utc_now()>=aware(plan["cutoff_utc"])
    report={"schema":"mission3-series-dry-run/1","valid":not differences and not conflicts and not expired,
            "source_changes":differences,"new_seed_conflicts":conflicts,"scheduled":len(plan["jobs"]),
            "deadline_expired":expired,
            "cutoff_utc":plan["cutoff_utc"],"argv":[j["argv"] for j in plan["jobs"]],"workers_started":0,
            "claims":plan["claims"]}
    save(Path(plan["output"])/"dry-run.json",report)
    return report


def execute_plan(path,*,now_fn=utc_now,sleep_fn=time.sleep):
    plan=read_plan(path);out=Path(plan["output"])
    if (out/"state.json").exists(): raise ValueError("series already started; no implicit retry/resume")
    try:
        with (out/".series-run-owner").open("x") as file:
            file.write(f"coordinator_pid={os.getpid()}\ncoordinator_pgid={os.getpgrp()}\n")
    except FileExistsError as exc: raise ValueError("series already started or claimed; no implicit retry/resume") from exc
    state={"schema":SCHEMA,"series_id":plan["series_id"],"plan_sha256":sha(path),
           "started_at_utc":now_fn().isoformat(),"peak_active":0,"stop_reason":None,"events":[],
           "jobs":[{**j,"status":"planned","pid":None,"pgid":None} for j in plan["jobs"]],"claims":plan["claims"]}
    pending=list(state["jobs"]);active={};blocked=None
    logs=out/"logs";logs.mkdir(exist_ok=True)
    def event(kind,**fields):
        state["events"].append({"t_utc":now_fn().isoformat(),"event":kind,**fields})
    def persist(): save(out/"state.json",summarize(state))
    def block(reason,detail=None):
        nonlocal blocked
        if blocked is None:
            blocked=reason;state["stop_reason"]=reason;event("future_launches_aborted",reason=reason,detail=detail)
        for job in pending: job["status"]="not_launched_"+blocked
        pending.clear()
    def signal_owned(item,sig):
        proc=item["process"];job=item["job"]
        if proc.poll() is not None: return
        try:
            if os.getpgid(proc.pid)!=job["pgid"] or job["pgid"]!=proc.pid:
                raise RuntimeError("owned process group identity changed")
            os.killpg(job["pgid"],sig)
            event("signal_owned_group",pid=proc.pid,pgid=job["pgid"],signal=signal.Signals(sig).name)
        except ProcessLookupError: pass
    def request_stop(item,reason):
        if item["stopping"] is None:
            item["stopping"]=reason;item["signal_at"]=time.monotonic();item["signal_stage"]=1
            signal_owned(item,signal.SIGINT)
    def finish(item,code):
        job=item["job"];job["returncode"]=code;job["ended_at_utc"]=now_fn().isoformat()
        item["log"].close();job["manifest_check"]=manifest_check(plan,job)
        if item["stopping"]: job["status"]=item["stopping"]
        elif job.get("identity_changed_during_run"): job["status"]="identity_changed"
        elif code==0 and job["manifest_check"]["valid"]: job["status"]="worker_succeeded"
        else: job["status"]="worker_failed"
        event("worker_finished",job_id=job["job_id"],pid=job["pid"],status=job["status"],returncode=code)
    def poll_active():
        for job_id,item in list(active.items()):
            code=item["process"].poll()
            if code is not None:
                finish(item,code);del active[job_id];continue
            elapsed=time.monotonic()-item["started_monotonic"]
            if elapsed>=plan["worker_timeout_s"]: request_stop(item,"timed_out")
            if item["stopping"]:
                age=time.monotonic()-item["signal_at"]
                if item["signal_stage"]==1 and age>=plan["signal_grace_s"]:
                    signal_owned(item,signal.SIGTERM);item["signal_stage"]=2;item["signal_at"]=time.monotonic()
                elif item["signal_stage"]==2 and age>=plan["terminate_grace_s"]:
                    signal_owned(item,signal.SIGKILL);item["signal_stage"]=3
    persist()
    try:
        conflicts=[j["output"] for j in pending if Path(j["output"]).exists()]
        if conflicts: block("output_conflict",conflicts)
        while pending or active:
            differences=check_identity(plan)
            if differences:
                block("identity_change",differences)
                for item in active.values(): item["job"]["identity_changed_during_run"]=True
            if now_fn()>=aware(plan["cutoff_utc"]):
                block("cutoff")
                for item in active.values(): request_stop(item,"cancelled_at_cutoff")
            poll_active()
            while pending and len(active)<plan["max_workers"]:
                differences=check_identity(plan)
                if differences: block("identity_change",differences);break
                if (aware(plan["cutoff_utc"])-now_fn()).total_seconds()<=plan["launch_margin_s"]:
                    block("cutoff");break
                history=used_seeds(plan["root"],plan["output"])
                job=pending[0]
                if not plan["allow_training_reuse"] and (job["entry_id"],job["seed"]) in history:
                    block("seed_conflict",{"seed":job["seed"]});break
                if (aware(plan["cutoff_utc"])-now_fn()).total_seconds()<=plan["launch_margin_s"]:
                    block("cutoff");break
                pending.pop(0)
                job["status"]="launch_failed"
                directory=Path(job["output"]);directory.parent.mkdir(parents=True,exist_ok=True)
                directory.mkdir(exist_ok=False)
                log=(logs/(job["job_id"]+".log")).open("xb")
                env={**os.environ,"PYTHONNOUSERSITE":"1","PYTHONDONTWRITEBYTECODE":"1","MUJOCO_GL":os.environ.get("MUJOCO_GL","egl")}
                job["start_requested_at_utc"]=now_fn().isoformat()
                if (aware(plan["cutoff_utc"])-aware(job["start_requested_at_utc"])).total_seconds()<=plan["launch_margin_s"]:
                    log.close();job["status"]="not_launched_cutoff";block("cutoff");break
                try:
                    proc=subprocess.Popen(job["argv"],cwd=plan["root"],env=env,stdout=log,stderr=subprocess.STDOUT,
                                          shell=False,start_new_session=True,close_fds=True)
                except OSError as exc:
                    log.close();job["status"]="launch_failed";job["launch_error"]=str(exc)
                    block("launch_error",str(exc));break
                job.update(status="running",pid=proc.pid,pgid=proc.pid,started_at_utc=now_fn().isoformat())
                active[job["job_id"]]={"job":job,"process":proc,"log":log,"started_monotonic":time.monotonic(),
                    "stopping":None,"signal_stage":0,"signal_at":None}
                state["peak_active"]=max(state["peak_active"],len(active))
                event("worker_started",job_id=job["job_id"],pid=proc.pid,pgid=proc.pid,argv=job["argv"])
                persist()
            persist()
            if active: sleep_fn(plan["poll_interval_s"])
    except KeyboardInterrupt:
        block("interrupted")
        for item in active.values(): request_stop(item,"interrupted")
    except Exception as exc:
        block("coordinator_error",f"{type(exc).__name__}: {exc}")
        for item in active.values(): request_stop(item,"coordinator_error")
    finally:
        while active:
            poll_active();persist()
            if active: time.sleep(min(plan["poll_interval_s"],.1))
        state["ended_at_utc"]=now_fn().isoformat();persist()
        save(out/"summary.json",summarize(state))
    return state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest="command",required=True)
    p=commands.add_parser("plan")
    p.add_argument("--out",type=Path,required=True);p.add_argument("--root",type=Path,default=ROOT)
    p.add_argument("--profile",type=Path);p.add_argument("--entry",type=int,default=1)
    p.add_argument("--seeds",type=int,nargs="+",required=True);p.add_argument("--split",choices=("training","holdout"),default="holdout")
    p.add_argument("--series",required=True);p.add_argument("--workers",type=int,default=3)
    p.add_argument("--cutoff");p.add_argument("--worker-timeout",type=float,default=900.)
    p.add_argument("--timeout",type=float);p.add_argument("--video",action="store_true")
    p.add_argument("--allow-near-focus",action="store_true");p.add_argument("--perturb-entry",action="store_true")
    p.add_argument("--allow-training-reuse",action="store_true")
    for name in ("dry-run","run"):
        command=commands.add_parser(name);command.add_argument("--plan",type=Path,required=True)
    a=parser.parse_args()
    try:
        if a.command=="plan":
            path=create_plan(a.out,root=a.root,profile=a.profile,entry_id=a.entry,seeds=a.seeds,split=a.split,
                series_id=a.series,max_workers=a.workers,cutoff=a.cutoff,worker_timeout_s=a.worker_timeout,
                mission_timeout_s=a.timeout,video=a.video,allow_near_focus=a.allow_near_focus,
                perturb_entry=a.perturb_entry,allow_training_reuse=a.allow_training_reuse)
            print(json.dumps({"plan":str(path),"workers_started":0}));return 0
        if a.command=="dry-run":
            result=dry_run(a.plan);print(json.dumps(result,indent=2));return 0 if result["valid"] else 2
        def interrupted(signum,frame):
            # 한번 중단을 받은 뒤 bounded drain 동안 재차 신호로 worker가 남지 않게 한다.
            signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
            raise KeyboardInterrupt
        previous={sig:signal.signal(sig,interrupted) for sig in (signal.SIGINT,signal.SIGTERM)}
        try: result=execute_plan(a.plan)
        finally:
            for sig,handler in previous.items(): signal.signal(sig,handler)
        print(json.dumps(result["counts"],indent=2))
        return 0 if result["counts"]["worker_succeeded"]==result["counts"]["scheduled"] else 2
    except (OSError,ValueError,KeyError) as exc:
        print(json.dumps({"error":f"{type(exc).__name__}: {exc}"}));return 2


if __name__=="__main__": raise SystemExit(main())
