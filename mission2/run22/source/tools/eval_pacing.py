"""Experimental, opt-in headless pacing; no native imports or startup on import.

Install on ``source.crazysim`` before ``source.main()``. This copies the reviewed
headless lifecycle/step sequence, NOT any motor, sensor, camera or scene code.
See docs/MISSION2_PACING.md for qualification limits and integration obligations.
"""
from __future__ import annotations

import hashlib
import inspect
import math
from pathlib import Path
import sys
import threading
import time
from types import CodeType


REVIEWED_CRAZYSIM_SHA256 = "ffb1d44c919e5248aeaacbe3a67d691060fc904593d8c5b8981e3d5fd7b2c5f6"
REVIEWED_SOURCE_SHA256 = "00a9bb2ee24d87fd69d77d66140d7d42ed77be28262a2d5a18e89b4a3b4724f8"


class PacingFailure(RuntimeError):
    """Infrastructure failure, never a mission/policy failure or success."""

    def __init__(self, reason, evidence):
        self.reason = reason
        self.evidence = dict(evidence)
        super().__init__(f"physics pacing infrastructure failure: {reason}")


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


class DeadlinePacer:
    """Single-owner fixed-step clock; snapshots may be read from another thread.

    Step n is released at epoch + n*dt, n >= 1. No deadline rebasing, skipped
    steps, enlarged dt, or speed multiplier. Hooks must complete before commit.
    Bounded sleeps allow cancellation; nonadvancing/early-return sleepers cannot
    spin forever. All limits are engineering choices, not sensor calibration.
    """

    def __init__(self, dt, lag_budget_s, *, clock=time.monotonic,
                 sleep=time.sleep, max_sleep_s=.01, max_wait_sleeps=128):
        self.dt = _positive(dt, "dt")
        self.lag_budget_s = _positive(lag_budget_s, "lag_budget_s")
        self.max_sleep_s = _positive(max_sleep_s, "max_sleep_s")
        if isinstance(max_wait_sleeps, bool) or not isinstance(max_wait_sleeps, int) or max_wait_sleeps < 1:
            raise ValueError("max_wait_sleeps must be a positive integer")
        self.max_wait_sleeps = max_wait_sleeps
        self._clock, self._sleep = clock, sleep
        self._lock = threading.Lock()
        self._state = dict(status="ready", failure_reason=None, infrastructure_failure=False,
                           physics_steps=0, dispatch_steps=0, sleep_calls=0,
                           catchup_steps=0, max_consecutive_catchup_steps=0,
                           max_lateness_s=0., wall_elapsed_s=0., sim_elapsed_s=0.,
                           wall_minus_sim_s=0., requested_sleep_s=0.,
                           run_finally_entered=False, run_finally_completed=False)
        self._epoch = self._last_clock = self._sim_start = None
        self._consecutive_catchup = 0
        self._waiting = False

    def _publish(self, **values):
        with self._lock:
            self._state.update(values)

    def snapshot(self):
        """JSON-safe evidence through the most recent owner-thread observation."""
        with self._lock:
            return dict(self._state)

    def fail(self, reason):
        self._publish(status="infrastructure_failure", infrastructure_failure=True,
                      failure_reason=reason)
        raise PacingFailure(reason, self.snapshot())

    def _now(self):
        now = float(self._clock())
        if not math.isfinite(now):
            self.fail("nonfinite_monotonic_clock")
        if self._last_clock is not None and now < self._last_clock:
            self.fail("monotonic_clock_regressed")
        self._last_clock = now
        if self._epoch is not None:
            wall = now - self._epoch
            elapsed = self._state["sim_elapsed_s"]
            self._publish(wall_elapsed_s=wall,
                          wall_minus_sim_s=wall - elapsed if elapsed is not None else None)
        return now

    def start(self, sim_time):
        if self._epoch is not None:
            raise RuntimeError("pacer is single-use")
        self._sim_start = float(sim_time)
        if not math.isfinite(self._sim_start):
            self.fail("nonfinite_simulation_clock")
        self._epoch = self._now()
        self._publish(status="running", epoch_monotonic_s=self._epoch,
                      start_sim_time_s=self._sim_start, timestep_s=self.dt,
                      lag_budget_s=self.lag_budget_s)

    def _check_lateness(self, lateness, phase):
        lateness = max(0., lateness)
        self._publish(max_lateness_s=max(self._state["max_lateness_s"], lateness),
                      last_lateness_s=lateness, last_check_phase=phase)
        if lateness > self.lag_budget_s:
            self.fail("lag_budget_exceeded")

    def wait_next(self, running):
        if self._epoch is None:
            raise RuntimeError("start the pacer first")
        if self._state["physics_steps"] != self._state["dispatch_steps"]:
            self.fail("incomplete_sensor_dispatch")
        target = self._epoch + (self._state["physics_steps"] + 1) * self.dt
        self._waiting = True
        self._publish(next_deadline_monotonic_s=target)
        slept = False
        for attempt in range(self.max_wait_sleeps + 1):
            if not running():
                return False
            now = self._now()
            if now >= target:
                self._check_lateness(now - target, "before_step")
                catchup = not slept and now > target
                self._consecutive_catchup = self._consecutive_catchup + 1 if catchup else 0
                self._publish(catchup_steps=self._state["catchup_steps"] + int(catchup),
                              max_consecutive_catchup_steps=max(
                                  self._state["max_consecutive_catchup_steps"], self._consecutive_catchup))
                self._waiting = False
                return bool(running())
            if attempt == self.max_wait_sleeps:
                self.fail("deadline_wait_attempt_limit")
            duration = min(target - now, self.max_sleep_s)
            self._publish(sleep_calls=self._state["sleep_calls"] + 1,
                          requested_sleep_s=self._state["requested_sleep_s"] + duration)
            self._sleep(duration)
            slept = True
        raise AssertionError("bounded deadline wait exhausted")

    def physics_advanced(self, sim_time):
        """Count mj_step immediately, including a later sensor/hook exception."""
        sim_time = float(sim_time)
        elapsed = sim_time - self._sim_start
        self._publish(physics_steps=self._state["physics_steps"] + 1,
                      sim_elapsed_s=elapsed if math.isfinite(elapsed) else None)

    def dispatch_complete(self):
        self._publish(dispatch_steps=self._state["dispatch_steps"] + 1)
        expected = self._state["physics_steps"] * self.dt
        actual = self._state["sim_elapsed_s"]
        if actual is None or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-8):
            self.fail("fixed_step_clock_invariant")
        now = self._now()
        self._check_lateness(now - (self._epoch + expected), "after_dispatch")

    def finish(self, status):
        now = self._now()
        if self._epoch is not None:
            target = (self._state["next_deadline_monotonic_s"] if self._waiting else
                      self._epoch + self._state["physics_steps"] * self.dt)
            if self._state["infrastructure_failure"]:
                self._publish(max_lateness_s=max(self._state["max_lateness_s"], max(0., now - target)))
            else:
                self._check_lateness(now - target, "run_exit")
        elif "run_entry_monotonic_s" in self._state:
            self._publish(startup_elapsed_s=now - self._state["run_entry_monotonic_s"])
        if not self._state["infrastructure_failure"]:
            self._publish(status=status)


def _nested_code(code, name):
    return next(part for part in code.co_consts if isinstance(part, CodeType) and part.co_name == name)


def _verify_reviewed(crazysim):
    """Check both the on-disk review and the actual loaded run implementation."""
    try:
        path = Path(crazysim.__file__)
        raw = path.read_bytes()
        run = crazysim.CrazySimMuJoCo.run
        if hashlib.sha256(raw).hexdigest() != REVIEWED_CRAZYSIM_SHA256:
            raise ValueError("crazysim.py source hash is unreviewed")
        compiled = compile(raw, str(path), "exec", dont_inherit=True, optimize=sys.flags.optimize)
        reviewed_run = _nested_code(_nested_code(compiled, "CrazySimMuJoCo"), "run")
        if (run.__code__ != reviewed_run or run.__globals__ is not vars(crazysim)
                or str(inspect.signature(run)) != "(self)"):
            raise ValueError("loaded CrazySimMuJoCo.run API/code differs from reviewed source")
        for name in ("mj_step", "mj_kinematics"):
            if not callable(getattr(crazysim.mujoco, name)):
                raise ValueError(f"missing MuJoCo API: {name}")
    except (AttributeError, OSError, StopIteration, TypeError) as exc:
        raise ValueError("cannot verify reviewed crazysim source/API") from exc


class PacingInstallation:
    """Install receipt; last/active trial evidence is available without I/O."""

    def __init__(self, enabled, config):
        self.enabled, self.config = enabled, dict(config)
        self._pacer = None
        self._last = dict(status="installed" if enabled else "disabled",
                          infrastructure_failure=False, failure_reason=None)

    def metadata(self):
        return dict(schema=1, adapter="experimental_absolute_headless_v1", enabled=self.enabled,
                    qualification="engineering_scheduling_limit_not_sensor_calibration",
                    reviewed_crazysim_sha256=REVIEWED_CRAZYSIM_SHA256,
                    reviewed_source_sha256=REVIEWED_SOURCE_SHA256,
                    constructor_timing="outside_run_not_measured_by_adapter",
                    epoch_boundary="after_handshake_and_agent_start_threads_before_first_step",
                    process_scope="single_trial_no_restore", **self.config)

    def snapshot(self):
        state = self._pacer.snapshot() if self._pacer is not None else dict(self._last)
        return dict(self.metadata(), **state)


def install(crazysim, *, enabled=False, lag_budget_s=None, clock=time.monotonic,
            sleep=time.sleep, max_sleep_s=.01, max_wait_sleeps=128):
    """Opt in before source.main; disabled calls do not inspect/mutate crazysim.

    An enabled install requires an explicit lag budget, verifies the reviewed
    native run, and rejects GUI runs. It does not launch anything or patch time.
    Parent integration must persist snapshot(), catch PacingFailure and invalidate
    the trial; this module deliberately does not modify eval_source or its CLI.
    """
    if not enabled:
        return PacingInstallation(False, {})
    if lag_budget_s is None:
        raise ValueError("enabled pacing requires an explicit lag_budget_s")
    # Validate configuration before mutating the class.
    probe = DeadlinePacer(.001, lag_budget_s, clock=clock, sleep=sleep,
                          max_sleep_s=max_sleep_s, max_wait_sleeps=max_wait_sleeps)
    _verify_reviewed(crazysim)
    receipt = PacingInstallation(True, dict(lag_budget_s=probe.lag_budget_s,
        max_sleep_s=probe.max_sleep_s, max_wait_sleeps=probe.max_wait_sleeps))
    cls = crazysim.CrazySimMuJoCo

    def run(sim):
        # Copy of reviewed headless lifecycle/step sequence at baseline 1298-1398.
        # Resource cleanup is strengthened to finally, including startup errors.
        pacer = None
        status = "stopped"
        primary = None
        deferred = None
        handshake_threads = []
        cleanup_errors = []
        try:
            if sim.visualize:
                raise ValueError("experimental physics pacing supports HEADLESS only")
            if receipt._pacer is not None:
                pacer = receipt._pacer
                pacer.fail("installation_reused")
            pacer = DeadlinePacer(sim.dt, probe.lag_budget_s, clock=clock, sleep=sleep,
                                  max_sleep_s=probe.max_sleep_s, max_wait_sleeps=probe.max_wait_sleeps)
            receipt._pacer = pacer
            run_entry = pacer._now()
            pacer._publish(run_entry_monotonic_s=run_entry)
            for agent in sim.agents:
                t = crazysim.threading.Thread(target=agent.handshake)
                t.start()
                handshake_threads.append(t)
            for t in handshake_threads:
                t.join()
            handshake_done = pacer._now()
            pacer._publish(handshake_elapsed_s=handshake_done - run_entry)

            sim._running = True
            for agent in sim.agents:
                agent.start_threads()
            pacer.start(sim.data.time)
            pacer._publish(startup_elapsed_s=pacer._epoch - run_entry,
                           agent_start_threads_elapsed_s=pacer._epoch - handshake_done)
            while sim._running:
                if not pacer.wait_next(lambda: sim._running):
                    break
                if (float(sim.dt) != pacer.dt or float(sim.model.opt.timestep) != pacer.dt
                        or any(float(agent.dt) != pacer.dt for agent in sim.agents)):
                    pacer.fail("timestep_changed")
                for agent in sim.agents:
                    agent.update_motors()
                    agent.apply_aero_effects()
                    agent.update_led_materials()
                if sim._downwash and sim.num_agents > 1:
                    sim._apply_downwash()
                if sim._scene_adapter is not None:
                    sim._scene_adapter.update(sim.model, sim.data)
                crazysim.mujoco.mj_step(sim.model, sim.data)
                pacer.physics_advanced(sim.data.time)
                if sim._scene_adapter is not None:
                    sim._scene_adapter.update(sim.model, sim.data)
                    crazysim.mujoco.mj_kinematics(sim.model, sim.data)
                for agent in sim.agents:
                    agent.send_sensor_data()
                pacer.dispatch_complete()
        except KeyboardInterrupt:
            status = "interrupted"  # Same headless KeyboardInterrupt return behavior.
        except BaseException as exc:
            primary = exc
            status = "infrastructure_failure"
            if pacer is not None:
                pacer._publish(status=status, infrastructure_failure=True,
                    failure_reason=getattr(exc, "reason", "run_exception"),
                    exception=f"{type(exc).__name__}: {exc}")
            else:
                receipt._last.update(status=status, infrastructure_failure=True,
                                     failure_reason="run_rejected", exception=f"{type(exc).__name__}: {exc}")
            raise
        finally:
            sim._running = False
            receipt._last.update(run_finally_entered=True)
            if pacer is not None:
                pacer._publish(run_finally_entered=True)
            # Sample the physics interval before shutdown; cleanup time is separate.
            if pacer is not None:
                try:
                    pacer.finish(status)
                except BaseException as exc:
                    deferred = exc
                    cleanup_errors.append(f"final_clock: {type(exc).__name__}: {exc}")
            for agent in sim.agents:
                try:
                    agent.stop()
                except BaseException as exc:
                    cleanup_errors.append(f"agent.stop: {type(exc).__name__}: {exc}")
            # Normal joins already completed; after interruption, close sockets
            # first and give each owned handshake thread a bounded exit join.
            for t in handshake_threads:
                try:
                    t.join(timeout=1.)
                    if t.is_alive():
                        cleanup_errors.append("handshake_thread_shutdown_timeout")
                except BaseException as exc:
                    cleanup_errors.append(f"handshake_join: {type(exc).__name__}: {exc}")
            if pacer is not None:
                pacer._publish(run_finally_completed=True, agent_stop_attempts=len(sim.agents),
                               cleanup_errors=list(cleanup_errors))
            receipt._last.update(run_finally_completed=True, agent_stop_attempts=len(sim.agents),
                                 cleanup_errors=list(cleanup_errors))
            if cleanup_errors:
                if pacer is not None:
                    pacer._publish(status="infrastructure_failure", infrastructure_failure=True,
                        failure_reason=pacer.snapshot()["failure_reason"] or "cleanup_failure",
                        cleanup_errors=cleanup_errors)
                else:
                    receipt._last.update(status="infrastructure_failure", infrastructure_failure=True,
                                         cleanup_errors=cleanup_errors)
                if primary is None and deferred is None:
                    raise PacingFailure("cleanup_failure", receipt.snapshot())
            if isinstance(primary, PacingFailure):
                primary.evidence = receipt.snapshot()
            if primary is None and deferred is not None:
                if isinstance(deferred, PacingFailure):
                    deferred.evidence = receipt.snapshot()
                raise deferred
            if primary is None:
                print("[crazysim] Done.")

    cls.run = run
    return receipt
