"""Instrument the real baseline source; GT never enters its camera/state transport."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from eval_common import (DEFAULT_BASELINE, baseline_modules, load_module, local_path,
                         provenance, workspace_builder, write_json, write_png)


class EvaluationLog:
    """Close evidence is about this wrapper, not a crash-safe flight recorder."""
    def __init__(self, output, session):
        self.output = output
        self.stream = (output / 'ground_truth.jsonl').open('w', buffering=1)
        self.trace = None
        self.closed = False
        self.status = dict(schema=1, evaluation_only=True, session=session,
            wrapper_started_host_monotonic=time.monotonic(), wrapper_closed=False,
            source_main_returned=False, physics_dispatches=0, gt_rows=0,
            contact_pair_changes=0, sim_time_regressions=0, host_time_regressions=0,
            first_dispatch=None, last_dispatch=None)
        write_json(output / 'evaluation_lifecycle.json', self.status)

    def write(self, value):
        return self.stream.write(value)

    def flush(self):
        return self.stream.flush()

    def close(self):
        if self.closed:
            return
        if self.trace is not None:
            self.trace.close()
            self.status['camera_timing'] = self.trace.status()
        self.stream.close()
        self.status.update(wrapper_closed=True, wrapper_closed_host_monotonic=time.monotonic())
        write_json(self.output / 'evaluation_lifecycle.json', self.status)
        self.closed = True


class EvaluationHooks:
    """Own opt-in adapters through source shutdown; persist actual hook status."""
    def __init__(self, source, config, output, source_args):
        self.source, self.config, self.output, self.source_args = source, config, output, source_args
        self.conditions = self.pacing = None
        self.original_run = self.installed_run = None
        self.pacing_failure_type = ()
        self.status = dict(schema=1, evaluation_only=True, policy_may_read=False,
            phase='preparing', conditions_installed=False, conditions_closed=None,
            pacing_installed_snapshot=None, pacing_final_snapshot=None, errors=[])

    def install(self):
        if os.environ.get('DRONE3_EVAL_SOURCE_HOOK') or os.environ.get('DRONE3_EVAL_SOURCE_HOOK_CONFIG'):
            raise ValueError('use copied top-level evaluation_conditions/evaluation_pacing config, not source hook env')
        if not isinstance(self.config, dict):
            raise ValueError('runtime config must be an object')
        conditions = self.config.get('evaluation_conditions')
        pacing = self.config.get('evaluation_pacing')
        if conditions is not None:
            if not isinstance(conditions, dict) or type(conditions.get('enabled')) is not bool:
                raise ValueError('evaluation_conditions requires a full object with boolean enabled')
            if conditions['enabled'] and (self.config.get('arena', {}).get('scenario') != 'mission2'
                    or self.config.get('flowdeck') is not True or '--flowdeck' not in self.source_args
                    or self.config.get('evaluation_setup') is not None):
                raise ValueError('enabled conditions require official mission2, flowdeck=true/--flowdeck, no evaluation_setup')
            from eval_conditions import resolve_conditions
            conditions = resolve_conditions(conditions)
        if pacing is not None:
            if (not isinstance(pacing, dict) or set(pacing)-{'enabled', 'lag_budget_s'}
                    or type(pacing.get('enabled')) is not bool):
                raise ValueError('evaluation_pacing permits only enabled:boolean and lag_budget_s:number')
            budget = pacing.get('lag_budget_s')
            if ('lag_budget_s' in pacing and (isinstance(budget, bool) or not isinstance(budget, (int, float))
                    or not math.isfinite(budget) or budget <= 0)) or (pacing['enabled'] and budget is None):
                raise ValueError('enabled pacing requires finite positive lag_budget_s; booleans are not numbers')
            if pacing['enabled'] and '--gui' in self.source_args:
                raise ValueError('enabled evaluation_pacing requires headless source')
        self.status.update(config=dict(evaluation_conditions=conditions, evaluation_pacing=pacing), phase='installing')
        self.persist()
        if pacing is not None:
            from eval_pacing import install, PacingFailure
            self.pacing_failure_type = PacingFailure
            self.original_run = self.source.crazysim.CrazySimMuJoCo.run
            self.pacing = install(self.source.crazysim, **pacing)
            self.installed_run = self.source.crazysim.CrazySimMuJoCo.run
            self.status['pacing_installed_snapshot'] = self.pacing.snapshot()
        if conditions is not None:
            from eval_conditions import install_conditions
            self.conditions = install_conditions(self.source, conditions, self.output)
            self.status['conditions_installed'] = self.conditions is not None
        self.status['phase'] = 'installed'
        self.persist()

    def persist(self):
        write_json(self.output / 'evaluation_hooks.json', self.status)

    def close(self, *, source_main_returned):
        self.status['source_main_returned'] = source_main_returned
        if self.conditions is not None:
            try:
                self.conditions.close()
                self.status['conditions_closed'] = True
            except BaseException as exc:
                self.status['conditions_closed'] = False
                self.status['errors'].append(f'conditions.close: {type(exc).__name__}: {exc}')
        if self.pacing is not None:
            try:
                self.status['pacing_final_snapshot'] = self.pacing.snapshot()
            except BaseException as exc:
                self.status['errors'].append(f'pacing.snapshot: {type(exc).__name__}: {exc}')
            if self.source.crazysim.CrazySimMuJoCo.run is self.installed_run:
                self.source.crazysim.CrazySimMuJoCo.run = self.original_run
        self.status['phase'] = 'closed'
        try:
            self.persist()
        except BaseException as exc:
            self.status['errors'].append(f'hooks.persist: {type(exc).__name__}: {exc}')


def run_configured_source(source, config, output, source_args, log):
    """Called after GT/capture/setup installation, before simulator construction."""
    hooks = EvaluationHooks(source, config, output, source_args)
    primary = None
    try:
        hooks.install()
        source.main()
        log.status['source_main_returned'] = True
    except BaseException as exc:
        primary = exc
        log.status['source_error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        hooks.close(source_main_returned=log.status['source_main_returned'])
        pacing = hooks.status['pacing_final_snapshot'] or {}
        failed = pacing.get('infrastructure_failure', False) or isinstance(primary, hooks.pacing_failure_type)
        log.status['infrastructure_failure'] = bool(failed)
        log.status['hook_cleanup_errors'] = list(hooks.status['errors'])
        log.status['source_run_status'] = ('infrastructure_failure' if failed else 'failed' if primary
            else 'cleanup_failure' if hooks.status['errors'] else 'returned')
        if hooks.status['errors'] and primary is None:
            raise RuntimeError('evaluation hook cleanup failed: '+'; '.join(hooks.status['errors']))


def model_metadata(model, data, mujoco):
    import numpy as np
    rings = {}
    for index in range(model.ngeom):
        name = model.geom(index).name
        if name.startswith('ring') and '_' in name and name.split('_')[0][4:].isdigit():
            rings.setdefault(name.split('_')[0], []).append(index)
    gates = []
    for name, indices in sorted(rings.items(), key=lambda item: int(item[0][4:])):
        centers = data.geom_xpos[indices]
        center = centers.mean(axis=0)
        radius = float(np.linalg.norm(centers[:, 1:] - center[1:], axis=1).mean())
        tube = float(model.geom_size[indices[0], 0])
        gates.append(dict(name=name, center_world_m=center.tolist(), normal_world=[1., 0., 0.],
                          aperture_radius_m=radius - tube, expected_direction=-1))
    pad = None
    try:
        geom = model.geom('landing_pad')
        pad = dict(name='landing_pad', center_rover_m=geom.pos.tolist(),
                   half_size_m=geom.size.tolist(), top_rover_z_m=float(geom.pos[2] + geom.size[2]))
    except KeyError:
        pass
    from eval_entry import metadata_from_model
    return dict(rings=gates, landing_pad=pad, entry_sections=metadata_from_model(model,data), drone_body='cf0_drone', camera='cf0_fpv_cam',
                camera_id=int(model.camera('cf0_fpv_cam').id),
                scoring_scope='ring crossings and pad contact only; walls/whole mission require independent review')


def sample_truth(model, data, mujoco, *, session, agent=None, rover_motion=None):
    import numpy as np
    def body(name):
        b = model.body(name)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, b.id, velocity, 0)
        return dict(name=name, position_world_m=data.xpos[b.id].tolist(),
                    quaternion_wxyz=data.xquat[b.id].tolist(),
                    linear_velocity_world_mps=velocity[3:].tolist(),
                    angular_velocity_world_radps=velocity[:3].tolist())
    drone = body('cf0_drone')
    # qvel is the free-joint origin velocity; objectVelocity is COM velocity.
    joint = int(model.body('cf0_drone').jntadr[0])
    vadr = int(model.jnt_dofadr[joint])
    drone['free_joint_velocity_world_mps'] = data.qvel[vadr:vadr+3].tolist()
    rover = None
    try:
        rover = body('mission3_vehicle')
        if rover_motion is not None:
            motion = rover_motion.sample(float(data.time))
            rover['linear_velocity_world_mps'] = motion['velocity'].tolist()
            rover['angular_velocity_world_radps'] = [0., 0., float(motion['yaw_rate'])]
            rover['velocity_source'] = 'prescribed rover motion (mocap velocity is otherwise zero)'
        rotation = data.xmat[model.body('mission3_vehicle').id].reshape(3, 3)
        rover['drone_position_rover_m'] = (rotation.T @ (
            data.xpos[model.body('cf0_drone').id] - data.xpos[model.body('mission3_vehicle').id])).tolist()
    except KeyError:
        pass
    contacts = []
    for contact_index, contact in enumerate(data.contact[:data.ncon]):
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if g1 < 0 or g2 < 0:
            continue
        body_names = [model.body(int(model.geom_bodyid[g])).name for g in (g1, g2)]
        if not any(name.startswith('cf0_') for name in body_names):
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, contact_index, force)
        contacts.append(dict(geoms=[model.geom(g1).name, model.geom(g2).name], bodies=body_names,
                             distance_m=float(contact.dist), normal_force_n=float(force[0]),
                             position_world_m=contact.pos.tolist()))
    row = dict(event='ground_truth', schema=1, evaluation_only=True, session=session,
               host_monotonic=time.monotonic(), sim_time_s=float(data.time),
               sim_time_ns=round(data.time * 1e9), drone=drone, rover=rover, contacts=contacts)
    if agent is not None:
        row['motor_rpm'] = agent._rpm.tolist()
        row['motor_rpm_reference'] = agent._rpm_ref.tolist()
    return row


def install(source, output, session, hz=50., capture_every=0, *, timing_max_frames=0, timing_every=1):
    """Patch a dispatch boundary after mj_step; no physics, sensor or motor writes."""
    if not 1 <= hz <= 1000 or capture_every < 0 or not 0 <= timing_max_frames <= 100000 or not 1 <= timing_every <= 100000:
        raise ValueError('invalid evaluation sampling limits')
    c = source.crazysim
    from arena import model_builder
    model_builder.build = workspace_builder()
    output = local_path(output)
    output.mkdir(parents=True, exist_ok=True)
    log = EvaluationLog(output, session)
    if timing_max_frames:
        from eval_timing import BoundedCaptureTrace
        log.trace = BoundedCaptureTrace(output / 'timing.jsonl', max_frames=timing_max_frames,
                                       every=timing_every)
    original_send = c.DroneAgent.send_sensor_data
    original_init = c.CrazySimMuJoCo.__init__
    context = dict(last=-math.inf, contacts=None, motion=None, sample_count=0)
    def init(sim, *args, **kwargs):
        original_init(sim, *args, **kwargs)
        context['motion'] = getattr(sim._scene_adapter, 'motion', None)
        write_json(output / 'model_metadata.json', model_metadata(sim.model, sim.data, c.mujoco))
    def dispatch(agent):
        if agent.agent_id == 0:
            stamp = dict(host_monotonic=time.monotonic(), sim_time_s=float(agent.data.time))
            previous = log.status['last_dispatch']
            if previous is not None:
                log.status['sim_time_regressions'] += int(stamp['sim_time_s'] < previous['sim_time_s'])
                log.status['host_time_regressions'] += int(stamp['host_monotonic'] <= previous['host_monotonic'])
            log.status['first_dispatch'] = log.status['first_dispatch'] or stamp
            log.status['last_dispatch'] = stamp
            log.status['physics_dispatches'] += 1
            # Inspect contacts every physics tick so brief collision events survive decimation.
            contacts = tuple(sorted(tuple(sorted((int(item.geom1), int(item.geom2))))
                                    for item in agent.data.contact[:agent.data.ncon]))
            if context['contacts'] is not None and contacts != context['contacts']:
                log.status['contact_pair_changes'] += 1
            if (agent.data.time - context['last'] >= 1. / hz - 1e-9
                    or contacts != context['contacts'] or agent.data.time < context['last']):
                row = sample_truth(agent.model, agent.data, c.mujoco, session=session,
                                   agent=agent, rover_motion=context['motion'])
                row['sample_index'] = context['sample_count']
                row['physics_dispatch_index'] = log.status['physics_dispatches'] - 1
                row['process_cpu_s'] = time.process_time()
                log.write(json.dumps(row, allow_nan=False) + '\n')
                log.status['gt_rows'] += 1
                context.update(last=float(agent.data.time), contacts=contacts,
                               sample_count=context['sample_count'] + 1)
        return original_send(agent)
    c.CrazySimMuJoCo.__init__ = init
    c.DroneAgent.send_sensor_data = dispatch
    if capture_every or log.trace is not None:
        import struct
        import numpy as np
        original_socket = source.FrameSocket
        class CapturingSocket(original_socket):
            def sendto(self, packet, dest):
                index, total, w, h = struct.unpack_from('<HHHH', packet)
                # Native FrameSocket keeps these same unmodified chunks until the final send.
                sequence = self.seq
                complete = (total == 2 and index != 0 and len(self.parts) == total - 1
                            and index not in self.parts)
                selected = log.trace.selected() if complete and log.trace is not None else False
                save_image = complete and capture_every and sequence % capture_every == 0
                raw = None
                if selected or save_image:
                    ready = time.monotonic()
                    pieces = dict(self.parts)
                    pieces[index] = packet[8:]
                    raw = b''.join(pieces[i] for i in range(total))
                    if (w, h, len(raw)) != (324, 244, 324*244):
                        raise ValueError('native camera capture dimensions')
                    if selected:
                        started = getattr(self, 'started', None)
                        if started is not None and not 0 < started <= ready:
                            started = None
                        log.trace.capture(raw, session=getattr(self, 'session', session),
                            sequence=sequence, sim_ns=self.sim_ns, started=started, ready=ready)
                result = super().sendto(packet, dest)
                if self.seq != sequence and save_image:
                    image = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                    name = f'frames/{sequence:08d}.png'
                    digest = write_png(output / name, image)
                    log.write(json.dumps(dict(event='camera_capture', evaluation_only=True,
                        session=session, source_sequence=sequence, sim_time_ns=self.sim_ns,
                        sim_time_s=self.sim_ns / 1e9, host_monotonic=time.monotonic(),
                        image=name, sha256=digest, evidence='native_source_camera_before_transport')) + '\n')
                return result
        source.FrameSocket = CapturingSocket
    return log


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--eval-dir', type=Path, required=True)
    parser.add_argument('--gt-hz', type=float, default=50.)
    parser.add_argument('--capture-every', type=int, default=0)
    parser.add_argument('--timing-max-frames', type=int,
                        default=os.environ.get('DRONE3_EVAL_TIMING_MAX_FRAMES', '0'))
    parser.add_argument('--timing-every', type=int,
                        default=os.environ.get('DRONE3_EVAL_TIMING_EVERY', '1'))
    args, source_args = parser.parse_known_args()
    if (not 1 <= args.gt_hz <= 1000 or args.capture_every < 0
            or not 0 <= args.timing_max_frames <= 100000 or not 1 <= args.timing_every <= 100000):
        parser.error('gt-hz 1..1000; capture-every >=0; timing-max-frames 0..100000; timing-every 1..100000')
    c, runtime, _ = baseline_modules(args.baseline)
    source = load_module('_evaluation_baseline_source', runtime / 'camera_input/source.py')
    session = source_args[source_args.index('--session') + 1]
    output = local_path(args.eval_dir)
    write_json(output / 'provenance.json', dict(provenance(args.baseline),
               evidence='native_instrumented_source', schema=1,
               gt_boundary='DroneAgent.send_sensor_data after native physics step; no GT in transport'))
    log = install(source, output, session, args.gt_hz, args.capture_every,
                  timing_max_frames=args.timing_max_frames, timing_every=args.timing_every)
    try:
        config = {}
        if '--arena-config' in source_args:
            from eval_setup import install_setup
            config_path=Path(source_args[source_args.index('--arena-config')+1])
            config = json.loads(config_path.read_text())
            install_setup(config,output)
        sys.argv = [str(runtime / 'camera_input/source.py'), *source_args]
        run_configured_source(source, config, output, source_args, log)
    finally:
        log.close()


if __name__ == '__main__':
    main()
