"""Original cflib hover commands on a process independent of image processing."""
import json
from pathlib import Path
from queue import Empty
import signal
import sys
import time


def command_worker(output, updates, stop, ready, ring_mask=0):
    baseline=Path('/home/razer/Desktop/Drone3')
    sys.path.insert(0,str(baseline/'crazyflie-lib-python'))
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    ack=0;current=None;mask=0;announced=False
    with (output/'commands.jsonl').open('w',buffering=1) as commands, (output/'events.jsonl').open('w',buffering=1) as events:
        def event(row):events.write(json.dumps(row)+'\n')
        def parameter_ack(name,value):
            nonlocal ack
            ack=int(value)
            event(dict(event='ring_pass_parameter_ack',parameter=name,value=ack,time=time.monotonic()))
        def record(command,verdict='active'):
            if current is None:return
            _,frame,identity=current
            commands.write(json.dumps(dict(sent_monotonic=time.monotonic(),verdict=verdict,
                source_frame=frame,source_identity=identity,source_epoch=0,command=command,**({'ring_pass_ack':ack} if ring_mask else {})))+'\n')
        try:
            cflib.crtp.init_drivers()
            cf=Crazyflie(rw_cache=str(output/'crtp_cache'))
            with SyncCrazyflie('udp://127.0.0.1:20050',cf=cf):
                try:
                    if ring_mask:
                        started=time.monotonic()
                        while not cf.param.is_updated and time.monotonic()-started<30:time.sleep(.05)
                        cf.param.add_update_callback(group='kalman',name='ringPass',cb=parameter_ack)
                        cf.param.set_value('kalman.ringPass','0')
                    ready.put(dict(ok=True));announced=True
                    while not stop.wait(.02):
                        try:
                            while True:current=updates.get_nowait()
                        except Empty:pass
                        if current is None:continue
                        command,_,_=current
                        requested=command.get('ring_pass_mask',0) if ring_mask else 0
                        if requested!=mask:
                            cf.param.set_value('kalman.ringPass',str(requested));mask=requested
                            event(dict(event='ring_pass_requested',value=mask,time=time.monotonic()))
                        sent=command.copy()
                        if mask and ack!=mask:sent['vx']=sent['vy']=0.
                        if sent['mode']=='stop':
                            cf.commander.send_stop_setpoint();record(sent,'explicit_stop')
                        else:
                            cf.commander.send_hover_setpoint(sent['vx'],sent['vy'],sent['yawrate'],sent['height_m'])
                            record(sent)
                finally:
                    if ring_mask:cf.param.set_value('kalman.ringPass','0')
                    cf.commander.send_stop_setpoint()
                    record(dict(mode='stop',vx=0.,vy=0.,yawrate=0.,height_m=0.,**({'ring_pass_mask':0} if ring_mask else {})),'explicit_stop')
                    if ring_mask:
                        deadline=time.monotonic()+1
                        while ack and time.monotonic()<deadline:time.sleep(.01)
        except BaseException as exc:
            error=f'{type(exc).__name__}: {exc}'
            event(dict(event='command_worker_error',error=error,time=time.monotonic()))
            if not announced:ready.put(dict(ok=False,error=error))
            stop.set()
            raise
