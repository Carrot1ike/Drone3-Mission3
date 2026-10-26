"""Host command contract without connecting to or modifying firmware."""
import json
from queue import Queue
import sys
from types import ModuleType, SimpleNamespace

from mission2_fresh.commands import command_worker
from mission2_fresh.mission import Mission


def test_sender_uses_only_original_hover_and_stop(tmp_path, monkeypatch):
    sent=[]
    class CF:
        def __init__(self, **kwargs):
            self.commander=SimpleNamespace(
                send_hover_setpoint=lambda *args:sent.append(('hover',args)),
                send_stop_setpoint=lambda:sent.append(('stop',())))
        @property
        def param(self):
            raise AssertionError('ordinary hover must not change firmware parameters')
    class Connection:
        def __init__(self, uri, cf):assert uri=='udp://127.0.0.1:20050'
        def __enter__(self):return self
        def __exit__(self,*args):pass
    class Stop:
        calls=0
        def wait(self, delay):
            assert delay==.02
            self.calls+=1
            return self.calls>1
        def set(self):pass
    package=ModuleType('cflib');crtp=ModuleType('cflib.crtp')
    crtp.init_drivers=lambda:None;package.crtp=crtp
    cf_module=ModuleType('cflib.crazyflie');cf_module.Crazyflie=CF
    sync=ModuleType('cflib.crazyflie.syncCrazyflie');sync.SyncCrazyflie=Connection
    for module in [package,crtp,cf_module,sync]:monkeypatch.setitem(sys.modules,module.__name__,module)
    monkeypatch.setattr('mission2_fresh.commands.signal.signal',lambda *args:None)
    monkeypatch.setattr(sys,'path',sys.path.copy())
    updates=Queue();ready=Queue()
    command=dict(mode='hover',vx=.2,vy=-.1,yawrate=3.,height_m=1.2)
    updates.put((command,dict(sequence=72),[1,2,3]))
    command_worker(tmp_path,updates,Stop(),ready)
    assert ready.get_nowait()=={'ok':True}
    assert sent==[('hover',(.2,-.1,3.,1.2)),('stop',())]
    logs=[json.loads(s) for s in (tmp_path/'commands.jsonl').read_text().splitlines()]
    assert logs[0]['command']==command
    assert logs[-1]['verdict']=='explicit_stop'
    assert all('ring_pass_ack' not in row and 'ring_pass_mask' not in row['command'] for row in logs)


def test_ring_crossing_finishes_without_firmware_marker():
    import numpy as np
    mission=Mission();mission.stage='STRAIGHT';mission.distance=.4
    mission.last=0.;mission.travel=.5
    state=dict(quaternion_xyzw=[0.,0.,0.,1.],ekf_height_m=1.,body_velocity_flu=[0.,0.,0.])
    command=mission.step(np.zeros((244,324),np.uint8),state,.1)
    assert mission.stage=='BRAKE' and mission.index==1
    assert set(command)=={'mode','vx','vy','yawrate','height_m'}
