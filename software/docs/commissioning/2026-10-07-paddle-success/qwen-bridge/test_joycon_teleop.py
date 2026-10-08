"""Failure-oriented tests using all 16 fake servos; no hardware or network."""
from types import SimpleNamespace
import tempfile, json, time
from pathlib import Path
from gemma_hardware_owner import HardwareOwner
from joycon_teleop import SCOPES, JOINTS, INPUT_TTL, validate_input
from joycon_teleop_api import TeleopAPI
from wheel_pulse_executor import WHEELS

class Plant:
    def __init__(self):
        self.motors = dict.fromkeys(SCOPES['both']+SCOPES['head']+list(WHEELS))
        self.r = {n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=100,Max_Position_Limit=4000,Present_Position=2000,Goal_Position=2000,Goal_Velocity=0,Lock=1,Torque_Limit=1000,Acceleration=0,P_Coefficient=16,Goal_Time=0,Status=0,Present_Load=0,Present_Voltage=120,Present_Temperature=30,Present_Velocity=0) for n in self.motors}
        self.writes = []
    def read(self,f,n,**kw): return self.r[n].get(f,0)
    def write(self,f,n,v,**kw): self.r[n][f]=v; self.writes.append((n,f,v))
    def disable_torque(self,ns,**kw):
        for n in ns:self.write('Torque_Enable',n,0)
    def step(self,dt):
        for n,r in self.r.items():
            if r['Torque_Enable']:
                if n in WHEELS:
                    r['Present_Velocity']=r['Goal_Velocity'];r['Present_Position']+=round(r['Goal_Velocity']*dt)
                else:r['Present_Position']=r['Goal_Position']
            else:r['Present_Velocity']=0

class Rig:
    def __init__(self):
        self.t=10.;self.b=Plant();self.cam=True
        c={n:SimpleNamespace(range_min=100,range_max=4000,homing_offset=0) for n in self.b.motors}
        self.o=HardwareOwner([self.b],c,lambda b,n:dict(b.r[n]),clock=lambda:self.t,wall=lambda:self.t,position_scope=SCOPES['both'],paddle_profile=True,wheels=True,teleop=True,camera_metadata=lambda:{'received_at':self.t if self.cam else self.t-20,'seq':1})
        self.o.inspect();assert not self.b.writes
        self.token='test-session-12345678901234567890';self.seq=0
    def claim(self,scope):
        self.scope=scope;self.o.command(dict(id=1,session_started=self.o.started,op='teleop_claim',scope=scope,token=self.token))
        self.send()
    def send(self, rates=None, left=False,right=False,linear=0,angular=0):
        self.seq+=1
        c=dict(token=self.token,sequence=self.seq,session_started=self.o.started,received=self.t,expires=self.t+INPUT_TTL,rates=rates or {},deadman=dict(left=left,right=right),linear=linear,angular=angular)
        self.o.teleop.accept(c);return c
    def step(self,dt=.05):self.b.step(dt);self.t+=dt;self.o.poll()
    def fault(self,dt=.05):
        try:self.step(dt)
        except RuntimeError as e:self.o.release_all(str(e))
        else:raise AssertionError('Fault was not raised')
        assert not self.o.teleop.active and not self.o.enabled
        assert all(r['Torque_Enable']==0 and (n not in WHEELS or r['Goal_Velocity']==0) for n,r in self.b.r.items())

# Every position servo can move both ways, within its own scope. Initial arm primes measured positions.
for scope in ('both','head'):
    r=Rig();r.claim(scope)
    assert r.o.enabled==set(SCOPES[scope])
    for n in SCOPES[scope]:
        before=r.b.r[n]['Present_Position']
        for direction in (1,-1):
            for _ in range(4):r.send({n:direction*80},left=True,right=True);r.step()
        r.step();assert abs(r.b.r[n]['Present_Position']-before)<=5
    r.o.release_all('test')
# Head remains inaccessible to normal AI enable; its calibration is checked for manual claims.
r=Rig()
try:r.o.enable(SCOPES['head'],True)
except ValueError:pass
else:raise AssertionError('AI head scope expanded')
r.o.state['calibration_mismatches']={'head_motor_1':{}}
try:r.claim('head')
except ValueError:assert not r.b.writes
else:raise AssertionError('Mismatched head allowed')
# Competing AI commands cannot steal manual control, but STOP always releases.
r=Rig();r.claim('left')
for op in ('hold','halt','enable_motors','base_pulse','direct_joint','teleop_claim'):
    try:r.o.command(dict(id=9,session_started=r.o.started,op=op,names=SCOPES['right'],enabled=True,scope='right',token=r.token))
    except ValueError:pass
    else:raise AssertionError('Manual ownership bypassed: '+op)
r.o.command(dict(id=10,session_started=r.o.started,op='stop'));assert not r.o.teleop.active
# Late/duplicate/old-session input cannot rearm or update the lease.
r=Rig();r.claim('right');c=r.send();deadline=r.o.teleop.expires
r.t+=.1;r.o.teleop.accept(c);assert r.o.teleop.expires==deadline
r.fault(.36);r.o.teleop.accept(c);assert not r.o.teleop.active
# Trigger release holds measured position; no catch-up on latency spikes.
r=Rig();r.claim('left');n=SCOPES['left'][0]
r.send({n:100},left=True);r.step();r.send();r.step();assert r.o.goals[n]==r.b.r[n]['Present_Position']
r.fault(.46)
# Bad rate/deadman/NaN/scope never produces a target.
r=Rig();r.claim('left');good=r.send();before=list(r.b.writes)
for extra in ({'rates':{n:float('nan')}},{'rates':{n:101}},{'rates':{n:10}},{'linear':.01},{'rates':{'head_motor_1':10}},{'deadman':{'left':1,'right':False}}):
    c=dict(good,sequence=100,**extra)
    try:r.o.teleop.accept(c)
    except ValueError:pass
    else:raise AssertionError('Malformed input accepted')
assert r.b.writes==before
# Powered startup is forbidden until neutral input.
r=Rig();r.o.command(dict(id=1,session_started=r.o.started,op='teleop_claim',scope='left',token=r.token))
try:r.send({n:10},left=True)
except RuntimeError:pass
else:raise AssertionError('Held trigger survived arm')
# Drive/turn both directions; deadman release brakes, observes rest, restores settings.
for lin,ang in ((.02,0),(-.02,0),(0,.16),(0,-.16)):
    r=Rig();r.claim('drive')
    for _ in range(5):r.send(left=True,right=True,linear=lin,angular=ang);r.step()
    assert all(r.b.r[n]['Torque_Enable']==1 for n in WHEELS)
    for _ in range(12):r.send();r.step()
    assert all(r.b.r[n]['Torque_Enable']==0 and r.b.r[n]['Goal_Velocity']==0 and r.b.r[n]['Operating_Mode']==0 for n in WHEELS)
# Network loss and camera loss zero both wheels even while moving.
for camera in (False,True):
    r=Rig();r.claim('drive');r.send(left=True,right=True,linear=.02);r.step()
    if camera:r.cam=False
    r.fault(.05 if camera else .46)
# Fault between the first register write and torque-on still restores wheel settings.
r=Rig();r.claim('drive');orig=r.o.write
failed=[False]
def fail(n,f,v):
    if n==WHEELS[1] and f=='Operating_Mode' and v==1 and not failed[0]:failed[0]=True;raise RuntimeError('injected partial startup failure')
    orig(n,f,v)
r.o.write=fail;r.send(left=True,right=True,linear=.02);r.fault()
assert all(r.b.r[n]['Operating_Mode']==0 for n in WHEELS)
# API permits expire independently of receipt; replay is rejected before mailbox writes.
with tempfile.TemporaryDirectory() as d:
    class Client:
        folder=Path(d)
        def status(self):return dict(started=7,status_age_s=0,teleop=dict(active=True,token='x',sequence=0,neutral_seen=True))
    t=[10.];a=TeleopAPI(Client(),clock=lambda:t[0]);a.token='x';a.started=7;a.scope='drive';p=a.renew()
    b=dict(token='x',owner_started=7,permit=p['permit'],sequence=1,rates={},deadman=dict(left=False,right=False),linear=0,angular=0)
    a.post('input',b);saved=(Path(d)/'teleop-input.json').read_text()
    try:a.post('input',b)
    except ValueError:pass
    else:raise AssertionError('Permit replay')
    assert (Path(d)/'teleop-input.json').read_text()==saved
    p=a.renew();b.update(permit=p['permit'],sequence=2);t[0]+=.46
    try:a.post('input',b)
    except ValueError:pass
    else:raise AssertionError('Delayed packet accepted')
print('Joy-Con teleop: all 16 fake servos, ownership, neutral gate, stale/replay, disconnect, camera, partial wheel failure and braking passed; no real hardware')
# A failed wheel release remains retryable; it must not discard the abort handle.
r=Rig();r.claim('drive');r.send(left=True,right=True,linear=.02);r.step()
orig=r.o.write
failed=[False]
def fail_release(n,f,v):
    if n==WHEELS[0] and f=='Torque_Enable' and v==0 and not failed[0]:
        failed[0]=True;raise RuntimeError('injected release failure')
    orig(n,f,v)
r.o.teleop.wheel.write=fail_release
r.o.release_all('test partial wheel release')
assert not r.o.state['ok'] and r.o.teleop.wheel is not None
r.o.release_all('retry release')
assert r.o.state['ok'] and r.o.teleop.wheel is None
assert all(r.b.r[n]['Torque_Enable']==0 and r.b.r[n]['Operating_Mode']==0 for n in WHEELS)
print('Manual wheel release failure remains retryable: passed')
