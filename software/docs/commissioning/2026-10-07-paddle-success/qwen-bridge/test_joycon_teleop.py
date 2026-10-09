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
    def __init__(self, head=False):
        self.t=10.;self.b=Plant();self.cam=True
        c={n:SimpleNamespace(range_min=100,range_max=4000,homing_offset=0) for n in self.b.motors}
        self.o=HardwareOwner([self.b],c,lambda b,n:dict(b.r[n]),clock=lambda:self.t,wall=lambda:self.t,position_scope=SCOPES['both'],paddle_profile=True,wheels=True,head=head,teleop=True,camera_metadata=lambda:{'received_at':self.t if self.cam else self.t-20,'seq':1})
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
for lin,ang in ((.02,0),(-.02,0),(0,.08),(0,-.08)):
    r=Rig();r.claim('drive')
    for _ in range(5):r.send(left=True,right=True,linear=lin,angular=ang);r.step()
    assert all(r.b.r[n]['Torque_Enable']==1 for n in WHEELS)
    for _ in range(12):r.send();r.step()
    assert all(r.b.r[n]['Torque_Enable']==0 and r.b.r[n]['Goal_Velocity']==0 and r.b.r[n]['Operating_Mode']==0 for n in WHEELS)
# The previous 0.16 rad/s turn exceeds 2 cm/s per wheel at the current 0.45 m track.
r=Rig();r.claim('drive');before=list(r.b.writes)
try:r.send(left=True,right=True,angular=.16)
except ValueError:pass
else:raise AssertionError('Old narrow-track turn exceeded current wheel limits')
assert r.b.writes==before
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

# Today's head and stream capabilities survive the Joy-Con merge. Policy/head
# commands cannot steal an active manual session or release it on refusal.
r=Rig(head=True)
assert r.o.state['head_supported'] and {'head_move','stream'}<=set(r.o.state['capabilities'])
assert set(r.o.state['pickup_required_enabled_motors'])==set(SCOPES['both'])
r.claim('head')
assert all(r.b.r[n]['Torque_Limit']==500 for n in SCOPES['head'])
for op in ('head_move','stream_targets','hold_here'):
    before=list(r.b.writes)
    try:r.o.command(dict(id=11,session_started=r.o.started,op=op,positions={'head_motor_1':2020},duration_s=1))
    except ValueError as e:assert 'Manual control owns' in str(e)
    else:raise AssertionError('Concurrent controller stole manual control: '+op)
    assert r.o.teleop.active and r.o.enabled==set(SCOPES['head']) and r.b.writes==before
r.send({'head_motor_1':60},left=True);r.step();r.step()
assert r.b.r['head_motor_1']['Present_Position']>2000
r.o.command(dict(id=12,session_started=r.o.started,op='stop'))
assert not r.o.teleop.active and not r.o.enabled
assert {'head_move','stream'}<=set(r.o.state['capabilities']) and not (r.o.engine and r.o.engine.active)
assert all(row['Torque_Enable']==0 for row in r.b.r.values())
print('Current head/stream scope, head torque cap, competing-controller refusals and manual STOP: passed')

# Explicit demo profile accelerates only positioning joints; defaults and all
# release/following/watchdog guards remain independent of the selected rate.
r=Rig();n=SCOPES['left'][0];old=dict(r.b.r[n])
r.o.command(dict(id=1,session_started=r.o.started,op='teleop_claim',scope='left',token=r.token,speed_profile='demo'))
r.scope='left';r.send()
assert r.b.r[n]['Goal_Velocity']==300
assert r.b.r['left_arm_gripper']['Goal_Velocity']==200
assert r.o.state['teleop']['position_rate_limits_ticks_s'][n]==300
assert r.o.state['teleop']['position_rate_limits_ticks_s']['head_motor_1']==100
for _ in range(3):r.send({n:300},left=True);r.step()
assert r.o.goals[n]>2030
for bad in ({n:301},{'left_arm_gripper':101}):
 before=list(r.b.writes)
 try:r.send(bad,left=True)
 except ValueError:pass
 else:raise AssertionError('Demo profile exceeded its scoped limit')
 assert r.b.writes==before
r.send();r.step();assert r.o.goals[n]==r.b.r[n]['Present_Position']
r.fault(.46)
assert r.b.r[n]['Goal_Velocity']==old['Goal_Velocity'] and r.b.r[n]['Acceleration']==old['Acceleration']
# Normal re-arm cannot inherit the previous fast profile.
r.claim('left');assert r.b.r[n]['Goal_Velocity']==100
try:r.send({n:300},left=True)
except ValueError:pass
else:raise AssertionError('Demo profile leaked into normal re-arm')
r.o.release_all('test')
# Invalid profile and failed register write must never enable a motor.
for profile in ('turbo',None,{},True):
 r=Rig()
 try:r.o.teleop.claim(dict(scope='left',token=r.token,speed_profile=profile))
 except ValueError:pass
 else:raise AssertionError('Invalid profile accepted')
 assert not r.b.writes
r=Rig();write=r.o.write
r.o.write=lambda n,f,v: None if f=='Goal_Velocity' and v==300 else write(n,f,v)
try:r.o.teleop.claim(dict(scope='left',token=r.token,speed_profile='demo'))
except RuntimeError as e:assert 'readback mismatch' in str(e)
else:raise AssertionError('Missing speed write accepted')
assert not any(f=='Torque_Enable' and v==1 for _,f,v in r.b.writes)
assert not r.o.enabled and not r.o.teleop.active
# Tracking error is still fatal at the demo speed.
r=Rig();r.o.teleop.claim(dict(scope='left',token=r.token,speed_profile='demo'));r.scope='left';r.send({})
r.o.teleop.targets[n]+=81
try:r.o.teleop.tick()
except RuntimeError as e:r.o.release_all(str(e))
else:raise AssertionError('Demo bypassed following error')
assert not r.o.enabled
print('Manual demo speed, scoped caps, normal re-arm, register readback and restoration: passed')

# Pilot/client path shares the selected owner profile without changing contact,
# head, streaming, restoration or normal-session behavior. Fake servos only.
from gemma_direct_client import DirectJointClient
import math
for profile, interval in [('normal', .4), ('demo', 40/300)]:
    r=Rig(head=True);n=SCOPES['right'][1]
    client=DirectJointClient(Path(tempfile.gettempdir())/'unused-speed-test', {})
    cid=[100]
    def deliver(command):
        cid[0]+=1
        r.o.command(dict(command,id=cid[0],session_started=r.o.started))
        return dict(r.o.state)
    client._command=deliver
    client.set_motor_enable(SCOPES['right'],True,speed_profile=profile)
    assert r.o.state['arm_speed_profiles'][n]==profile
    assert r.b.r[n]['Goal_Velocity']==(300 if profile=='demo' else 100)
    assert r.b.r[SCOPES['right'][-1]]['Goal_Velocity']==200
    before=list(r.b.writes)
    try:client.set_motor_enable(SCOPES['right'],True,speed_profile='demo' if profile=='normal' else 'normal')
    except ValueError as e:assert 'Release the arm' in str(e)
    else:raise AssertionError('Pilot changed speed while enabled')
    assert r.b.writes==before
    if profile=='demo':
        try:deliver(dict(op='stream_targets',targets={n:2040}))
        except ValueError as e:assert 'requires normal' in str(e)
        else:raise AssertionError('Demo changed trained policy speed')
        assert r.b.writes==before
    deliver(dict(op='direct_joint',waypoints=[{n:2200},{n:2000}],duration_s=.1))
    assert math.isclose(r.o.engine.interval,interval)
    for _ in range(300):
        r.step()
        if not r.o.engine.active:break
    assert r.o.state['closure_outcome']=='endpoint_settled' and r.b.r[n]['Present_Position']==2000
    assert r.o.state['stop_count']==0
    deliver(dict(op='direct_joint',positions={SCOPES['right'][-1]:1900},duration_s=.1))
    assert r.o.engine.interval==1.5
    deliver(dict(op='halt'))
    client.set_motor_enable(SCOPES['head'],True,speed_profile=profile)
    assert all(r.b.r[h]['Goal_Velocity']==100 for h in SCOPES['head'])
    deliver(dict(op='stop'))
    assert not r.o.arm_speed_profiles and not r.o.enabled
    assert all(row['Goal_Velocity']==0 and row['Acceleration']==0 and row['Torque_Enable']==0 for row in r.b.r.values())
    client.set_motor_enable(SCOPES['right'],True)
    assert r.b.r[n]['Goal_Velocity']==100
    deliver(dict(op='stop'))

# A demo path still halts on loaded contact and fails on tracking error.
for load in (400,0):
    r=Rig();n=SCOPES['left'][1]
    r.o.enable(SCOPES['left'],True,speed_profile='demo')
    r.o.command(dict(id=200,session_started=r.o.started,op='direct_joint',waypoints=[{n:2280},{n:2000}],duration_s=.1))
    r.b.r[n]['Present_Load']=load
    failure=None
    for _ in range(100):
        r.t+=.05
        try:r.o.poll()
        except RuntimeError as e:failure=str(e);r.o.release_all(failure);break
        if not r.o.engine.active:break
    if load:
        assert failure is None and r.o.state['closure_outcome']=='contact_halt', (failure,r.o.state)
        assert r.o.engine.leg==0
        r.o.release_all('test contact')
    else:assert failure and 'following error' in failure
    assert not r.o.enabled and not r.o.arm_speed_profiles
print('Pilot demo: client/owner paths, normal timing, faster waypoints, no hot switching, unchanged gripper/head, stream refusal, contact/tracking and restoration passed')
