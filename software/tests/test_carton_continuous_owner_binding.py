"""Independent owner binding tests: ephemeral files/readers, no real devices."""
import hashlib
from types import SimpleNamespace

import pytest
from carton.servo.common import Refused,atomic_json,binding,read_json
from carton.servo.continuous_owner_binding import ContinuousOwnerBinding

PAN='right_arm_shoulder_pan'


class Rig:
    def __init__(self,folder):
        self.folder=folder;folder.mkdir();self.now=1000.;self.seq=1
        self.writes=[];self.stops=0;self.guard_fault=False
        self.q={f'right_arm_{n}':2000 for n in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')}
        self.streams={'head':'head-stream','right_wrist':'wrist-stream'}
        self.identities={'head':'head-device','right_wrist':'wrist-device'}
        self.frame_offset={n:0 for n in self.streams};self.frame_seq_offset={n:0 for n in self.streams}
        self.frame_stamp_offset={n:0 for n in self.streams}
        (folder/'calibration.json').write_text('{}')
        self.config={'schema':1,'units':'encoder_ticks','arm':'right','joints':[PAN],
            'ranges':{n:[1000,3000] for n in self.q},'measurements':[],'target':[],
            'calibration_file':str(folder/'calibration.json'),
            'calibration_sha256':hashlib.sha256((folder/'calibration.json').read_bytes()).hexdigest(),
            'session_dir':str(folder),'cameras':{}}
        self.cameras={}
        for name in self.streams:
            reference=folder/f'{name}.png';reference.write_bytes(b'FAKE_REFERENCE_ONLY')
            self.config['cameras'][name]={'camera_id':self.identities[name],'reference':str(reference),
                'manifest':str(folder/f'{name}.json'),'regions':{'anchor':{'anchor':name=='head','roi':[0,0,10,10]}}}
            self.cameras[name]=SimpleNamespace(camera_id=self.identities[name],read=lambda n=name:self.frame(n))
        self.profile={'schema':1,'units':'encoder_ticks','arm':'right','joints':list(self.q),
            'fingerprint':binding(self.config),'commissioning_evidence':'FAKE TEST FIXTURE ONLY',
            'corridor':{n:[1800,2700] for n in self.q},'velocity':{n:100 for n in self.q},
            'acceleration':{n:200 for n in self.q},'following_ticks':24,'start_ticks':5,'settle_ticks':5,
            'vision_age_s':1,'max_tick_gap_s':.2,'settle_timeout_s':1}
        atomic_json(folder/'config.json',self.config);atomic_json(folder/'profile.json',self.profile)
        self.telemetry_times={n:self.now for n in self.q}
    def clock(self):return self.now
    def frame(self,name):
        return SimpleNamespace(camera_id=self.identities[name],stream=self.streams[name],
            seq=self.seq+self.frame_seq_offset[name],stamp=self.now+self.frame_stamp_offset[name])
    def rows(self):
        return {n:{'Present_Position':q,'Present_Temperature':36,'Present_Load':20,'Status':0} for n,q in self.q.items()}
    def guard(self):
        if self.guard_fault:raise Refused('Independent owner thermal/STOP guard')
    def write(self,goals):self.writes.append(goals.copy());self.q.update(goals)
    def stop(self):self.stops+=1
    def create(self):
        return ContinuousOwnerBinding(self.folder/'profile.json',self.folder/'config.json',list(self.q),
            cameras=self.cameras,guard=self.guard,write_goals=self.write,stop=self.stop,clock=self.clock,wall=self.clock)
    def command(self,owner,duration=8):
        return {'id':1,'op':'trajectory','session_started':900,
            'profile_sha256':owner.capabilities()['trajectory_profile_sha256'],'camera_streams':self.streams.copy(),
            'waypoints':[{'time_s':0,'positions':self.q.copy()},
                {'time_s':duration,'positions':{**self.q,PAN:2454}}]}
    def vision(self,command,**updates):
        data={'command_id':command['id'],'session_started':900,'ok':True,'streams':command['camera_streams'],
            'sequences':{n:self.seq for n in self.streams},'captured_at':{n:self.now for n in self.streams}}
        data.update(updates);atomic_json(self.folder/'trajectory-vision.json',data)
    def start(self,owner):
        command=self.command(owner);self.vision(command)
        owner.start(command,self.rows(),self.telemetry_times,session_started=900,lease_remaining=180)
        return command
    def advance(self):
        self.now+=.02;self.seq+=1;self.telemetry_times={n:self.now for n in self.q}


def test_one454tick_continuous_command_uses_canonical_executor_and_metrics(tmp_path):
    rig=Rig(tmp_path/'session');owner=rig.create();command=rig.start(owner)
    assert owner.active
    for _ in range(450):
        rig.advance();rig.vision(command)
        result=owner.tick(rig.rows(),rig.telemetry_times)
        if not owner.active:break
    assert result['completed']==1 and result['phase']=='holding'
    assert rig.q[PAN]==2454 and len(rig.writes)>300 and rig.stops==0
    metrics=owner.metrics()
    assert metrics['whole_trajectory_commands']==1 and metrics['owner_update_count']>300
    assert 49<metrics['observed_loop_hz']<51 and metrics['check_latency_max_s']==0


@pytest.mark.parametrize('field,value',[('velocity',101),('acceleration',201)])
def test_local_binding_rejects_rates_published_validator_does_not_cap(tmp_path,field,value):
    rig=Rig(tmp_path/'session');rig.profile[field][PAN]=value
    atomic_json(rig.folder/'profile.json',rig.profile)
    with pytest.raises(Refused,match='rate'):rig.create()
    assert not rig.writes


def test_actual_calibration_bytes_and_exact_selected_arm_are_independent(tmp_path):
    rig=Rig(tmp_path/'session');(rig.folder/'calibration.json').write_text('{"changed":true}')
    with pytest.raises(Refused,match='calibration'):rig.create()
    rig=Rig(tmp_path/'other')
    with pytest.raises(Refused,match='selected arm'):
        ContinuousOwnerBinding(rig.folder/'profile.json',rig.folder/'config.json',['left_arm_shoulder_pan']*6,
            cameras=rig.cameras,guard=rig.guard,write_goals=rig.write,stop=rig.stop)


def test_local_profile_config_fingerprint_mismatch_refuses(tmp_path):
    rig=Rig(tmp_path/'session');rig.profile['fingerprint']='wrong';atomic_json(rig.folder/'profile.json',rig.profile)
    with pytest.raises(Refused,match='configuration'):rig.create()


def test_independent_camera_reader_registration_not_command_identity_is_authority(tmp_path):
    rig=Rig(tmp_path/'session');rig.cameras['head'].camera_id='different-device'
    with pytest.raises(Refused,match='registration'):rig.create()


def test_path_and_settling_local30second_horizon(tmp_path):
    rig=Rig(tmp_path/'session');owner=rig.create();command=rig.command(owner,29.5);rig.vision(command)
    with pytest.raises(Refused,match='30seconds'):
        owner.start(command,rig.rows(),rig.telemetry_times,session_started=900,lease_remaining=180)
    assert rig.stops==1 and not rig.writes


@pytest.mark.parametrize('fault',[ 'oldest_motor','stream','relabel','future_sequence','bad_command',
                                 'bad_session','stale_camera','guard','stop','thermal',
                                 'following','corridor','status','tracking','rollback','load'])
def test_every_fault_stops_before_another_sdk_callback(tmp_path,fault):
    rig=Rig(tmp_path/'session');owner=rig.create();command=rig.start(owner)
    rig.advance();rig.vision(command)
    telemetry=rig.telemetry_times.copy()
    if fault=='oldest_motor':telemetry[PAN]=rig.now-.21
    if fault=='stream':rig.streams['head']='restarted'
    if fault=='relabel':rig.frame_stamp_offset['head']=-.01
    if fault=='future_sequence':rig.frame_seq_offset['head']=-1
    if fault=='bad_command':rig.vision(command,command_id=2)
    if fault=='bad_session':rig.vision(command,session_started=901)
    if fault=='stale_camera':rig.frame_stamp_offset['head']=-1.1
    if fault=='guard':rig.guard_fault=True
    if fault=='tracking':rig.vision(command,ok=False)
    if fault=='rollback':
        rig.vision(command,sequences={n:0 for n in rig.streams})
    rows=rig.rows()
    if fault=='thermal':rows['right_arm_gripper']['Present_Temperature']=94
    if fault=='following':rows[PAN]['Present_Position']+=25
    if fault=='corridor':rows[PAN]['Present_Position']=1799
    if fault=='status':rows[PAN]['Status']=1
    if fault=='load':rows['right_arm_gripper']['Present_Load']=-500
    with pytest.raises(Refused):owner.tick(rows,telemetry,stop_requested=fault=='stop')
    assert not rig.writes and rig.stops==1 and not owner.active
    assert owner.metrics()['last_error']


def test_guard_rechecked_at_sdk_write_after_successful_feedback_checks(tmp_path):
    rig=Rig(tmp_path/'session');owner=rig.create();command=rig.start(owner)
    rig.advance();rig.vision(command)
    original=owner.guard
    calls=[]
    def guard():
        calls.append(1)
        if len(calls)==2:raise Refused('STOP arrived before SDK callback')
        original()
    owner.guard=guard
    with pytest.raises(Refused,match='STOP arrived'):
        owner.tick(rig.rows(),rig.telemetry_times)
    assert len(calls)==2 and not rig.writes and rig.stops==1 and not owner.active


def test_changed_profile_after_startup_cannot_start_and_stop_is_single_call(tmp_path):
    rig=Rig(tmp_path/'session');owner=rig.create();command=rig.command(owner);rig.vision(command)
    with open(rig.folder/'profile.json','a') as stream:stream.write(' ')
    with pytest.raises(Refused):owner.start(command,rig.rows(),rig.telemetry_times,session_started=900,lease_remaining=180)
    with pytest.raises(Refused):owner.tick(rig.rows(),rig.telemetry_times)
    assert rig.stops==1 and not rig.writes
