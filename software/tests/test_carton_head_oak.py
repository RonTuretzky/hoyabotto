"""Explicit head/OAK configuration; synthetic protocol fixtures, no camera I/O."""
import copy

import pytest

from carton.servo.common import Refused, Trace, atomic_json, binding, validate_config
from carton.servo.continuous import ContinuousTransport, TrajectoryExecutor
from carton.servo.controller import Experiment
from carton.servo.program import Program, preflight
from test_carton_continuous import ContinuousRig
from test_carton_continuous_owner_binding import Rig
from test_carton_depth import spec


def head_oak_binding(rig):
    old='right_wrist'
    rig.config['camera_pair']='head_oak';rig.profile['camera_pair']='head_oak'
    rig.config['cameras']['oak']=rig.config['cameras'].pop(old)
    rig.identities['oak']=rig.identities.pop(old)
    rig.config['cameras']['oak']['camera_id']=rig.identities['oak']
    rig.streams['oak']=rig.streams.pop(old)
    rig.frame_seq_offset['oak']=rig.frame_seq_offset.pop(old)
    rig.frame_stamp_offset['oak']=rig.frame_stamp_offset.pop(old)
    rig.cameras.pop(old)
    from types import SimpleNamespace
    rig.cameras['oak']=SimpleNamespace(camera_id=rig.identities['oak'],read=lambda:rig.frame('oak'))
    rig.profile['fingerprint']=binding(rig.config)
    atomic_json(rig.folder/'config.json',rig.config);atomic_json(rig.folder/'profile.json',rig.profile)


@pytest.mark.parametrize('fault', [None,'stop','oak_loss','oak_restart','tracking'])
def test_independent_binding_head_oak_has_no_wrist_and_stops_before_write(tmp_path,fault):
    rig=Rig(tmp_path/'owner');head_oak_binding(rig)
    owner=rig.create();command=rig.start(owner)
    assert set(command['camera_streams'])=={'head','oak'}
    rig.advance();rig.vision(command)
    if fault=='oak_loss':rig.frame_stamp_offset['oak']=-2
    if fault=='oak_restart':rig.streams['oak']='new-stream'
    if fault=='tracking':rig.vision(command,ok=False)
    if fault:
        with pytest.raises(Refused):owner.tick(rig.rows(),rig.telemetry_times,stop_requested=fault=='stop')
        assert rig.stops==1 and not rig.writes
    else:
        owner.tick(rig.rows(),rig.telemetry_times)
        assert rig.writes and rig.stops==0


def test_oak_cannot_be_implicitly_relabelled_as_wrist(tmp_path):
    rig=Rig(tmp_path/'owner');head_oak_binding(rig)
    rig.profile.pop('camera_pair');atomic_json(rig.folder/'profile.json',rig.profile)
    with pytest.raises(Refused,match='camera pairs differ'):rig.create()


class HeadOakProgramRig(ContinuousRig):
    def __init__(self,folder,fault=None):
        super().__init__(folder)
        self.depth_fault=fault
        self.config['camera_pair']='head_oak'
        self.config['cameras']['oak']=self.config['cameras'].pop('right_wrist')
        self.config['cameras']['oak']['camera_id']='oak-test'
        self.recipe['depth']=spec(folder)
        self.config['cameras']['oak']['manifest']=self.recipe['depth']['manifest']
        self.profile['camera_pair']='head_oak'
        self.profile['fingerprint']=binding(self.config)
        self.engine=TrajectoryExecutor(self.profile,self.write,self.stop,clock=self.clock,wall=self.clock)
        self.s.update(self.engine.capabilities());self.publish()
    def observe(self,after=0):
        obs=super().observe(after)
        obs.points['oak']=obs.points.pop('right_wrist')
        obs.streams['oak']=obs.streams.pop('right_wrist')
        obs.sequences['oak']=obs.sequences.pop('right_wrist')
        obs.captured_times['oak']=obs.captured_times.pop('right_wrist')
        return obs
    def depth(self,paired_at):
        if self.depth_fault=='depth_loss' and len(self.writes)>8:raise Refused('Depth timestamps are stale')
        if self.depth_fault=='camera_moved' and len(self.writes)>8:raise Refused('Table/camera registration moved')
        x=self.q['right_arm_shoulder_pan']-2000
        paddle=x-self.held_at if self.held_at is not None else self.placed_at
        if self.depth_fault=='dropped' and x>465:paddle-=15
        return dict(seq=self.seq,stream='oak-depth',captured_at=self.now,normal=[1,0,0],
                    points={'tool':[x,0,500],'paddle':[paddle,0,500]},bottom_clearance_mm=paddle-8,
                    robot_frame_calibrated=False)


@pytest.mark.parametrize('fault',[None,'depth_loss','camera_moved','dropped'])
def test_no_wrist_program_requires_metric_retention_and_stops_on_depth_failure(tmp_path,fault):
    from types import SimpleNamespace
    rig=HeadOakProgramRig(tmp_path/'owner',fault)
    assert preflight(rig.recipe,rig.config,None)['problems']==[]
    transport=ContinuousTransport(rig.config,rig.limits,rig.profile,execute=True,clock=rig.clock,sleep=rig.sleep)
    trace=Trace(tmp_path/'evidence')
    def run():
        with transport:
            experiment=Experiment(rig.config,transport,rig,trace,binding(rig.config),rig.clock)
            return Program(rig.recipe,experiment,SimpleNamespace(observe=rig.depth)).run(None)
    try:
        if fault:
            with pytest.raises(Refused):run()
        else:
            assert run()['status']=='PADDLE_PICKUP_CYCLE_PASSED'
    finally:trace.close()
    if fault:
        # Client abort writes STOP atomically; the independent owner consumes
        # it on its next cycle, rather than being called synchronously by client.
        from carton.servo.common import read_json
        assert read_json(rig.folder/'command.json')['op']=='stop'
        rig.sleep(.02)
    assert rig.commands[-1]['op']=='stop' and rig.stops==1
    assert all(set(c['camera_streams'])=={'head','oak'} for c in rig.commands if c['op']=='trajectory')


def test_no_wrist_recipe_refuses_missing_depth_and_wrong_oak_identity(tmp_path):
    rig=HeadOakProgramRig(tmp_path/'owner')
    recipe=copy.deepcopy(rig.recipe);recipe.pop('depth')
    assert any('metric depth' in p for p in preflight(recipe,rig.config,None)['problems'])
    rig.recipe['depth']['camera_id']='different-camera'
    assert any('identities must match' in p for p in preflight(rig.recipe,rig.config,None)['problems'])
    config=copy.deepcopy(rig.config);config.pop('camera_pair')
    with pytest.raises(Refused,match='camera pair'):validate_config(config)


@pytest.mark.parametrize('fault',[None,'missing','hash','stale','units'])
def test_owner_oak_reader_independently_checks_depth_before_feedback(tmp_path,fault):
    from test_carton_depth import stream,publish
    from carton.servo.vision import ManifestDepthCamera
    writer=stream(tmp_path);manifest=publish(writer)
    if fault=='missing':(tmp_path/manifest['depth_image']).unlink()
    if fault=='hash':manifest['depth_sha256']='bad'
    if fault=='stale':manifest['depth_captured_at']-=2
    if fault=='units':manifest['depth_units']='m'
    atomic_json(tmp_path/'oak.json',manifest)
    reader=ManifestDepthCamera(tmp_path/'oak.json','oak-test')
    if fault:
        with pytest.raises((Refused,OSError)):reader.read()
    else:
        frame=reader.read()
        assert frame.camera_id=='oak-test' and frame.manifest['robot_frame_calibrated'] is False


def test_actual_prepare_cli_uses_only_head_and_oak_no_wrist(tmp_path,monkeypatch):
    import json
    from test_carton_depth import stream,publish
    from carton.servo.cli import main
    from carton.servo.common import read_json
    frames=tmp_path/'frames';frames.mkdir()
    oak=tmp_path/'oak';oak.mkdir()
    manifest=publish(stream(oak))
    # Ephemeral coherent RGB head fixture, not a camera capture.
    head=dict(manifest,camera_id='head-test')
    (frames/manifest['image']).write_bytes((oak/manifest['image']).read_bytes())
    atomic_json(frames/'head.json',head)
    calibration=tmp_path/'calibration.json'
    calibration.write_text(json.dumps({f'left_arm_{n}':dict(range_min=1000,range_max=3000)
        for n in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')}))
    out=tmp_path/'experiment'
    assert main(['prepare','--arm','left','--camera-pair','head_oak','--frames',str(frames),
                 '--oak-frames',str(oak),'--calibration',str(calibration),'--session',str(tmp_path/'session'),
                 '--out',str(out)])==0
    config=read_json(out/'experiment.json')
    assert config['camera_pair']=='head_oak' and set(config['cameras'])=={'head','oak'}
    assert config['cameras']['oak']['camera_id']=='oak-test'
    assert not list(out.glob('*wrist*'))
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
    import carton_session
    monkeypatch.setattr(carton_session,'camera_identity',lambda name:{'head':'head-test','oak':'oak-test'}[name])
    readers=carton_session.head_oak_readers(dict(manifests={'head':str(frames/'head.json'),'oak':str(oak/'oak.json')}))
    assert set(readers)=={'head','oak'}
    # This is an immutable fixture, not a live publisher; imports must not age it.
    for reader in readers.values():
        reader.clock=lambda: max(head[k] for k in ('captured_at','rgb_captured_at','depth_captured_at')) + .01
    assert {n:r.read().camera_id for n,r in readers.items()}=={'head':'head-test','oak':'oak-test'}
    with pytest.raises(ValueError,match='manifest paths'):
        carton_session.head_oak_readers(dict(manifests={'head':str(frames/'head.json')}))
