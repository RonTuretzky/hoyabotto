"""One-command fold-policy refit (tools/refit_fold_policy.py). Simulation only; never touches the network."""
import copy
import json
import math
import shutil
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest

SOFTWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOFTWARE / 'tools'))
sys.path.insert(0, str(SOFTWARE / 'tests'))

import refit_fold_policy as rf  # noqa: E402
from carton import folding_station_measured as fsm  # noqa: E402

PROFILE = json.loads((SOFTWARE / 'profiles/fold-station-xlerobot-220.json').read_text())
REAL_DEMOS = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/fold-demos')


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    """Any attempt to open an internet connection fails the test."""
    real = socket.socket.connect

    def guarded(self, address):
        if self.family in (socket.AF_INET, socket.AF_INET6):
            raise AssertionError(f'network access attempted: {address}')
        return real(self, address)

    monkeypatch.setattr(socket.socket, 'connect', guarded)
    monkeypatch.setenv('HF_HUB_OFFLINE', '1')


# ---------------------------------------------------------------- fixtures: fake demos and "measurements"

def mini_scene(spacing=.22, base_height=.12, setback=.15):
    """A scene with the station/camera structure of carton.folding_sim.build_scene (bases, grippers, table)."""
    edge, w = -.1515, fsm.words
    base_y = edge - setback
    bases = ''.join(
        f'<body name="{side}_base_link" pos="{w([sx * spacing / 2, base_y, base_height])}">'
        f'<body name="{side}_gripper_link" pos="0 .2 .1"/></body>' for side, sx in (('left', -1), ('right', 1)))
    return f'''<mujoco model="mini"><worldbody>{bases}
<geom name="table" type="box" pos="{w([0, edge + .55, -.016])}" size=".55 .55 .016"/>
<camera name="overhead" pos="0 0 .85" xyaxes="1 0 0 0 1 0" fovy="48"/>
<camera name="front" pos="0 -.3815 .57" xyaxes="1 0 0 0 .7885836665 .6149274761" fovy="48"/>
</worldbody></mujoco>'''


def fake_batch(root, name, seeds, spacing=.22):
    batch = root / name
    for i, seed in enumerate(seeds):
        t = batch / f'trial-{i:03d}'
        (t / 'run').mkdir(parents=True)
        (t / 'run/scene.xml').write_text(mini_scene(spacing))
        (t / 'demo.json').write_text(json.dumps({'seed': seed, 'success': True, 'carton_offset_x': -.01,
                                                 'carton_yaw_degrees': 1., 'hinge_stiffness': .02}))
        (t / 'demo.npz').write_bytes(b'npz')
    (batch / 'batch.json').write_text(json.dumps({'args': {
        'simulation_root': '/sim', 'episodes': str(len(seeds)), 'seed0': str(seeds[0]), 'task': 'both-shorts',
        'hold_after': '3.0', 'max_time': '75.0', 'offset_x': '[-0.025, 0.005]', 'yaw': '[-4.0, 4.0]',
        'stiffness': '[0.012, 0.03]'}}))
    return batch


def lens(fovy_deg=90., width=640, height=480, rms=.4):
    fy = height / 2 / math.tan(math.radians(fovy_deg) / 2)
    return {'fx': fy, 'fy': fy, 'cx': width / 2, 'cy': height / 2, 'width': width, 'height': height,
            'distortion': [0.] * 5, 'rms_reprojection_px': rms, 'photos_used': 18}


def head_pose(fovy_deg=54.):
    """camera_pose_from_tag.py-shaped output built from the model-derived profile (fake 'measurement')."""
    front = PROFILE['cameras']['front']
    fy = 240 / math.tan(math.radians(fovy_deg) / 2)
    entry = {'position_m': front['position_m'], 'rotation_cv': front['rotation_cv'],
             'intrinsics': {'fx': fy, 'fy': fy, 'cx': 320., 'cy': 240., 'width': 640, 'height': 480},
             'sources': 'tools/camera_pose_from_tag.py: front.png, tags [1, 20] 60 mm; reprojection 0.4 px'}
    return {'position_m': entry['position_m'], 'rotation_cv': entry['rotation_cv'], 'tags_used': [1, 20],
            'max_reprojection_px': .4, 'camera_entry': entry}


def make_manifest(tmp_path, *, spacing=.22, wrist_fovy=90., extra=None, demos=None):
    d = tmp_path / 'measurements'
    d.mkdir(exist_ok=True)
    station = copy.deepcopy(PROFILE)
    station['station']['base_spacing_m'] = spacing
    (d / 'station.json').write_text(json.dumps(station))
    (d / 'head-pose.json').write_text(json.dumps(head_pose()))
    (d / 'left_wrist-640x480.json').write_text(json.dumps(lens(wrist_fovy)))
    (d / 'right_wrist-640x480.json').write_text(json.dumps(lens(wrist_fovy)))
    if demos is None:
        demos = {'train_batches': [str(fake_batch(tmp_path / 'demos', 'batch-a', [3000, 3001, 3010]))],
                 'eval_batches': [str(fake_batch(tmp_path / 'demos', 'batch-b', [4000, 4001]))]}
    m = {'schema': rf.MANIFEST_SCHEMA, 'name': 'test-01', 'measured': True, 'station': 'station.json',
         'head_camera': 'head-pose.json',
         'wrist_lenses': {'left': 'left_wrist-640x480.json', 'right': 'right_wrist-640x480.json'},
         'demos': demos, **(extra or {})}
    path = d / 'manifest.json'
    path.write_text(json.dumps(m))
    return path


def refit(manifest, tmp_path, *more):
    args = rf.parse_args([str(manifest), '--run-dir', str(tmp_path / 'run'), *more])
    v = rf.validate(args.manifest, allow_unmeasured=args.allow_unmeasured, output_root=args.output_root)
    r = rf.Refit(v, args)
    r.plan()
    return r


def stage(r, name):
    return next(s for s in r.stages if s.name == name)


# ---------------------------------------------------------------- schema validation

def test_manifest_composes_a_valid_measurement(tmp_path):
    v = rf.validate(make_manifest(tmp_path, wrist_fovy=100.))
    assert v.errors == []
    m = fsm.load_measurement(v.measurement)
    assert set(m['cameras']) == {'front', 'left_wrist', 'right_wrist'}   # no overhead camera
    assert np.allclose(m['cameras']['front']['position_m'], PROFILE['cameras']['front']['position_m'])
    # The wrist lens keeps the model mount and takes its field of view from the calibration.
    left = m['cameras']['left_wrist']
    assert left['frame'] == 'left_gripper_link'
    assert left['position_m'] == PROFILE['cameras']['left_wrist']['position_m']
    assert fsm.camera_pose(left, 'left_wrist')[3] == pytest.approx(100.)
    assert m['station']['base_spacing_m'] == .22
    assert v.mode == 'restage' and v.station_mismatch == {}
    assert v.provenance['front'].startswith('measured') and v.provenance['left_wrist'].startswith('lens measured')
    assert v.warnings == []


def test_missing_parts_fall_back_to_the_model_and_are_reported(tmp_path):
    path = make_manifest(tmp_path)
    m = json.loads(path.read_text())
    del m['wrist_lenses']['right'], m['station']
    path.write_text(json.dumps(m))
    v = rf.validate(path)
    assert v.errors == []
    assert v.measurement['model_derived'] is True
    assert 'right wrist lens' in v.warnings[0] and 'station' in v.warnings[0]
    assert v.provenance['right_wrist'].startswith('model')


def _mutate_file(name, fn):
    def apply(path):
        f = path.parent / name
        data = json.loads(f.read_text())
        fn(data)
        f.write_text(json.dumps(data))
    return apply


def _mutate_manifest(fn):
    return _mutate_file('manifest.json', fn)


@pytest.mark.parametrize('mutate, message', [
    (_mutate_manifest(lambda m: m.update(schema='other/1')), 'schema must be'),
    (_mutate_manifest(lambda m: m.update(name='Bad Name!')), 'name must be'),
    (_mutate_manifest(lambda m: m.update(measured=False)), '--allow-unmeasured'),
    (_mutate_manifest(lambda m: m.update(colour='red')), 'unknown keys'),
    (_mutate_manifest(lambda m: m.update(head_camera='missing.json')), 'file not found'),
    (_mutate_manifest(lambda m: m['wrist_lenses'].update(middle='x.json')), 'keys must be left'),
    (_mutate_file('left_wrist-640x480.json', lambda d: d.update(rms_reprojection_px=3.1)), 'above 2 px'),
    (_mutate_file('left_wrist-640x480.json', lambda d: d.update(photos_used=4)), 'only 4 calibration photos'),
    (_mutate_file('left_wrist-640x480.json', lambda d: d.update(cx=900)), 'principal point'),
    (_mutate_file('left_wrist-640x480.json', lambda d: d.pop('fy')), 'intrinsics need numbers'),
    (_mutate_file('head-pose.json', lambda d: d['camera_entry'].update(frame='left_gripper_link')), 'arm_base frame'),
    (_mutate_file('head-pose.json', lambda d: d['camera_entry'].update(rotation_cv=[[1, 0, 0], [0, 1, 0], [0, 0, 2]])),
     'proper rotation'),
    (_mutate_file('station.json', lambda d: d['station'].pop('base_spacing_m')), 'missing'),
    (_mutate_file('station.json', lambda d: d['station'].update(carton_near_wall_to_table_edge_m=.05)), '10 mm'),
    (_mutate_file('station.json', lambda d: d.update(measured=False, model_derived=False)), 'measured": false'),
])
def test_invalid_manifests_are_refused(tmp_path, mutate, message):
    path = make_manifest(tmp_path)
    mutate(path)
    v = rf.validate(path)
    assert any(message in e for e in v.errors), v.errors


def test_lens_warnings_distortion_crop_and_rms(tmp_path):
    path = make_manifest(tmp_path)
    left = json.loads((path.parent / 'left_wrist-640x480.json').read_text())
    left.update(distortion=[-.3, .1, 0, 0, 0], rms_reprojection_px=1.4)
    (path.parent / 'left_wrist-640x480.json').write_text(json.dumps(left))
    right = lens(90., width=640, height=360)   # 16:9: the policy sees a centred 4:3 crop
    (path.parent / 'right_wrist-640x480.json').write_text(json.dumps(right))
    v = rf.validate(path)
    assert v.errors == []
    text = ' '.join(v.warnings)
    assert 'distortion' in text and 'above 1 px' in text and 'right_wrist: the policy sees a centred 4:3 crop' in text
    assert v.crops['right_wrist']['crop_xyxy'] == [80.0, 0.0, 560.0, 360.0]


def test_unmeasured_manifest_with_model_values_only(tmp_path):
    path = tmp_path / 'm.json'
    batch = fake_batch(tmp_path, 'batch-a', [3000, 3001])
    path.write_text(json.dumps({'schema': rf.MANIFEST_SCHEMA, 'name': 'model', 'measured': False,
                                'demos': {'train_batches': [str(batch)], 'eval_batches': []}}))
    assert any('--allow-unmeasured' in e for e in rf.validate(path).errors)
    v = rf.validate(path, allow_unmeasured=True)
    assert v.errors == [] and v.mode == 'restage'
    assert fsm.load_measurement(v.measurement)['cameras']['front']['fovy_deg'] == 54.


def test_batches_recorded_at_different_stations_are_refused(tmp_path):
    a = fake_batch(tmp_path / 'demos', 'batch-a', [3000])
    b = fake_batch(tmp_path / 'demos', 'batch-b', [4000], spacing=.30)
    v = rf.validate(make_manifest(tmp_path, demos={'train_batches': [str(a)], 'eval_batches': [str(b)]}))
    assert any('different station' in e for e in v.errors)


# ---------------------------------------------------------------- plan (dry run)

def test_dry_run_prints_the_plan_and_writes_nothing(tmp_path, capsys):
    manifest = make_manifest(tmp_path)
    rf.main([str(manifest), '--run-dir', str(tmp_path / 'run')])
    out = capsys.readouterr().out
    assert not (tmp_path / 'run').exists()
    assert 'DRY RUN' in out and 'Nothing was uploaded and no job was launched' in out
    for name in ('validate', 'restage', 'dataset', 'holdout', 'push', 'train', 'wait', 'download', 'eval', 'pick'):
        assert f'\n{name}' in out
    # The proven recipe, on the cloud, only behind --launch.
    for flag in ('--policy.chunk_size=100', '--policy.n_action_steps=100', '--batch_size=32',
                 '--policy.optimizer_lr=3e-05', '--steps=25000', '--job.target=a100-large', '--job.timeout=2h',
                 '--policy.private=true', '--save_checkpoint_to_hub=true'):
        assert flag in out
    assert out.count('needs --launch') >= 4
    assert '--temporal-ensemble 0.01' in out
    assert 'restage_fold_scenes.py' in out and 'fold_demos_to_lerobot.py' in out
    assert '--cameras front=front left_wrist=left_wrist right_wrist=right_wrist' in out
    assert 'record_measured_fold_demos.py' not in out     # station matches: no re-recording
    assert '$' in out and 'cap $5.00' in out
    assert 'hf_' not in out.lower().replace('hf_lerobot_home', '').replace('hf_hub', '')   # no token-like text


def test_plan_json_lists_statuses_costs_and_commands(tmp_path, capsys):
    rf.main([str(make_manifest(tmp_path)), '--run-dir', str(tmp_path / 'run'), '--json'])
    plan = json.loads(capsys.readouterr().out)
    stages = {s['name']: s for s in plan['stages']}
    assert stages['record']['status'] == 'skip'
    assert stages['validate']['status'] == 'todo'
    assert stages['train']['cloud'] and stages['train']['status'] == 'needs --launch'
    assert 2 < stages['train']['usd'] < 3.5
    assert stages['eval']['commands'][0].count('checkpoints/015000/pretrained_model') == 1
    assert not any('last' in c for c in stages['eval']['commands'])
    assert plan['validation']['mode'] == 'restage'


def test_station_change_is_detected_and_needs_rerecord(tmp_path, capsys):
    manifest = make_manifest(tmp_path, spacing=.26)
    r = refit(manifest, tmp_path)
    assert r.v.mode == 'rerecord'
    assert set(r.v.station_mismatch) == {'base_spacing_m'}
    rec = stage(r, 'record')
    assert rec.status == 'blocked' and 're-rendering cannot match' in rec.blocked
    r = refit(manifest, tmp_path, '--allow-rerecord')
    rec = stage(r, 'record')
    assert rec.status == 'todo'
    shown = [c.show() for c in rec.commands]
    assert '--episodes 16' in shown[0] and '--seed0 9000' in shown[0]       # pilot first
    assert '--seed0 3000' in shown[1] and '--seed0 4000' in shown[2]        # same starts as the recorded batches
    assert '--offset-x -0.025 0.005' in shown[1]
    # Restaging then works from the new recordings.
    assert all('demos-recorded' in c.show() for c in stage(r, 'restage').commands)
    with pytest.raises(SystemExit, match='re-rendering cannot match'):
        rf.main([str(manifest), '--run-dir', str(tmp_path / 'run2'), '--run-local'])


def test_eval_steps_must_be_saved_checkpoints(tmp_path):
    with pytest.raises(SystemExit):
        rf.parse_args([str(tmp_path / 'm.json'), '--eval-steps', '12345'])


# ---------------------------------------------------------------- stage skipping and resume

def fake_local_tools(monkeypatch, calls):
    """Replace the subprocess tools by stand-ins that create their outputs."""

    def run_cmd(self, cmd, label, popen=False):
        calls.append(label)
        argv = [str(a) for a in cmd.argv]
        out = Path(argv[argv.index('--out') + 1])
        if 'restage_fold_scenes.py' in argv[1]:
            src = Path(argv[argv.index('--batches') + 1])
            for t in rf.trials(src):
                (out / t.name / 'run').mkdir(parents=True)
                fsm.restage_scene_xml(t / 'run/scene.xml', out / t.name / 'run/scene.xml',
                                      fsm.load_measurement(json.loads(Path(argv[argv.index('--measurement') + 1]).read_text())))
                for n in ('demo.json', 'demo.npz'):
                    (out / t.name / n).symlink_to(t / n)
        elif 'fold_demos_to_lerobot.py' in argv[1]:
            out.mkdir(parents=True)
            (out / 'holdout.json').write_text(json.dumps([{'trial': 'a', 'seed': 3000}]))
            (out / 'conversion.json').write_text(json.dumps({'episodes': [{'frames': 5}], 'skipped': [],
                                                             'cameras': {'front': 'front'}}))
        else:
            raise AssertionError(argv)

    monkeypatch.setattr(rf.Refit, 'run_cmd', run_cmd)
    monkeypatch.setattr(rf, 'holdout_entries', lambda batch, every, task=rf.TASK: [{'trial': str(batch), 'seed': 4000}])


def test_run_local_then_resume_skips_done_stages_and_rebuilds_on_change(tmp_path, monkeypatch, capsys):
    calls = []
    fake_local_tools(monkeypatch, calls)
    manifest = make_manifest(tmp_path)
    run = tmp_path / 'run'
    rf.main([str(manifest), '--run-dir', str(run), '--run-local'])
    assert calls == ['restage-0', 'restage-1', 'dataset-0']
    out = capsys.readouterr().out
    assert 'Local stages finished' in out and 're-run with --launch' in out
    for name in ('validate', 'restage', 'dataset', 'holdout'):
        assert (run / 'stages' / f'{name}.json').exists()
    assert not (run / 'stages/push.json').exists()
    # Restaged scenes carry the measured cameras.
    restaged = (run / 'demos-restaged/batch-a/trial-000/run/scene.xml').read_text()
    assert 'name="left_wrist"' in restaged
    assert len(json.loads((run / 'holdout.json').read_text())) == 2

    # Second run: everything local is done, nothing re-runs.
    calls.clear()
    r = refit(manifest, tmp_path, '--run-local')
    assert [s.status for s in r.stages[:5]] == ['done', 'skip', 'done', 'done', 'done']
    r.execute()
    assert calls == []

    # A changed measurement invalidates validate and everything downstream; outputs are rebuilt.
    head = json.loads((manifest.parent / 'head-pose.json').read_text())
    head['camera_entry']['position_m'][2] += .01
    (manifest.parent / 'head-pose.json').write_text(json.dumps(head))
    r = refit(manifest, tmp_path, '--run-local')
    assert [s.status for s in r.stages[:5]] == ['stale', 'skip', 'stale', 'stale', 'stale']
    r.execute()
    assert calls == ['restage-0', 'restage-1', 'dataset-0']
    m = json.loads((run / 'measurement.json').read_text())
    assert m['cameras']['front']['position_m'][2] == pytest.approx(PROFILE['cameras']['front']['position_m'][2] + .01)

    # A changed conversion parameter rebuilds only the dataset and what follows it.
    calls.clear()
    r = refit(manifest, tmp_path, '--run-local', '--height', '120', '--width', '160')
    assert [s.status for s in r.stages[:5]] == ['done', 'skip', 'done', 'stale', 'stale']
    r.execute()
    assert calls == ['dataset-0']


def test_outputs_outside_the_run_dir_are_never_removed(tmp_path):
    r = refit(make_manifest(tmp_path), tmp_path)
    keep = tmp_path / 'keep'
    keep.mkdir()
    st = rf.Stage('x', 'h', [keep], [], 0.)
    with pytest.raises(SystemExit, match='outside the run directory'):
        r.clear_outputs(st)
    assert keep.exists()
    # A symlinked output is unlinked, its target left alone.
    r.run_dir.mkdir(parents=True)
    link = r.run_dir / 'link'
    link.symlink_to(keep, target_is_directory=True)
    r.clear_outputs(rf.Stage('y', 'h', [link], [], 0.))
    assert not link.exists() and keep.exists()


def fake_checkpoints(root, steps=(15000, 20000, 25000)):
    for s in steps:
        p = root / 'checkpoints' / f'{s:06d}' / 'pretrained_model'
        p.mkdir(parents=True)
        (p / 'model.safetensors').write_bytes(f'weights {s}'.encode())
        (p / 'config.json').write_text(json.dumps({'type': 'act', 'dtype': None, 'chunk_size': 100}))
    return root


def episode(seed, success, carton=3., pen=.1):
    return {'seed': seed, 'success': success, 'max_carton_translation_mm': carton,
            'max_robot_flap_penetration_mm': pen, 'max_robot_other_penetration_mm': 0.,
            'flaps_degrees': {'short_left_hinge': 92. if success else 30., 'short_right_hinge': 91.}}


def test_existing_checkpoints_are_evaluated_ranked_and_installed(tmp_path, monkeypatch, capsys):
    calls = []
    fake_local_tools(monkeypatch, calls)
    ckpts = fake_checkpoints(tmp_path / 'trained')
    # 15k: all succeed; 20k: one failure; 25k ("last"): all succeed but slides the carton more.
    outcome = {15000: (True, 2.), 20000: (False, 1.), 25000: (True, 9.)}
    real_run_cmd = rf.Refit.run_cmd

    class Done:
        returncode = 0

        def poll(self):
            return 0

    def run_cmd(self, cmd, label, popen=False):
        argv = [str(a) for a in cmd.argv]
        if 'eval_fold_policy.py' not in argv[1]:
            return real_run_cmd(self, cmd, label, popen)
        calls.append(label)
        step = int(Path(argv[argv.index('--checkpoint') + 1]).parent.name)
        assert '--temporal-ensemble' in argv and argv[argv.index('--temporal-ensemble') + 1] == '0.01'
        assert json.loads((Path(argv[argv.index('--checkpoint') + 1]) / 'config.json').read_text()).get('dtype', 1) == 1
        hold = json.loads(Path(argv[argv.index('--holdout') + 1]).read_text())
        out = Path(argv[argv.index('--out') + 1])
        out.mkdir(parents=True)
        ok, carton = outcome[step]
        rows = [episode(h['seed'], ok or i, carton) for i, h in enumerate(hold)]
        (out / 'result.json').write_text(json.dumps({'summary': {}, 'episodes': rows}))
        return Done(), open(tmp_path / 'log', 'w')

    monkeypatch.setattr(rf.Refit, 'run_cmd', run_cmd)
    monkeypatch.setattr(rf.time, 'sleep', lambda s: None)
    pilot = tmp_path / 'pilot'
    (pilot / '.private').mkdir(parents=True)
    (pilot / '.private/fold-policy.json').write_text(json.dumps({'schema': 1, 'checkpoint': '/old'}))
    manifest = make_manifest(tmp_path)
    args = [str(manifest), '--run-dir', str(tmp_path / 'run'), '--run-local', '--checkpoints', str(ckpts),
            '--eval-shard-size', '1', '--min-success', '.9', '--pilot', str(pilot)]
    rf.main(args)
    run = tmp_path / 'run'
    report = json.loads((run / 'report.json').read_text())
    assert [s['step'] for s in report['scores']] == [15000, 20000, 25000]
    assert report['chosen']['step'] == 15000 and report['install_gate']['passed']
    chosen = ckpts / 'checkpoints/015000/pretrained_model'
    snippet = json.loads((run / 'fold-policy-checkpoint.json').read_text())
    assert snippet['checkpoint'] == str(chosen.resolve())
    assert snippet['model_sha256'] == rf.sha256_file(chosen / 'model.safetensors')
    assert json.loads((pilot / '.private/fold-policy.json').read_text())['checkpoint'] == str(chosen.resolve())
    assert len(list((pilot / '.private/fold-policy-backups').iterdir())) == 1
    assert 'dtype' not in json.loads((chosen / 'config.json').read_text())
    md = (run / 'report.md').read_text()
    assert '**15000**' in md and 'Needs a human' in md
    n_shards = len(json.loads((run / 'holdout.json').read_text()))
    assert len([c for c in calls if c.startswith('eval-')]) == 3 * n_shards == 6

    # Resume: nothing is restaged, converted or evaluated again.
    calls.clear()
    rf.main(args)
    assert calls == []


def test_pick_ranks_by_score_never_by_last():
    s = lambda step, ok, cmax, cmed=1., pen=.1: {'step': step, 'episodes': 10, 'success': ok, 'success_rate': ok / 10,
                                                 'carton_max_mm': cmax, 'carton_median_mm': cmed, 'flap_pen_max_mm': pen}
    assert rf.pick([s(15000, 9, 5.), s(25000, 8, 1.)])['step'] == 15000
    assert rf.pick([s(25000, 9, 5.), s(20000, 9, 4.)])['step'] == 20000         # tie: less carton slide
    assert rf.pick([s(25000, 9, 5., pen=.1), s(20000, 9, 5., pen=.3)])['step'] == 25000
    assert rf.pick([{**s(0, 9, 1.), 'step': 'last'}]) is None
    assert rf.pick([]) is None


def test_failing_gate_does_not_install(tmp_path):
    r = refit(make_manifest(tmp_path), tmp_path, '--min-success', '.95')
    r.checkpoints = {15000: tmp_path / 'ck'}
    report = rf.build_report(r, [{'step': 15000, 'success': 8, 'episodes': 10, 'success_rate': .8,
                                  'carton_median_mm': 1., 'carton_max_mm': 2., 'flap_pen_max_mm': .1,
                                  'other_pen_max_mm': 0.}], None, False, None)
    assert report['checkpoint'] is None and not report['install_gate']['passed']
    assert 'do not install' in report['needs_a_human'][0]


def test_fix_dtype(tmp_path):
    (tmp_path / 'config.json').write_text(json.dumps({'type': 'act', 'dtype': None}))
    assert rf.fix_dtype(tmp_path) is True
    assert json.loads((tmp_path / 'config.json').read_text()) == {'type': 'act'}
    assert rf.fix_dtype(tmp_path) is False
    (tmp_path / 'config.json').write_text(json.dumps({'dtype': 'float32'}))
    assert rf.fix_dtype(tmp_path) is False


def test_cloud_stages_refuse_without_login(tmp_path, monkeypatch):
    import huggingface_hub
    monkeypatch.setattr(huggingface_hub, 'get_token', lambda: None)
    r = refit(make_manifest(tmp_path), tmp_path)
    with pytest.raises(SystemExit, match='hf auth login'):
        r._hf()


# ---------------------------------------------------------------- smoke: real recorded trials, tiny renders

SMOKE_TRIALS = [REAL_DEMOS / 'batch-220-01/trial-001', REAL_DEMOS / 'batch-220-01/trial-010',
                REAL_DEMOS / 'batch-220-02/trial-000']


@pytest.mark.skipif(not all((t / 'demo.npz').exists() for t in SMOKE_TRIALS), reason='recorded demonstrations absent')
def test_smoke_rerender_two_trials_into_a_tiny_dataset(tmp_path, capsys):
    pytest.importorskip('mujoco')
    pytest.importorskip('lerobot')
    demos = tmp_path / 'demos'
    for t in SMOKE_TRIALS:
        dst = demos / t.parent.name / t.name
        (dst / 'run').mkdir(parents=True)
        for name in ('demo.npz', 'demo.json', 'status.json'):
            if (t / name).exists():
                shutil.copy(t / name, dst / name)
        shutil.copy(t / 'run/scene.xml', dst / 'run/scene.xml')
    manifest = make_manifest(tmp_path, wrist_fovy=100., demos={
        'train_batches': [str(demos / 'batch-220-01')], 'eval_batches': [str(demos / 'batch-220-02')]})
    t0 = time.time()
    rf.main([str(manifest), '--run-dir', str(tmp_path / 'run'), '--run-local', '--height', '48', '--width', '64',
             '--workers', '1'])
    run = tmp_path / 'run'
    conversion = json.loads((run / 'dataset/conversion.json').read_text())
    assert [e['seed'] for e in conversion['episodes']] == [3001]
    assert conversion['cameras'] == {'front': 'front', 'left_wrist': 'left_wrist', 'right_wrist': 'right_wrist'}
    assert sorted(h['seed'] for h in json.loads((run / 'holdout.json').read_text())) == [3010, 4000]
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(run / 'demos-restaged/batch-220-01/trial-001/run/scene.xml'))
    assert model.cam_fovy[model.camera('left_wrist').id] == pytest.approx(100.)
    info = json.loads((run / 'dataset/meta/info.json').read_text())
    assert info['features']['observation.images.front']['shape'][:2] == [48, 64]
    assert time.time() - t0 < 120
