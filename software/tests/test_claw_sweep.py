"""Exercise real worker processes: isolated outputs and failure classification."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys

from tools.run_claw_sweep import run_trial, snapshot_sources


def test_workers_do_not_share_scratch_files_or_count_crashes_as_passes(tmp_path):
    software=tmp_path/'software'
    (software/'tools').mkdir(parents=True)
    runner=software/'tools/diagnose_short_flap_brace.py'
    runner.write_text('''import argparse,json,sys,time
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--out',type=Path);p.add_argument('--seed',type=int)
a,_=p.parse_known_args()
Path('MUJOCO_LOG.TXT').write_text(str(a.seed))
time.sleep(.15)
assert Path('MUJOCO_LOG.TXT').read_text()==str(a.seed)
a.out.mkdir()
(a.out/'result.json').write_text(json.dumps({
 'open_claw_transfer':{'both_shorts_retained_by_right_claw':True}}))
sys.exit(2 if a.seed==1 else 0)
''')
    snapshot=tmp_path/'source-snapshot'
    hashes=snapshot_sources(software,snapshot)
    # Mid-run edits must not affect the snapshot launched by either worker.
    runner.write_text('raise RuntimeError("changed working copy")')
    root=tmp_path/'runs';root.mkdir()
    kwargs=dict(root=root,snapshot=snapshot,simulation_root=tmp_path,
                python=Path(sys.executable),video=False,timeout=10.)
    jobs=[dict(id=f'seed-{i}',seed=i,carton_offset_x=0.,
               prepare_near_degrees=-15.,support_height=.111) for i in (0,1)]
    with ThreadPoolExecutor(max_workers=2) as pool:
        records=list(pool.map(lambda j:run_trial(j,**kwargs),jobs))
    assert hashes['tools/diagnose_short_flap_brace.py']
    assert records[0]['partial_support_passed']
    assert records[1]['exit_code']==2 and not records[1]['partial_support_passed']
    assert not any(r['full_task_complete'] for r in records)
    for i in (0,1):
        assert (root/f'seed-{i}'/'MUJOCO_LOG.TXT').read_text()==str(i)
        saved=json.loads((root/f'seed-{i}'/'worker.json').read_text())
        assert saved['seed']==i and saved['hardware_commands'] is False
    assert not (root/'MUJOCO_LOG.TXT').exists()


def test_major_first_worker_does_not_execute_short_hold_prefix(tmp_path):
    software = tmp_path/'software'
    (software/'tools').mkdir(parents=True)
    (software/'tools/diagnose_short_flap_brace.py').write_text('''import argparse,json,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path)
a,_=p.parse_known_args()
assert '--open-shorts-first' in sys.argv
assert not {'--fold-right','--press-left','--open-claw-transfer'} & set(sys.argv)
a.out.mkdir()
(a.out/'result.json').write_text(json.dumps({
 'short_opening':{'physically_opened_and_released':True},
 'near_major_transfer':{'held_only':True,'full_task_complete':False}}))
''')
    snapshot = tmp_path/'source-snapshot'
    snapshot_sources(software,snapshot)
    root = tmp_path/'runs';root.mkdir()
    record = run_trial(dict(id='major',seed=0,carton_offset_x=0.,prepare_near_degrees=-15.,
        support_height=.111,open_short_angle=-15.,near_pre_out=.03),
        root=root,snapshot=snapshot,simulation_root=tmp_path,python=Path(sys.executable),video=False,timeout=10.)
    assert record['exit_code'] == 0
    assert record['shorts_opened'] and record['near_major_held']
    assert not record['partial_support_passed'] and not record['full_task_complete']


def test_partial_release_is_not_counted_as_a_completed_short_probe(tmp_path):
    software = tmp_path/'software'
    (software/'tools').mkdir(parents=True)
    (software/'tools/diagnose_short_flap_brace.py').write_text('''import argparse,json,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path);p.add_argument('--seed',type=int)
a,_=p.parse_known_args()
assert {'--open-shorts-first','--far-after-near','--release-far-after',
        '--release-near-after-far','--probe-shorts-after-release'} <= set(sys.argv)
assert sys.argv[sys.argv.index('--short-view-camera')+1] == 'front_left_back'
assert '--allow-primary-carton-absence' in sys.argv
assert '--observe-primary-open-shorts' in sys.argv
assert sys.argv[sys.argv.index('--short-contact-policy')+1] == 'setpoint_feedback_v3'
assert sys.argv[sys.argv.index('--short-approach-policy')+1] == 'whole_jaw_normal_v1'
assert not {'--fold-right','--press-left','--open-claw-transfer'} & set(sys.argv)
a.out.mkdir()
(a.out/'result.json').write_text(json.dumps({
 'partial_major_release':{'both_majors_passively_retained':True},
 'partial_short_probe':{'bounded_target_verified':a.seed != 0,
                        'fault':'audit failed' if a.seed==2 else None}}))
''')
    snapshot = tmp_path/'source-snapshot'
    snapshot_sources(software, snapshot)
    root = tmp_path/'runs';root.mkdir()
    for seed, expected in ((0, False), (1, True), (2, False)):
        record = run_trial(dict(id=f'probe-{seed}', seed=seed, carton_offset_x=0.,
            prepare_near_degrees=-15., support_height=.111, open_short_angle=-15.,
            near_pre_out=.03, near_hold_degrees=40., far_after_near=True,
            far_hold_degrees=35., release_far_after=True, release_near_after_far=True,
            probe_shorts_after_release=True, short_view_camera='front_left_back',
            allow_primary_carton_absence=True, short_contact_policy='setpoint_feedback_v3',
            observe_primary_open_shorts=True, short_approach_policy='whole_jaw_normal_v1'), root=root, snapshot=snapshot,
            simulation_root=tmp_path, python=Path(sys.executable), video=False, timeout=10.)
        assert record['exit_code'] == 0
        assert record['both_partial_majors_released']
        assert record['bounded_short_probe_held'] is expected
        assert not record['full_task_complete']
