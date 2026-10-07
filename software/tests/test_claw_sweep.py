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
