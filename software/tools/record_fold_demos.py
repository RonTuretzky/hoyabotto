"""Record fixed-rate demonstrations of the scripted short-flap folds for imitation learning. Simulation only.

Each trial runs the unchanged vision-in-the-loop controller (`tools/diagnose_short_flap_brace.py`, the
four-flap command of docs/carton-sim-training-handoff-2026-10-08.md) from a frozen source snapshot, in its
own process. A wrapper around `mujoco.mj_step` samples the simulation every 0.1 s of simulated time:
full qpos/qvel and the 12 commanded robot targets (`ctrl`), which the saved folding frames do not store.
The trial stops once the task's flaps have stayed folded for `--hold-after` seconds, so only the short-flap
part of the sequence is simulated.

    PYTHONPATH=. python tools/record_fold_demos.py --simulation-root .../gemma-xlerobot \
        --out .../fold-demos/batch-01 --episodes 40 --workers 4 --task both-shorts

Per trial: `run/` (the controller's own outputs, including scene.xml) and `demo.npz` with
time, qpos, qvel, ctrl, plus `demo.json` (outcome, randomization, stop reason).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np

SOFTWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOFTWARE / 'tools'))
from run_claw_sweep import snapshot_sources  # noqa: E402

ROBOT = [f'{s}_{j}' for s in ('left', 'right') for j in
         ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')]
HINGES = ('short_left_hinge', 'short_right_hinge', 'long_far_hinge', 'long_near_hinge')
TASKS = {'right-short': ('short_right_hinge',), 'both-shorts': ('short_left_hinge', 'short_right_hinge')}
FOLDED_DEGREES = 80.
SAMPLE_DT = .1

# The four-flap command from the 2026-10-08 handoff (hold04-k018), minus per-trial values.
BASE_ARGS = ['--tool', 'claws', '--radius', '.115', '--park-back', '--normal-only', '--floor-marker-x', '.08',
             '--center-floor-marker', '--width', '1280', '--height', '720', '--prepare-near-degrees', '-15.0',
             '--support-height', '0.113', '--fold-right', '--press-left', '--open-claw-transfer',
             '--close-majors-after-open-claw', '--far-hold-degrees', '34.0', '--extra-wall-markers',
             '--clearance', '-0.0045', '--base-height', '0.12', '--far-open-degrees', '1.0']

TRIAL = r'''
import gzip, hashlib, json, math, sys, runpy
import numpy as np, mujoco
from carton.folding_sim import FoldingSimulation
from carton.folding_contact_audit import sample_applied_contacts, score_applied_contacts
cfg = json.loads(sys.argv[1])
class Done(BaseException):
    """Not an Exception, so the controller's own error handling cannot swallow it."""
rec = {'time': [], 'qpos': [], 'qvel': [], 'ctrl': []}
state = {'next': 0.0, 'folded_since': None, 'ids': None, 'reason': 'controller ended'}
audit = {'steps': 0, 'first_refusal': None}
audit_path = cfg['npz'].replace('demo.npz', 'recorder-contact-steps.jsonl.gz')
audit_stream = gzip.open(audit_path, 'wt', encoding='utf-8', compresslevel=1)
audit_end = 0.
original = mujoco.mj_step
def stepped(m, d, *a):
    global audit_end
    started = float(d.time)
    original(m, d, *a)
    row = sample_applied_contacts(m, d, step_started_at=started,
        forbidden_contact=lambda a,b: FoldingSimulation.forbidden_contact(None,a,b))
    audit_stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
    audit_end = float(d.time)
    audit['steps'] += 1
    if row['refusal_reason']:
        audit['first_refusal'] = row
        state['reason'] = row['refusal_reason']
        raise Done()
    if state['ids'] is None:
        state['ids'] = ([m.actuator(n).id for n in cfg['robot']],
                        [m.jnt_qposadr[m.joint(n).id] for n in cfg['task_hinges']])
    if d.time + 1e-9 < state['next']:
        return
    state['next'] += cfg['dt']
    act, hinge = state['ids']
    rec['time'].append(d.time); rec['qpos'].append(d.qpos.copy()); rec['qvel'].append(d.qvel.copy())
    rec['ctrl'].append(d.ctrl[act].copy())
    if all(math.degrees(d.qpos[h]) >= cfg['folded'] for h in hinge):
        state['folded_since'] = state['folded_since'] if state['folded_since'] is not None else d.time
        if d.time - state['folded_since'] >= cfg['hold_after']:
            state['reason'] = 'task flaps folded and held'
    else:
        state['folded_since'] = None
        state['reason'] = 'controller ended'
    if d.time > cfg['max_time']:
        state['reason'] = 'time limit'
        raise Done()
mujoco.mj_step = stepped
original_move = FoldingSimulation.move
def checked_move(self, *args, **kwargs):
    event = original_move(self, *args, **kwargs)
    # Finish the segment so the original penetration, tracking and diagnostic
    # gates also inspect its final step before the recorder accepts a hold.
    if state['reason'] == 'task flaps folded and held':
        failure = event.get('step_error')
        if event.get('bad_penetration_mm', 0) > 1:
            failure = failure or 'Robot collision exceeded 1 mm'
        if event.get('max_target_tracking_error_m', 0) > .035:
            failure = failure or 'Actual fingertips missed target by over 35 mm'
        if failure:
            state['reason'] = failure
        raise Done()
    return event
FoldingSimulation.move = checked_move
sys.argv = [cfg['script']] + cfg['argv']
error = None
try:
    runpy.run_path(cfg['script'], run_name='__main__')
except Done:
    pass
except SystemExit:
    pass
except Exception as e:
    error = repr(e)[:500]
audit_stream.close()
with gzip.open(audit_path, 'rt') as stream:
    audit['independent_score'] = score_applied_contacts((json.loads(line) for line in stream),
        expected_start_time=0., expected_end_time=audit_end,
        forbidden_contact=lambda a,b: FoldingSimulation.forbidden_contact(None,a,b)) if audit_end > 0 else {'passed': False}
with open(audit_path, 'rb') as stream:
    audit['sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
audit['path'] = audit_path
np.savez_compressed(cfg['npz'], **{k: np.array(v) for k, v in rec.items()})
with open(cfg['status'], 'w') as f:
    json.dump({'stop_reason': state['reason'], 'controller_error': error, 'samples': len(rec['time']),
               'sim_time': rec['time'][-1] if rec['time'] else 0.0, 'contact_audit': audit}, f)
# The controller's renderer is only closed by FoldingSimulation.save(), which a stopped trial never reaches;
# finalizing it during interpreter shutdown segfaults in glDeleteTextures. Outputs are written; skip teardown.
import os
sys.stdout.flush()
os._exit(0)
'''


def demo_succeeded(demo):
    audit = demo.get('contact_audit') or {}
    return (demo.get('stop_reason') == 'task flaps folded and held'
            and not demo.get('controller_error') and demo.get('exit_code') == 0
            and audit.get('steps', 0) > 0 and audit.get('first_refusal') is None
            and audit.get('independent_score', {}).get('passed') is True)


def run_trial(python, snapshot, root, trial_dir, seed, offset_x, yaw, stiffness, task, hold_after, max_time):
    trial_dir.mkdir(parents=True)
    argv = ['--simulation-root', str(root), '--out', str(trial_dir / 'run'), '--carton-yaw-degrees', f'{yaw:.4f}',
            '--carton-offset-x', f'{offset_x:.5f}', '--seed', str(seed), '--hinge-stiffness', f'{stiffness:.5f}',
            *BASE_ARGS]
    cfg = {'script': str(snapshot / 'tools/diagnose_short_flap_brace.py'), 'argv': argv, 'robot': ROBOT,
           'task_hinges': list(TASKS[task]), 'dt': SAMPLE_DT, 'folded': FOLDED_DEGREES, 'hold_after': hold_after,
           'max_time': max_time, 'npz': str(trial_dir / 'demo.npz'), 'status': str(trial_dir / 'status.json')}
    started = time.time()
    with open(trial_dir / 'stdout.log', 'w') as log:
        proc = subprocess.run([python, '-B', '-c', TRIAL, json.dumps(cfg)], cwd=snapshot, stdout=log,
                              stderr=subprocess.STDOUT, env={**os.environ, 'PYTHONPATH': str(snapshot)})
    status = json.loads((trial_dir / 'status.json').read_text()) if (trial_dir / 'status.json').exists() else {}
    result_path = trial_dir / 'run/result.json'
    if result_path.exists():
        result = json.loads(result_path.read_text())
        status['controller_error'] = status.get('controller_error') or result.get('error')
    demo = {'seed': seed, 'carton_offset_x': offset_x, 'carton_yaw_degrees': yaw, 'hinge_stiffness': stiffness,
            'task': task, 'exit_code': proc.returncode, 'wall_s': round(time.time() - started, 1), **status}
    demo['success'] = demo_succeeded(demo)
    (trial_dir / 'demo.json').write_text(json.dumps(demo, indent=1))
    return demo


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simulation-root', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--seed0', type=int, default=0)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--task', choices=sorted(TASKS), default='both-shorts')
    ap.add_argument('--hold-after', type=float, default=3.)
    ap.add_argument('--max-time', type=float, default=75.)
    ap.add_argument('--offset-x', type=float, nargs=2, default=(0., 0.), help='carton x offset range (m), uniform')
    ap.add_argument('--yaw', type=float, nargs=2, default=(0., 0.), help='carton yaw range (degrees), uniform')
    ap.add_argument('--stiffness', type=float, nargs=2, default=(.018, .018), help='hinge stiffness range')
    ap.add_argument('--python', default=sys.executable)
    args = ap.parse_args(argv)
    args.simulation_root = args.simulation_root.resolve()
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=False)
    snapshot = args.out / 'source-snapshot'
    snapshot_sources(SOFTWARE, snapshot)
    rng = np.random.default_rng(args.seed0)
    jobs = []
    for i in range(args.episodes):
        seed = args.seed0 + i
        jobs.append((seed, float(rng.uniform(*args.offset_x)), float(rng.uniform(*args.yaw)),
                     float(rng.uniform(*args.stiffness))))
    (args.out / 'batch.json').write_text(json.dumps({'args': {k: str(v) for k, v in vars(args).items()},
                                                     'base_args': BASE_ARGS, 'jobs': jobs}, indent=1))
    results = []
    with ThreadPoolExecutor(args.workers) as pool:
        futures = {pool.submit(run_trial, args.python, snapshot, args.simulation_root, args.out / f'trial-{i:03d}',
                               *job, args.task, args.hold_after, args.max_time): i for i, job in enumerate(jobs)}
        for f in as_completed(futures):
            r = f.result()
            results.append(r)
            print(json.dumps({k: r.get(k) for k in ('seed', 'success', 'stop_reason', 'sim_time', 'wall_s',
                                                     'carton_offset_x', 'carton_yaw_degrees', 'controller_error')}),
                  flush=True)
    ok = sum(r['success'] for r in results)
    (args.out / 'summary.json').write_text(json.dumps({'episodes': len(results), 'successes': ok,
                                                       'results': sorted(results, key=lambda r: r['seed'])}, indent=1))
    print(f'{ok}/{len(results)} demonstrations folded and held the task flaps')


if __name__ == '__main__':
    main()
