"""Turn recorded fold demonstrations (tools/record_fold_demos.py) into a LeRobotDataset for ACT. Simulation only.

Each successful trial's samples (10 Hz) are re-rendered from its own scene.xml: `top` (the simulated
overhead camera) and `front`. State is the 12 robot joints (left six, right six, radians); the action at a
sample is the controller's commanded target at the next sample, so pushes and pinches keep their lead over
the measured joints. Carton pose and flap angles are never written into observations.

Trials whose seed is a multiple of `--holdout-every` are left out and listed in `holdout.json` for
closed-loop evaluation from unseen starts.

    PYTHONPATH=. python tools/fold_demos_to_lerobot.py --batches .../fold-demos/batch-01 \
        --out .../fold-datasets/both-shorts-v1 --task both-shorts

`--cameras` picks the policy cameras, e.g. the robot's three (head + wrists) on a model-derived station
(tools/record_measured_fold_demos.py with profiles/fold-station-xlerobot-220.json):
`--cameras front=front left_wrist=left_wrist right_wrist=right_wrist`.
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import time
from pathlib import Path

import mujoco
import numpy as np

from farm.learning.recorder import EpisodeRecorder

ROBOT = [f'{s}_{j}' for s in ('left', 'right') for j in
         ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')]
CAMERAS = {'top': 'overhead', 'front': 'front'}  # default; --cameras KEY=SCENE_CAMERA ... overrides
TASK_TEXT = {'both-shorts': 'fold both short carton flaps and hold them',
             'right-short': 'fold the right short carton flap and hold it'}
FOLDED, HOLD = 80., 3.


def shard_trials(todo, shard_index, num_shards):
    if num_shards < 1 or not 0 <= shard_index < num_shards:
        raise ValueError('Invalid shard index/count')
    return todo[shard_index::num_shards]


def set_render_pose(model, data, qpos):
    """Update rigid geometry and cameras without recomputing contacts/dynamics.

    Falls back for deformable geometry. This path only renders recorded qpos;
    simulation collection and contact audits still execute full physics.
    """
    data.qpos[:] = qpos
    if model.nflex or model.nskin:
        mujoco.mj_forward(model, data)
    else:
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_camlight(model, data)


def task_end(qpos, time, model, task):
    """Index after which the task flaps have been folded for HOLD seconds (None if never)."""
    names = ('short_right_hinge',) if task == 'right-short' else ('short_left_hinge', 'short_right_hinge')
    adr = [model.jnt_qposadr[model.joint(n).id] for n in names]
    since = None
    for k, q in enumerate(qpos):
        if all(math.degrees(q[a]) >= FOLDED for a in adr):
            since = time[k] if since is None else since
            if time[k] - since >= HOLD:
                return k
        else:
            since = None
    return None


def render_trial(job):
    """Render one trial's cameras up to `end` (worker process); returns uint8 arrays per camera."""
    trial, end, height, width, cameras = job
    model = mujoco.MjModel.from_xml_path(str(Path(trial) / 'run/scene.xml'))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height, width)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # collision hulls have separate CAD visuals
    option.geomgroup[4] = 0  # synthetic teacher-only registration anchors
    try:
        z = np.load(Path(trial) / 'demo.npz')
        out = {key: np.empty((end + 1, height, width, 3), np.uint8) for key in cameras}
        for k in range(end + 1):
            set_render_pose(model, data, z['qpos'][k])
            for key, cam in cameras.items():
                renderer.update_scene(data, camera=cam, scene_option=option)
                out[key][k] = renderer.render()
    finally:
        renderer.close()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--batches', type=Path, nargs='+', required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--task', choices=sorted(TASK_TEXT), default='both-shorts')
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--cameras', nargs='+', default=[f'{k}={v}' for k, v in CAMERAS.items()],
                    help='policy camera KEY=SCENE_CAMERA pairs (default: top=overhead front=front)')
    ap.add_argument('--holdout-every', type=int, default=10)
    ap.add_argument('--max-episodes', type=int)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--shard-index', type=int, default=0)
    ap.add_argument('--num-shards', type=int, default=1)
    ap.add_argument('--image-writer-threads', type=int, default=0)
    args = ap.parse_args(argv)
    cameras = dict(item.split('=', 1) for item in args.cameras)
    if args.out.exists():
        raise SystemExit(f'{args.out} exists; choose a new dataset directory')
    trials = sorted(t for b in args.batches for t in b.glob('trial-*') if (t / 'demo.json').exists())
    rec = EpisodeRecorder(args.out, f'local/carton_{args.task.replace("-", "_")}_sim', 10, list(cameras),
                          (args.height, args.width), ROBOT, robot_type='xlerobot_sim',
                          image_writer_threads=args.image_writer_threads)
    used, holdout, skipped, todo = [], [], [], []
    for t in trials:
        demo = json.loads((t / 'demo.json').read_text())
        if args.task == 'both-shorts' and not demo.get('success'):
            skipped.append(str(t))
            continue
        model = mujoco.MjModel.from_xml_path(str(t / 'run/scene.xml'))
        z = np.load(t / 'demo.npz')
        end = task_end(z['qpos'], z['time'], model, args.task)
        if end is None:
            skipped.append(str(t))
            continue
        if demo['seed'] % args.holdout_every == 0:
            holdout.append({'trial': str(t), 'seed': demo['seed'], 'end_index': end,
                            'carton_offset_x': demo['carton_offset_x'],
                            'carton_yaw_degrees': demo['carton_yaw_degrees'],
                            'hinge_stiffness': demo['hinge_stiffness']})
        elif not args.max_episodes or len(todo) < args.max_episodes:
            todo.append((t, demo, end, [model.jnt_qposadr[model.joint(n).id] for n in ROBOT]))
    todo = shard_trials(todo, args.shard_index, args.num_shards)
    if not todo:
        raise ValueError('No training episodes assigned to shard')
    try:
        with mp.get_context('spawn').Pool(args.workers) as pool:
            rendered = pool.imap(render_trial, [(str(t), end, args.height, args.width, cameras)
                                                   for t, _, end, _ in todo])
            for (t, demo, end, adr), images in zip(todo, rendered):
                z = np.load(t / 'demo.npz')
                rec.start_episode(TASK_TEXT[args.task])
                for k in range(end + 1):
                    state = dict(zip(ROBOT, z['qpos'][k][adr]))
                    action = dict(zip(ROBOT, z['ctrl'][min(k + 1, end)]))
                    if not rec.tick(state, action, {key: images[key][k] for key in cameras}):
                        raise RuntimeError(f'Failed to write {t} frame {k}')
                n = rec.end_episode(save=True)
                if n != end + 1:
                    raise RuntimeError(f'Incomplete episode {t}: {n}/{end + 1} frames')
                used.append({'trial': str(t), 'seed': demo['seed'], 'frames': n})
                print(json.dumps(used[-1]), flush=True)
                progress = args.out/'conversion-progress.json'
                temporary = progress.with_suffix('.tmp')
                temporary.write_text(json.dumps(dict(completed=len(used), total=len(todo),
                    frames=sum(e['frames'] for e in used), shard_index=args.shard_index, updated=time.time())))
                temporary.replace(progress)
    finally:
        rec.close()
    (args.out / 'conversion.json').write_text(json.dumps(
        {'task': args.task, 'cameras': cameras, 'joints': ROBOT, 'fps': 10, 'episodes': used,
         'skipped': skipped, 'simulation_only': True}, indent=1))
    (args.out / 'holdout.json').write_text(json.dumps(holdout, indent=1))
    print(f'{len(used)} episodes, {sum(u["frames"] for u in used)} frames; {len(holdout)} held out')


if __name__ == '__main__':
    main()
