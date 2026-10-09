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

Robot conditions: `--fps` resamples finely recorded demos (record_fold_demos `--sample-dt`) to the policy tick rate
the robot sustains (~4 Hz from the chat Mac); `--camera-lag KEY=SECONDS` renders each camera from the state that
many seconds before the tick (run 5 on 9 Oct: head 0.2 s, wrists 0.4 s), so the policy learns with the delays it
will see; `--visual-jitter SCALE` applies carton/fold_visual_jitter.py per episode. The action stays the commanded
target one policy tick ahead.
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
from carton.refit_camera_contract import trial_contract, render_policy_camera, save_contract, provenance

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


def sample_index(time, t):
    """Index of the latest recorded sample at or before t (clamped into the recording)."""
    return int(np.clip(np.searchsorted(time, t + 1e-6, side='right') - 1, 0, len(time) - 1))


def policy_schedule(time, end, fps, lags, cameras):
    """Policy frames k at t_k = t0 + k/fps up to the recording's `end` sample.

    state: the sample at t_k; action: the commanded target one policy tick ahead (capped at `end`); each camera:
    the sample `lags[key]` seconds before t_k (the robot's frame delay), never before the start.
    With fps == 1/sample_dt and no lags this is the original one-sample-per-frame schedule.
    """
    time = np.asarray(time, float)
    frames = []
    k = 0
    while True:
        t = time[0] + k / fps
        if t > time[end] + 1e-6:
            break
        frames.append(dict(t=float(t), state=sample_index(time, t),
                           action=min(end, sample_index(time, t + 1 / fps)),
                           cameras={key: sample_index(time, t - lags.get(key, 0.)) for key in cameras}))
        k += 1
    if not frames:
        raise ValueError('Empty policy schedule')
    return frames


def parse_lags(values):
    lags = {}
    for item in values or []:
        key, seconds = item.split('=', 1)
        lags[key] = float(seconds)
        if lags[key] < 0:
            raise ValueError(f'Negative camera lag for {key}')
    return lags


def render_trial(job):
    """Render one trial's cameras for a policy schedule (worker process); returns uint8 arrays per camera.

    `job` is (trial, schedule, height, width, cameras, jitter_scale, seed), or the original
    (trial, end, height, width, cameras): one frame per recorded sample up to `end`, no lag, no jitter."""
    if len(job) == 5:
        trial, end, height, width, cameras = job
        schedule = [dict(state=k, action=min(k + 1, end), cameras={key: k for key in cameras}) for k in range(end + 1)]
        jitter_scale, seed = 0., 0
    else:
        trial, schedule, height, width, cameras, jitter_scale, seed = job
    from carton.fold_visual_jitter import episode_rng, jitter_cameras, ImageJitter
    model = mujoco.MjModel.from_xml_path(str(Path(trial) / 'run/scene.xml'))
    data = mujoco.MjData(model)
    contract = trial_contract(trial)
    rng = episode_rng(seed)
    jitter_cameras(model, rng, jitter_scale, cameras=tuple(cameras.values()))
    image_jitter = ImageJitter(rng, jitter_scale)
    renderer = mujoco.Renderer(model, height, width)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # collision hulls have separate CAD visuals
    option.geomgroup[4] = 0  # synthetic teacher-only registration anchors
    try:
        z = np.load(Path(trial) / 'demo.npz')
        qpos = z['qpos']
        n = len(schedule)
        out = {key: np.empty((n, height, width, 3), np.uint8) for key in cameras}
        cache = {}
        for k, frame in enumerate(schedule):
            for key, cam in cameras.items():
                index = frame['cameras'][key]
                if cache.get(key, (None, None))[0] != index:
                    set_render_pose(model, data, qpos[index])
                    cache[key] = (index, image_jitter(render_policy_camera(renderer, data, cam, option, contract)))
                out[key][k] = cache[key][1]
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
    ap.add_argument('--fps', type=int, default=10, help='policy tick rate; demos recorded finer are resampled')
    ap.add_argument('--camera-lag', nargs='*', default=[], metavar='KEY=SECONDS',
                    help='render each camera from this many seconds before the tick (the robot\'s frame delay)')
    ap.add_argument('--visual-jitter', type=float, default=0., help='carton/fold_visual_jitter scale per episode (0: off)')
    args = ap.parse_args(argv)
    cameras = dict(item.split('=', 1) for item in args.cameras)
    lags = parse_lags(args.camera_lag)
    if set(lags) - set(cameras):
        raise SystemExit(f'--camera-lag keys {sorted(set(lags) - set(cameras))} are not policy cameras')
    if args.fps <= 0 or args.visual_jitter < 0:
        raise SystemExit('--fps must be positive and --visual-jitter non-negative')
    if args.out.exists():
        raise SystemExit(f'{args.out} exists; choose a new dataset directory')
    trials = sorted(t for b in args.batches for t in b.glob('trial-*') if (t / 'demo.json').exists())
    used, holdout, skipped, todo = [], [], [], []
    sample_dts, caps = set(), set()
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
        sample_dt = float(demo.get('sample_dt', round(float(np.median(np.diff(z['time']))), 4)))
        if 1 / args.fps < sample_dt - 1e-9:
            raise ValueError(f'{t}: recorded every {sample_dt} s, coarser than the policy tick 1/{args.fps} s')
        sample_dts.add(round(sample_dt, 4))
        caps.add(json.dumps(demo.get('speed_caps_ticks_per_s'), sort_keys=True))
        if demo['seed'] % args.holdout_every == 0:
            holdout.append({'trial': str(t), 'seed': demo['seed'], 'end_index': end,
                            'end_time': float(z['time'][end]), 'sample_dt': sample_dt, 'fps': args.fps,
                            'carton_offset_x': demo['carton_offset_x'],
                            'carton_yaw_degrees': demo['carton_yaw_degrees'],
                            'hinge_stiffness': demo['hinge_stiffness']})
        elif not args.max_episodes or len(todo) < args.max_episodes:
            schedule = policy_schedule(z['time'], end, args.fps, lags, cameras)
            todo.append((t, demo, schedule, [model.jnt_qposadr[model.joint(n).id] for n in ROBOT]))
    if not todo:
        raise ValueError('No training episodes')
    if len(sample_dts) > 1 or len(caps) > 1:
        raise ValueError(f'Demonstrations disagree on sample_dt {sorted(sample_dts)} or speed caps {sorted(caps)}')
    contracts = [trial_contract(t) for t, _, _, _ in todo] + [trial_contract(h['trial']) for h in holdout]
    if any(c != contracts[0] for c in contracts):
        raise ValueError('Demonstrations have different camera contracts')
    contract = contracts[0]
    if contract is not None:
        if [args.width, args.height] != contract['policy']['size_wh']:
            raise ValueError('Dataset dimensions differ from camera contract')
    todo = shard_trials(todo, args.shard_index, args.num_shards)
    if not todo:
        raise ValueError('No training episodes assigned to shard')
    rec = EpisodeRecorder(args.out, f'local/carton_{args.task.replace("-", "_")}_sim', args.fps, list(cameras),
                          (args.height, args.width), ROBOT, robot_type='xlerobot_sim',
                          image_writer_threads=args.image_writer_threads)
    if contract is not None:
        save_contract(args.out, contract)
    try:
        with mp.get_context('spawn').Pool(args.workers) as pool:
            rendered = pool.imap(render_trial, [(str(t), schedule, args.height, args.width, cameras,
                                                    args.visual_jitter, demo['seed'])
                                                   for t, demo, schedule, _ in todo])
            for (t, demo, schedule, adr), images in zip(todo, rendered):
                z = np.load(t / 'demo.npz')
                rec.start_episode(TASK_TEXT[args.task])
                for k, frame in enumerate(schedule):
                    state = dict(zip(ROBOT, z['qpos'][frame['state']][adr]))
                    action = dict(zip(ROBOT, z['ctrl'][frame['action']]))
                    if not rec.tick(state, action, {key: images[key][k] for key in cameras}):
                        raise RuntimeError(f'Failed to write {t} frame {k}')
                n = rec.end_episode(save=True)
                if n != len(schedule):
                    raise RuntimeError(f'Incomplete episode {t}: {n}/{len(schedule)} frames')
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
        {'task': args.task, 'cameras': cameras, 'joints': ROBOT, 'fps': args.fps, 'episodes': used,
         'skipped': skipped, 'simulation_only': True,
         'sample_dt': sorted(sample_dts)[0], 'camera_lag_s': lags, 'visual_jitter': args.visual_jitter,
         'speed_caps_ticks_per_s': json.loads(sorted(caps)[0]),
         'camera_contract': provenance(contract) if contract is not None else None}, indent=1))
    (args.out / 'holdout.json').write_text(json.dumps(holdout, indent=1))
    print(f'{len(used)} episodes, {sum(u["frames"] for u in used)} frames; {len(holdout)} held out')


if __name__ == '__main__':
    main()
