"""Closed-loop evaluation of a trained fold policy in the carton simulation. Simulation only.

Starts from the first recorded state of held-out demonstrations (unseen carton offset/yaw/stiffness, scene
from their own scene.xml). Every 0.1 s the policy gets the rendered `top`/`front` images and the 12 robot
joints, returns 12 joint targets, and the actuators are ramped to them as the scripted controller ramps its
own targets. Physics truth is used only for scoring:

- success: the task's flaps stay >= 80 degrees for 3 s (the demonstrations' own end condition);
- carton translation, robot/flap penetration, and robot contact with anything but the flaps.

`--replay` plays each held-out demonstration's recorded commands instead of a policy; it checks that this
harness reproduces the controller's physics before any policy result is trusted.

    PYTHONPATH=. python tools/eval_fold_policy.py --checkpoint .../checkpoints/last \
        --holdout .../fold-datasets/both-shorts-v1/holdout.json --out .../fold-evals/run-01
"""
from __future__ import annotations

import argparse
import atexit
import json
import math
import time
import weakref
from pathlib import Path

import mujoco
import numpy as np

ROBOT = [f'{s}_{j}' for s in ('left', 'right') for j in
         ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')]
CAMERAS = {'top': 'overhead', 'front': 'front'}   # policy key -> scene camera; replaced from the checkpoint
SCENE_CAMERA = {'top': 'overhead'}                  # keys whose scene camera has a different name
TASK_HINGES = {'both-shorts': ('short_left_hinge', 'short_right_hinge'), 'right-short': ('short_right_hinge',)}
TASK_TEXT = {'both-shorts': 'fold both short carton flaps and hold them',
             'right-short': 'fold the right short carton flap and hold it'}
FOLDED, HOLD, DT = 80., 3., .1
# A mujoco.Renderer finalized during interpreter shutdown segfaults (glDeleteTextures without a context);
# close any still-open renderer before module teardown.
_RENDERERS = weakref.WeakSet()


@atexit.register
def _close_renderers():
    for r in list(_RENDERERS):
        try:
            r.close()
        except Exception:
            pass


def make_renderer(model, height, width):
    r = mujoco.Renderer(model, height, width)
    _RENDERERS.add(r)
    return r


class Episode:
    def __init__(self, trial: Path, height: int, width: int):
        from carton.refit_camera_contract import trial_contract
        self.camera_contract = trial_contract(trial)
        self.model = mujoco.MjModel.from_xml_path(str(trial / 'run/scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.demo = np.load(trial / 'demo.npz')
        m, d = self.model, self.data
        d.qpos[:] = self.demo['qpos'][0]
        d.qvel[:] = self.demo['qvel'][0]
        self.act_ids = [m.actuator(n).id for n in ROBOT]
        d.ctrl[self.act_ids] = self.demo['ctrl'][0]
        d.time = float(self.demo['time'][0])
        mujoco.mj_forward(m, d)
        self.robot_adr = [m.jnt_qposadr[m.joint(n).id] for n in ROBOT]
        self.lo, self.hi = m.actuator_ctrlrange[self.act_ids].T
        self.substeps = int(round(DT / m.opt.timestep))
        self.renderer = make_renderer(m, height, width)
        self.carton0 = d.qpos[12:15].copy()
        names = [m.geom(g).name for g in range(m.ngeom)]
        self.robot_geoms = {g for g, n in enumerate(names) if n.startswith(('left_', 'right_'))}
        self.flap_geoms = {g for g, n in enumerate(names) if 'cardboard' in n}
        self.max_flap_pen = self.max_other_pen = self.max_carton_mm = 0.

    def images(self):
        from carton.refit_camera_contract import render_policy_camera
        out = {}
        for key, cam in CAMERAS.items():
            out[key] = render_policy_camera(self.renderer, self.data, cam, contract=self.camera_contract)
        return out

    def state(self):
        return self.data.qpos[self.robot_adr].astype(np.float32)

    def hinge_degrees(self, name):
        return math.degrees(self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]])

    def step(self, target):
        m, d = self.model, self.data
        target = np.clip(np.asarray(target, float), self.lo, self.hi)
        start = d.ctrl[self.act_ids].copy()
        for i in range(self.substeps):
            t = min(1., (i + 1) / (self.substeps * .8))
            d.ctrl[self.act_ids] = start + (target - start) * (t * t * (3 - 2 * t))
            mujoco.mj_step(m, d)
        for c in d.contact[:d.ncon]:
            g = {int(c.geom1), int(c.geom2)}
            if g & self.robot_geoms:
                other = g - self.robot_geoms
                if other and other <= self.flap_geoms:
                    self.max_flap_pen = max(self.max_flap_pen, -c.dist * 1000)
                elif not (len(g) == 1):
                    self.max_other_pen = max(self.max_other_pen, -c.dist * 1000)
        self.max_carton_mm = max(self.max_carton_mm, float(np.linalg.norm(d.qpos[12:15] - self.carton0) * 1000))
        return np.isfinite(d.qpos).all()


class Video:
    """Presentation MP4 (10 fps, real time): `station` and `front` cameras side by side with a caption."""

    def __init__(self, ep, path, label, height=540, width=960):
        import subprocess
        self.ep, self.label = ep, label
        self.renderer = make_renderer(ep.model, height, width)
        self.proc = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                      '-s', f'{2 * width}x{height}', '-r', str(round(1 / DT)), '-i', '-',
                                      '-pix_fmt', 'yuv420p', '-c:v', 'libx264', '-crf', '20', str(path)],
                                     stdin=subprocess.PIPE)

    def frame(self):
        from PIL import Image, ImageDraw
        views = []
        for cam in ('station', 'front'):
            self.renderer.update_scene(self.ep.data, camera=cam)
            views.append(self.renderer.render())
        im = Image.fromarray(np.concatenate(views, 1))
        d = ImageDraw.Draw(im)
        left, right = (self.ep.hinge_degrees(h) for h in ('short_left_hinge', 'short_right_hinge'))
        d.rectangle((0, 0, im.width, 34), fill=(255, 255, 255))
        d.text((12, 10), f'{self.label} | t = {self.ep.data.time:5.1f} s | short flaps: left {left:5.1f} deg, '
                         f'right {right:5.1f} deg | SIMULATION', fill=(0, 0, 0))
        self.proc.stdin.write(np.asarray(im, np.uint8).tobytes())

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()
        self.renderer.close()


def run(entry, task, policy, max_time, height, width, gif=None, video=None, label='policy'):
    ep = Episode(Path(entry['trial']), height, width)
    if policy is not None:
        policy.reset()
    folded_since, success, frames, k = None, False, [], 0
    vid = Video(ep, video, label) if video else None
    t0 = time.time()
    while ep.data.time < max_time:
        if vid:
            vid.frame()
        imgs = ep.images()
        if gif is not None and k % 3 == 0:
            frames.append(np.concatenate([imgs[k] for k in CAMERAS], 1))
        if policy is None:
            target = ep.demo['ctrl'][min(k + 1, len(ep.demo['ctrl']) - 1)]
        else:
            target = policy.act(ep.state(), imgs, TASK_TEXT[task])
        if not ep.step(target):
            break
        k += 1
        if all(ep.hinge_degrees(h) >= FOLDED for h in TASK_HINGES[task]):
            folded_since = ep.data.time if folded_since is None else folded_since
            if ep.data.time - folded_since >= HOLD:
                success = True
                break
        else:
            folded_since = None
    if vid:
        for _ in range(int(2 / DT)):
            vid.frame()
        vid.close()
    if gif is not None and frames:
        from PIL import Image
        ims = [Image.fromarray(f) for f in frames]
        ims[0].save(gif, save_all=True, append_images=ims[1:] + [ims[-1]] * 10, duration=100, loop=0)
    ep.renderer.close()
    return {'seed': entry['seed'], 'success': success, 'sim_time': round(float(ep.data.time), 2),
            'flaps_degrees': {h: round(ep.hinge_degrees(h), 1) for h in
                              ('short_left_hinge', 'short_right_hinge', 'long_far_hinge', 'long_near_hinge')},
            'max_carton_translation_mm': round(ep.max_carton_mm, 1),
            'max_robot_flap_penetration_mm': round(ep.max_flap_pen, 2),
            'max_robot_other_penetration_mm': round(ep.max_other_pen, 2),
            'carton_offset_x': entry['carton_offset_x'], 'carton_yaw_degrees': entry['carton_yaw_degrees'],
            'hinge_stiffness': entry['hinge_stiffness'], 'wall_s': round(time.time() - t0, 1)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--checkpoint')
    g.add_argument('--replay', action='store_true')
    ap.add_argument('--holdout', type=Path, required=True)
    ap.add_argument('--task', choices=sorted(TASK_HINGES), default='both-shorts')
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--max-time', type=float, default=75.)
    ap.add_argument('--gifs', type=int, default=2, help='render this many episodes to GIF')
    ap.add_argument('--videos', type=int, default=0, help='record this many episodes as presentation MP4s')
    ap.add_argument('--label', default='Learned ACT policy (cameras + joints only)')
    ap.add_argument('--device', default='mps')
    ap.add_argument('--threads', type=int, default=3, help='torch CPU threads (keeps parallel evals from starving training)')
    ap.add_argument('--temporal-ensemble', type=float, metavar='COEFF',
                    help='ACT temporal ensembling (original ACT uses 0.01): re-plan every step and average chunks')
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=False)
    entries = json.loads(args.holdout.read_text())[:args.episodes]
    policy, hw = None, (240, 320)
    if args.checkpoint:
        import torch
        torch.set_num_threads(args.threads)
        from farm.learning.infer import PolicyRunner
        policy = PolicyRunner(args.checkpoint, device=args.device)
        if args.temporal_ensemble is not None:
            from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
            cfg = policy.policy.config
            cfg.temporal_ensemble_coeff, cfg.n_action_steps = args.temporal_ensemble, 1
            policy.policy.temporal_ensembler = ACTTemporalEnsembler(args.temporal_ensemble, cfg.chunk_size)
        spec = policy.input_spec()
        hw = next(iter(spec['cameras'].values()))
        CAMERAS.clear()
        CAMERAS.update({k: SCENE_CAMERA.get(k, k) for k in spec['camera_names']})
    rows = []
    for i, e in enumerate(entries):
        r = run(e, args.task, policy, args.max_time, *hw,
                gif=str(args.out / f'seed{e["seed"]}.gif') if i < args.gifs else None,
                video=str(args.out / f'seed{e["seed"]}.mp4') if i < args.videos else None, label=args.label)
        rows.append(r)
        print(json.dumps(r), flush=True)
    n = len(rows)
    summary = {'checkpoint': args.checkpoint or 'replay', 'task': args.task, 'episodes': n,
               'temporal_ensemble': args.temporal_ensemble,
               'success_rate': sum(r['success'] for r in rows) / max(1, n),
               'max_carton_translation_mm': max(r['max_carton_translation_mm'] for r in rows),
               'max_robot_flap_penetration_mm': max(r['max_robot_flap_penetration_mm'] for r in rows),
               'max_robot_other_penetration_mm': max(r['max_robot_other_penetration_mm'] for r in rows),
               'simulation_only': True}
    (args.out / 'result.json').write_text(json.dumps({'summary': summary, 'episodes': rows}, indent=1))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
