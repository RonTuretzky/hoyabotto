"""Closed-loop evaluation of a trained fold policy in the carton simulation. Simulation only.

Starts from the first recorded state of held-out demonstrations (unseen carton offset/yaw/stiffness, scene
from their own scene.xml). Every 0.1 s the policy gets the rendered `top`/`front` images and the 12 robot
joints, returns 12 joint targets, and the actuators are ramped to them as the scripted controller ramps its
own targets. Physics truth is used only for scoring:

- success: the task's flaps stay >= 80 degrees for 3 s (the demonstrations' own end condition);
- carton translation, robot/flap penetration, and robot contact with anything but the flaps.

`--replay` plays each held-out demonstration's recorded commands instead of a policy; it checks that this
harness reproduces the controller's physics before any policy result is trusted.

Robot conditions (`--dt`, `--max-speed-ticks-s`, `--step-clamp`, `--camera-lag`, `--visual-jitter`): the policy
ticks at the robot's rate, every target is clamped to the owner's per-tick step (40 ticks per arm joint, 10 per
jaw, from the measured position), the actuators move no faster than the servos (run 5 on 9 Oct: 74-91 ticks/s),
each camera image is the one captured that many seconds earlier, and carton/fold_visual_jitter.py perturbs the
cameras per episode. These are the conditions fold_demos_to_lerobot builds the robot-rate dataset for.

    PYTHONPATH=. python tools/eval_fold_policy.py --checkpoint .../checkpoints/last \
        --holdout .../fold-datasets/both-shorts-v1/holdout.json --out .../fold-evals/run-01
"""
from __future__ import annotations

import argparse
import atexit
from collections import deque
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
TICKS_PER_RAD = 4096 / (2 * math.pi)
JAW_INDICES = (5, 11)


def set_dt(dt):
    """Policy tick period for this process (Episode.substeps and the video frame rate read it)."""
    global DT
    if not 0 < dt <= 1:
        raise ValueError('dt must be within (0, 1] s')
    DT = float(dt)


class RobotLimits:
    """What the robot's control path does to a policy target (carton/fold_policy_runner.py, qwen-bridge owner).

    A plain class: fold_policy_fakes loads this module by file path, where a dataclass cannot resolve annotations.
    """
    def __init__(self, max_speed_ticks_s=None, step_clamp=False, arm_step_ticks=40, jaw_step_ticks=10,
                 camera_lag_s=None, visual_jitter=0.):
        self.max_speed_ticks_s = max_speed_ticks_s   # servo speed; jaws get twice this (Goal_Velocity 200 vs 100)
        self.step_clamp = step_clamp                 # |target - measured| <= 40 ticks (arm) / 10 ticks (jaw) per tick
        self.arm_step_ticks, self.jaw_step_ticks = arm_step_ticks, jaw_step_ticks
        self.camera_lag_s = dict(camera_lag_s or {})
        self.visual_jitter = visual_jitter

    def lag_ticks(self, key):
        return int(round(self.camera_lag_s.get(key, 0.) / DT))

    def report(self):
        return dict(dt=DT, max_speed_ticks_s=self.max_speed_ticks_s, step_clamp=self.step_clamp,
                    arm_step_ticks=self.arm_step_ticks, jaw_step_ticks=self.jaw_step_ticks,
                    camera_lag_s=dict(self.camera_lag_s), visual_jitter=self.visual_jitter)


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
    limits = RobotLimits()          # no robot limits unless an instance sets them
    step_rad = None
    speed_rad_step = None
    image_jitter = staticmethod(lambda image: image)
    jitter_applied = {}

    def __init__(self, trial: Path, height: int, width: int, limits: RobotLimits | None = None, seed=None):
        from carton.refit_camera_contract import trial_contract
        from carton.fold_visual_jitter import episode_rng, jitter_cameras, ImageJitter
        self.camera_contract = trial_contract(trial)
        self.model = mujoco.MjModel.from_xml_path(str(trial / 'run/scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.demo = np.load(trial / 'demo.npz')
        self.limits = limits or RobotLimits()
        rng = episode_rng(seed if seed is not None else 0)
        self.jitter_applied = jitter_cameras(self.model, rng, self.limits.visual_jitter)
        self.image_jitter = ImageJitter(rng, self.limits.visual_jitter)
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
        self.step_rad = np.array([(self.limits.jaw_step_ticks if i in JAW_INDICES else self.limits.arm_step_ticks)
                                  / TICKS_PER_RAD for i in range(12)])
        self.speed_rad_step = None
        if self.limits.max_speed_ticks_s:
            self.speed_rad_step = np.array([(2 if i in JAW_INDICES else 1) * self.limits.max_speed_ticks_s
                                            / TICKS_PER_RAD * m.opt.timestep for i in range(12)])
        self.renderer = make_renderer(m, height, width)
        self.carton0 = d.qpos[12:15].copy()
        names = [m.geom(g).name for g in range(m.ngeom)]
        self.robot_geoms = {g for g, n in enumerate(names) if n.startswith(('left_', 'right_'))}
        self.flap_geoms = {g for g, n in enumerate(names) if 'cardboard' in n}
        self.max_flap_pen = self.max_other_pen = self.max_carton_mm = 0.

    def images(self):
        from carton.refit_camera_contract import render_policy_camera
        jitter = getattr(self, 'image_jitter', None)   # absent on bare stand-ins (tests, the fake plant)
        out = {}
        for key, cam in CAMERAS.items():
            image = render_policy_camera(self.renderer, self.data, cam, contract=self.camera_contract)
            out[key] = jitter(image) if jitter is not None else image
        return out

    def state(self):
        return self.data.qpos[self.robot_adr].astype(np.float32)

    def hinge_degrees(self, name):
        return math.degrees(self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]])

    def replay_target(self):
        """The demonstration's commanded target one policy tick after now (its own sample period may be finer)."""
        t = np.asarray(self.demo['time'])
        k = int(np.clip(np.searchsorted(t, float(self.data.time) + DT + 1e-6, side='right') - 1, 0, len(t) - 1))
        return self.demo['ctrl'][k]

    def ramp(self, target):
        """Per-physics-step actuator targets for one policy tick, under the robot limits (none by default)."""
        d = self.data
        target = np.clip(np.asarray(target, float), self.lo, self.hi)
        if self.limits.step_clamp:
            q = d.qpos[self.robot_adr]
            target = np.clip(np.clip(target, q - self.step_rad, q + self.step_rad), self.lo, self.hi)
        start = d.ctrl[self.act_ids].copy()
        previous = start.copy()
        for i in range(self.substeps):
            t = min(1., (i + 1) / (self.substeps * .8))
            ctrl = start + (target - start) * (t * t * (3 - 2 * t))
            if self.speed_rad_step is not None:
                ctrl = previous + np.clip(ctrl - previous, -self.speed_rad_step, self.speed_rad_step)
            previous = ctrl
            yield ctrl

    def step(self, target):
        m, d = self.model, self.data
        for ctrl in self.ramp(target):
            d.ctrl[self.act_ids] = ctrl
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


def run(entry, task, policy, max_time, height, width, gif=None, video=None, label='policy', limits=None):
    limits = limits or RobotLimits()
    ep = Episode(Path(entry['trial']), height, width, limits=limits, seed=entry.get('seed'))
    if policy is not None:
        policy.reset()
    folded_since, success, frames, k = None, False, [], 0
    # The policy sees each camera's frame from `lag` ticks ago (a full buffer's oldest entry); before the buffer
    # fills it sees the first frame, as the robot does right after its cameras start.
    history = {key: deque(maxlen=limits.lag_ticks(key) + 1) for key in CAMERAS}
    vid = Video(ep, video, label) if video else None
    t0 = time.time()
    while ep.data.time < max_time:
        if vid:
            vid.frame()
        imgs = ep.images()
        for key in CAMERAS:
            history[key].append(imgs[key])
        observed = {key: history[key][0] for key in CAMERAS}
        if gif is not None and k % 3 == 0:
            frames.append(np.concatenate([observed[key] for key in CAMERAS], 1))
        if policy is None:
            target = ep.replay_target()
        else:
            target = policy.act(ep.state(), observed, TASK_TEXT[task])
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
        ims[0].save(gif, save_all=True, append_images=ims[1:] + [ims[-1]] * 10, duration=int(300 * DT), loop=0)
    ep.renderer.close()
    return {'seed': entry['seed'], 'success': success, 'sim_time': round(float(ep.data.time), 2),
            'visual_jitter': ep.jitter_applied or None,
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
    ap.add_argument('--dt', type=float, default=.1, help='policy tick period (s); the dataset fps is 1/dt')
    ap.add_argument('--max-speed-ticks-s', type=float, help='servo speed cap on the actuators (robot: ~80-90)')
    ap.add_argument('--step-clamp', action='store_true', help='owner per-tick step: 40 ticks arm, 10 ticks jaw')
    ap.add_argument('--camera-lag', nargs='*', default=[], metavar='KEY=SECONDS', help='frame delay per camera')
    ap.add_argument('--visual-jitter', type=float, default=0., help='carton/fold_visual_jitter scale per episode')
    args = ap.parse_args(argv)
    set_dt(args.dt)
    lags = {}
    for item in args.camera_lag:
        key, seconds = item.split('=', 1)
        lags[key] = float(seconds)
    limits = RobotLimits(max_speed_ticks_s=args.max_speed_ticks_s, step_clamp=args.step_clamp,
                         camera_lag_s=lags, visual_jitter=args.visual_jitter)
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
    if set(lags) - set(CAMERAS):
        raise SystemExit(f'--camera-lag keys {sorted(set(lags) - set(CAMERAS))} are not policy cameras {sorted(CAMERAS)}')
    rows = []
    for i, e in enumerate(entries):
        r = run(e, args.task, policy, args.max_time, *hw,
                gif=str(args.out / f'seed{e["seed"]}.gif') if i < args.gifs else None,
                video=str(args.out / f'seed{e["seed"]}.mp4') if i < args.videos else None, label=args.label,
                limits=limits)
        rows.append(r)
        print(json.dumps(r), flush=True)
    n = len(rows)
    summary = {'checkpoint': args.checkpoint or 'replay', 'task': args.task, 'episodes': n,
               'temporal_ensemble': args.temporal_ensemble,
               'success_rate': sum(r['success'] for r in rows) / max(1, n),
               'max_carton_translation_mm': max(r['max_carton_translation_mm'] for r in rows),
               'max_robot_flap_penetration_mm': max(r['max_robot_flap_penetration_mm'] for r in rows),
               'max_robot_other_penetration_mm': max(r['max_robot_other_penetration_mm'] for r in rows),
               'robot_limits': limits.report(), 'simulation_only': True}
    (args.out / 'result.json').write_text(json.dumps({'summary': summary, 'episodes': rows}, indent=1))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
