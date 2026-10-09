"""Automated wrist-camera lens capture through the robot API: point one wrist camera at the tagged carton from about a
dozen planned poses, save a frame at each, then calibrate with tools/calibrate_wrist_from_tags.py.

    # plan only (default): reads the arm's position through the API (read-only) and prints/logs the poses
    python tools/capture_wrist_calibration.py --pilot-root <pilot> --arm right --out runs/wrist-lens-right
    # supervised run: the operator holds STOP; the carton stays in its spot
    python tools/capture_wrist_calibration.py --pilot-root <pilot> --arm right --out runs/wrist-lens-right \
        --execute --operator NAME

Planning (simulation only, no robot): the 220 mm training scene (its carton in the nominal spot, flaps upright, the
robot at its model pose) and its forward kinematics. Random joint configurations of the chosen arm, inside the model
joint limits and the owner's commandable range minus a margin, are screened for: >= 3 box tags in front of the wrist
camera, facing it, inside a narrower-than-assumed field of view (so an unknown lens still frames them) and not hidden
(ray cast); no part of the arm within the clearance of the carton, table, cart or other arm at the pose and along the
straight joint-space path from the start pose (the owner splits long moves proportionally, i.e. along that line). A
greedy max-min pick then spreads the chosen poses over camera position, viewing direction and roll.

Execution (--execute --operator): through the robot's own API only (the pilot's chat_server.Robot, --pilot-root):
robot_get_execution (owner checks), robot_move_joint_targets (one arm, wait=true) to each pose, a settle pause, a
fresh robot_get_cameras frame (captured after the arm settled), robot_move_joint_targets back to the start pose, and
so on. It needs the paddle-success-v1 owner with the arm's six motors already enabled by the operator. It aborts,
leaving the arm holding where the owner left it, on any refusal, unfinished move, owner fault, STOP (stop_count
change), owner restart, motor status bit, a joint not where it was sent, a stale frame or Ctrl-C. It never calls
robot_stop: STOP stays the operator's. The gripper is never commanded.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carton import box_tag_geometry as G  # noqa: E402
from carton.fold_policy_runner import (ARMS, OWNER_JOINTS, OWNER_PROFILE, OWNER_TARGET_MARGIN, SUFFIXES,  # noqa: E402
                                       load_arm_maps, parse_owner_status)
from carton.servo.common import Refused  # noqa: E402

SOFTWARE = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/fold-demos/batch-220-01/trial-020/run/scene.xml')
DEFAULT_MAPS = SOFTWARE / 'profiles/fold-joint-maps'
ARM_JOINTS = SUFFIXES[:5]               # the gripper is never commanded
EXTRA_MARGIN_TICKS = 25                 # on top of the owner's 40-tick target margin
LIMIT_MARGIN_RAD = math.radians(4)      # inside the model joint limits
SETTLED_TICKS = 30                      # |measured - sent| after a completed move (owner tolerance is 57)


def _quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@dataclass
class ViewPose:
    index: int
    targets: dict                   # owner joint name -> encoder tick (five arm joints)
    radians: list                   # model joint angles, same order
    tags: list                      # tag IDs the plan expects in view
    camera_position_m: list         # world frame of the training scene
    camera_distance_m: float        # to the carton's top-centre
    view_elevation_deg: float       # optical axis below horizontal
    min_clearance_m: float          # closest approach along the path (pairs already that close at the start excluded)
    max_travel_ticks: int


@dataclass
class Plan:
    arm: str
    scene: str
    start_ticks: dict               # all twelve arm joints when planned
    poses: list = field(default_factory=list)
    settings: dict = field(default_factory=dict)

    def to_json(self):
        return {'arm': self.arm, 'scene': self.scene, 'start_ticks': self.start_ticks, 'settings': self.settings,
                'poses': [asdict(p) for p in self.poses]}


class ViewPlanner:
    """Forward kinematics, tag visibility and clearance in the training scene. Simulation only."""

    def __init__(self, scene_xml, arm, maps, start_ticks, *, width=640, height=480, plan_fovy=90., window_fovy=75.,
                 min_tag_px=20., clearance=.03, owner_ranges=None):
        import mujoco
        if arm not in ARMS:
            raise Refused(f'arm must be left or right, not {arm!r}')
        self.mj, self.arm, self.maps = mujoco, arm, maps
        self.scene = str(scene_xml)
        spec = mujoco.MjSpec.from_file(self.scene)
        spec.memory = 256 * 2 ** 20          # room for every contact inside the clearance margin
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.width, self.height = width, height
        self.f_plan = height / 2 / math.tan(math.radians(plan_fovy) / 2)
        self.f_window = height / 2 / math.tan(math.radians(window_fovy) / 2)
        self.min_tag_px, self.clearance = min_tag_px, clearance
        self.start_ticks = {n: int(start_ticks[n]) for n in OWNER_JOINTS}
        self.robot_adr = {}
        for a in ARMS:
            for s in SUFFIXES:
                self.robot_adr[f'{a}_arm_{s}'] = m.jnt_qposadr[m.joint(f'{a}_{s}').id]
        self.names = [f'{arm}_arm_{s}' for s in ARM_JOINTS]
        self.adr = np.array([self.robot_adr[n] for n in self.names])
        self.cam = m.camera(f'{arm}_wrist').id
        self.cam_body = m.cam_bodyid[self.cam]
        self.cam_rot = _quat_mat(m.cam_quat[self.cam])
        # scene nominal: carton where the scene puts it, flaps upright (qpos0)
        self.d.qpos[:] = m.qpos0
        self.start_rad = self._rad(self.start_ticks)
        for n, v in self.start_rad.items():
            self.d.qpos[self.robot_adr[n]] = v
        mujoco.mj_kinematics(m, self.d)
        self.corners = G.scene_corners_world(m, self.d)
        r, p = G.scene_carton_pose(m, self.d)
        self.target = p + r @ np.array([0, 0, G.H])
        self.carton_r, self.carton_p = r, p
        self.normals = {i: r @ G.TAGS[i].normal for i in G.TAGS}
        self.ids = np.array(sorted(self.corners))
        self.all_corners = np.array([self.corners[i] for i in self.ids])
        self.all_centres = self.all_corners.mean(1)
        self.all_normals = np.array([self.normals[i] for i in self.ids])
        self.tag_bodies = {i: m.body(G.TAGS[i].scene_body).id for i in G.TAGS}
        # joint bounds (radians): model limits minus a margin, intersected with the commandable ticks
        self.bounds = []
        amap = maps[arm]
        for s, n, a in zip(ARM_JOINTS, self.names, self.adr):
            lo, hi = m.jnt_range[m.joint(f'{arm}_{s}').id]
            rlo, rhi = (owner_ranges or {}).get(n, amap.ranges()[n])
            tick_lo, tick_hi = rlo + OWNER_TARGET_MARGIN + EXTRA_MARGIN_TICKS, rhi - OWNER_TARGET_MARGIN - EXTRA_MARGIN_TICKS
            u = amap.units[s]
            ends = sorted(math.radians(u.model_sign * (t - u.model_zero_tick) * 360 / 4096) for t in (tick_lo, tick_hi))
            self.bounds.append((max(lo + LIMIT_MARGIN_RAD, ends[0]), min(hi - LIMIT_MARGIN_RAD, ends[1])))
        self.bounds = np.array(self.bounds)
        if (self.bounds[:, 0] >= self.bounds[:, 1]).any():
            raise Refused(f'No commandable joint interval for the {arm} arm: {self.bounds.tolist()}')
        # collision bookkeeping
        arm_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith(arm + '_') and b != m.body(f'{arm}_base_link').id}
        self.moving = np.array([m.geom_bodyid[g] in arm_bodies and (m.geom_contype[g] or m.geom_conaffinity[g])
                                for g in range(m.ngeom)])
        self.margin0 = m.geom_margin.copy()

    # ----------------------------------------------------------------- conversions
    def _rad(self, ticks):
        out = {}
        for a in ARMS:
            vals = self.maps[a].ticks_to_rad({n: int(ticks[n]) for n in OWNER_JOINTS if n.startswith(a + '_')})
            out.update({f'{a}_arm_{s}': v for s, v in zip(SUFFIXES, vals)})
        return out

    def ticks(self, q):
        u = self.maps[self.arm].units
        return {n: int(round(u[s].model_zero_tick + u[s].model_sign * math.degrees(float(v)) * 4096 / 360))
                for s, n, v in zip(ARM_JOINTS, self.names, q)}

    # ----------------------------------------------------------------- kinematics and visibility
    def set_arm(self, q):
        self.d.qpos[self.adr] = q
        self.mj.mj_kinematics(self.m, self.d)

    def camera(self):
        b = self.cam_body
        rb = self.d.xmat[b].reshape(3, 3)
        pos = self.d.xpos[b] + rb @ self.m.cam_pos[self.cam]
        r_cv = rb @ self.cam_rot @ np.diag([1., -1., -1.])        # columns: x right, y down, z forward
        return r_cv, pos

    def screen(self, r_cv, pos):
        """Tags passing the cheap checks: in front, facing, inside the window, big enough (list of IDs)."""
        cam = (self.all_corners - pos) @ r_cv                                 # (12, 4, 3)
        z = cam[..., 2]
        front = (z > .08).all(1)
        view = pos - self.all_centres
        facing = np.einsum('ij,ij->i', self.all_normals, view) > .35 * np.linalg.norm(view, axis=1)
        xy = cam[..., :2] / np.where(z > .08, z, 1.)[..., None]
        win = xy * self.f_window + [self.width / 2, self.height / 2]
        inside = ((win >= 10).all(-1) & (win[..., 0] <= self.width - 10) & (win[..., 1] <= self.height - 10)).all(1)
        uv = xy * self.f_plan
        big = np.linalg.norm(uv - np.roll(uv, 1, 1), axis=-1).min(1) >= self.min_tag_px
        return [int(i) for i in self.ids[front & facing & inside & big]]

    def unoccluded(self, ids, pos):
        geomid = np.zeros(1, np.int32)
        ok = []
        for i in ids:
            centre = self.corners[i].mean(0)
            clear = True
            for c in self.corners[i]:
                p = c + (centre - c) * .08             # just inside the black square
                vec = p - pos
                x = self.mj.mj_ray(self.m, self.d, pos, vec, None, 1, -1, geomid)
                if x >= 0 and x < 1 - .006 / np.linalg.norm(vec) and self.m.geom_bodyid[geomid[0]] != self.tag_bodies[i]:
                    clear = False
                    break
            if clear:
                ok.append(i)
        return ok

    # ----------------------------------------------------------------- clearance
    def _close_pairs(self, q):
        """{(moving geom, other geom): distance} for every pair closer than the clearance at arm pose q."""
        m, d = self.m, self.d
        self.d.qpos[self.adr] = q
        m.geom_margin[:] = np.where(self.moving, self.clearance, self.margin0)
        try:
            self.mj.mj_forward(m, d)
        finally:
            m.geom_margin[:] = self.margin0
        out = {}
        for c in d.contact[:d.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if self.moving[g1] == self.moving[g2]:
                if self.moving[g1] and c.dist < -.002:       # the arm into itself (non-adjacent links)
                    out[(g1, g2)] = float(c.dist)
                continue
            key = (g1, g2) if self.moving[g1] else (g2, g1)
            out[key] = min(out.get(key, 1.), float(c.dist))
        return out

    def path_clearance(self, q_goal, max_step_rad=math.radians(3)):
        """Closest approach (m) on the straight joint path from the start (sampled every <= 3 deg of the largest
        joint travel); None if it violates the clearance."""
        q0 = np.array([self.start_rad[n] for n in self.names])
        steps = max(8, int(math.ceil(np.abs(q_goal - q0).max() / max_step_rad)))
        start = self._close_pairs(q0)
        closest = min([self.clearance, *start.values()])      # reported: pairs close at the start included
        for k in range(1, steps + 1):
            for key, dist in self._close_pairs(q0 + (q_goal - q0) * k / steps).items():
                closest = min(closest, dist)
                if key in start and dist >= start[key] - .002:
                    continue
                if dist < self.clearance:
                    self.set_arm(q_goal)
                    return None
        self.set_arm(q_goal)
        return closest

    # ----------------------------------------------------------------- planning
    def aim(self, q, look):
        """Turn shoulder pan and wrist flex (inside their bounds) so the optical axis passes through `look`."""
        from scipy.optimize import least_squares
        q = np.array(q, float)
        idx = [0, 3]

        def residual(x):
            q[idx] = x
            self.set_arm(q)
            r_cv, pos = self.camera()
            want = (look - pos) / np.linalg.norm(look - pos)
            return np.cross(r_cv[:, 2], want) + (r_cv[:, 2] @ want < 0) * 2.0
        fit = least_squares(residual, q[idx], bounds=(self.bounds[idx, 0], self.bounds[idx, 1]), xtol=1e-6, max_nfev=40)
        q[idx] = fit.x
        self.set_arm(q)
        r_cv, pos = self.camera()
        want = (look - pos) / np.linalg.norm(look - pos)
        return q if r_cv[:, 2] @ want > math.cos(math.radians(4)) else None

    def candidates(self, samples, rng, min_tags=3):
        """Random lift/elbow/roll (and starting pan/flex); pan and flex then aim the camera near a random tag (the
        robot-facing ones; ID 13 faces away); kept if the cheap visibility screen sees >= min_tags tags."""
        lo, hi = self.bounds.T
        aims = [self.corners[i].mean(0) for i in self.ids if i != 13]
        out = []
        for _ in range(samples):
            look = aims[rng.integers(len(aims))] + rng.uniform(-.06, .06, 3)
            q = self.aim(rng.uniform(lo, hi), look)
            if q is None:
                continue
            r_cv, pos = self.camera()
            distance = float(np.linalg.norm(pos - self.target))
            if not .15 <= distance <= .65:
                continue
            seen = self.screen(r_cv, pos)
            if len(seen) < min_tags:
                continue
            out.append((q, r_cv, pos, seen))
        return out

    def plan(self, n_poses=12, *, samples=1500, seed=0, min_tags=2) -> Plan:
        rng = np.random.default_rng(seed)
        cands = self.candidates(samples, rng, min_tags)
        feats = []
        for q, r_cv, pos, seen in cands:
            fwd = r_cv[:, 2]
            roll = math.atan2(r_cv[2, 0], -r_cv[2, 1])               # image x axis vs world up
            cam = (self.all_corners[np.isin(self.ids, seen)] - pos) @ r_cv
            centroid = (cam[..., :2] / cam[..., 2:]).reshape(-1, 2).mean(0)     # where the tags fall in the image
            feats.append(np.r_[(pos - self.target) * 4, fwd, .5 * math.cos(roll), .5 * math.sin(roll), centroid])
        feats = np.array(feats) if feats else np.zeros((0, 10))
        chosen, rejected = [], 0
        alive = np.ones(len(cands), bool)
        mind = np.full(len(cands), np.inf)
        q_start = np.array([self.start_rad[n] for n in self.names])
        while len(chosen) < n_poses and alive.any():
            score = np.where(alive, np.minimum(mind, 1e3) * (1 + .25 * np.array([len(c[3]) - 2 for c in cands])), -1)
            k = int(np.argmax(score))
            alive[k] = False
            q, r_cv, pos, seen = cands[k]
            self.set_arm(q)
            visible = self.unoccluded(seen, pos)
            if len(visible) < min_tags:
                rejected += 1
                continue
            clear = self.path_clearance(q)
            if clear is None:
                rejected += 1
                continue
            mind = np.minimum(mind, np.linalg.norm(feats - feats[k], axis=1))
            targets = self.ticks(q)
            fwd = r_cv[:, 2]
            chosen.append(ViewPose(
                len(chosen), targets, [float(v) for v in q], [int(i) for i in visible],
                [round(float(v), 4) for v in pos], round(float(np.linalg.norm(pos - self.target)), 3),
                round(math.degrees(math.asin(max(-1., min(1., -fwd[2])))), 1), round(float(clear), 4),
                int(max(abs(targets[n] - self.start_ticks[n]) for n in self.names))))
        plan = Plan(self.arm, self.scene, dict(self.start_ticks), chosen,
                    {'samples': samples, 'seed': seed, 'candidates': len(cands), 'rejected_after_screen': rejected,
                     'clearance_m': self.clearance, 'min_tags': min_tags,
                     'joint_bounds_rad': self.bounds.round(4).tolist()})
        return plan


# ------------------------------------------------------------------------------------------------ execution
class Aborted(RuntimeError):
    pass


def _ok(payload, tool):
    if not isinstance(payload, dict) or payload.get('ok') is not True or not isinstance(payload.get('result'), dict):
        raise Aborted(f'{tool} refused: {str(payload)[:600]}')
    return payload['result']


class Capture:
    """Runs a plan through the robot API (call(name, args) -> {'ok', 'result', 'images'})."""

    def __init__(self, robot, plan: Plan, out: Path, *, duration_s=6.0, settle_s=1.0, clock=time.time,
                 sleep=time.sleep, stop_requested=lambda: False, log=print):
        self.robot, self.plan, self.out = robot, plan, Path(out)
        self.duration_s, self.settle_s = duration_s, settle_s
        self.clock, self.sleep, self.stop_requested, self.log = clock, sleep, stop_requested, log
        self.arm = plan.arm
        self.names = [f'{self.arm}_arm_{s}' for s in ARM_JOINTS]
        self.camera = f'{self.arm}_wrist'
        self.initial = None
        self.calls = []

    def call(self, name, args):
        if name == 'robot_stop':             # never: STOP belongs to the operator
            raise AssertionError('capture never calls robot_stop')
        self.calls.append(name)
        return self.robot.call(name, args)

    def snapshot(self):
        sent = self.clock()
        result = _ok(self.call('robot_get_execution', {}), 'robot_get_execution')
        try:
            return parse_owner_status(result, local_sent=sent, local_received=self.clock())
        except Refused as exc:
            raise Aborted(str(exc)) from exc

    def check(self, snap, where):
        if not snap.ok or snap.release_errors:
            raise Aborted(f'{where}: owner not healthy: {snap.release_errors or snap.last_stop}')
        if self.initial is not None:
            if snap.owner_started != self.initial.owner_started:
                raise Aborted(f'{where}: owner restarted')
            if snap.stop_count != self.initial.stop_count:
                raise Aborted(f'{where}: owner STOP or fault (stop_count {self.initial.stop_count} -> '
                              f'{snap.stop_count}): {snap.last_stop}')
        bad = {n: snap.status[n] for n in OWNER_JOINTS if snap.status[n] != 0}
        if bad:
            raise Aborted(f'{where}: motor status bits {bad}')
        off = [n for n in (f'{self.arm}_arm_{s}' for s in SUFFIXES) if n not in snap.enabled or snap.torque[n] != 1]
        if off:
            raise Aborted(f'{where}: {self.arm} arm motors not enabled/holding: {off}')
        if self.stop_requested():
            raise Aborted(f'{where}: stop requested')

    def preflight(self):
        snap = self.snapshot()
        if snap.profile != OWNER_PROFILE:
            raise Aborted(f'Execution needs the {OWNER_PROFILE} owner; this owner is {snap.profile}')
        if snap.phase not in ('idle', 'holding'):
            raise Aborted(f'Owner is {snap.phase}; start from a stationary hold')
        self.check(snap, 'preflight')
        drift = {n: snap.ticks[n] - self.plan.start_ticks[n] for n in OWNER_JOINTS
                 if abs(snap.ticks[n] - self.plan.start_ticks[n]) > SETTLED_TICKS}
        if drift:
            raise Aborted(f'The robot is not where the plan started (re-plan): {drift}')
        for n in self.names:
            if n not in snap.ranges:
                raise Aborted(f'The owner reports no commandable range for {n}')
            lo, hi = snap.ranges[n]
            for p in self.plan.poses:
                if not lo + OWNER_TARGET_MARGIN <= p.targets[n] <= hi - OWNER_TARGET_MARGIN:
                    raise Aborted(f'pose {p.index}: {n}={p.targets[n]} outside the commandable range')
        self.initial = snap
        return snap

    def move(self, targets, where):
        self.check(self.snapshot(), where + ' (before)')
        result = _ok(self.call('robot_move_joint_targets', {'arm': self.arm, 'positions': dict(targets),
                                                             'duration_s': self.duration_s, 'wait': True}),
                     'robot_move_joint_targets')
        if result.get('accepted') is not True and result.get('no_op') is not True:
            raise Aborted(f'{where}: move not accepted: {str(result)[:400]}')
        if result.get('completed') is not True:
            raise Aborted(f'{where}: move did not complete ({result.get("closure_outcome")}); the arm holds where '
                          f'the owner stopped it: {str(result)[:400]}')
        snap = self.snapshot()
        self.check(snap, where + ' (after)')
        off = {n: [snap.ticks[n], t] for n, t in targets.items() if abs(snap.ticks[n] - t) > SETTLED_TICKS}
        if off:
            raise Aborted(f'{where}: joints not where they were sent [measured, sent]: {off}')
        return snap

    def frame(self, settled_robot_time):
        import cv2
        payload = self.call('robot_get_cameras', {'cameras': [self.camera], 'revive': False})
        result = _ok(payload, 'robot_get_cameras')
        if result.get('camera_errors'):
            raise Aborted(f'{self.camera} not fresh: {result["camera_errors"]}')
        images = payload.get('images') or []
        if len(images) != 1:
            raise Aborted(f'expected one {self.camera} image, got {len(images)}')
        image = images[0]
        data = base64.b64decode(image['data_base64'])
        if hashlib.sha256(data).hexdigest() != image.get('sha256'):
            raise Aborted(f'{self.camera}: image hash mismatch')
        stamp = image.get('captured_at')
        if type(stamp) not in (int, float) or stamp < settled_robot_time:
            raise Aborted(f'{self.camera}: frame captured at {stamp}, before the arm settled at {settled_robot_time}')
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise Aborted(f'{self.camera}: undecodable image')
        return bgr, image

    def run(self):
        import cv2
        frames_dir = self.out / 'frames'
        frames_dir.mkdir(parents=True, exist_ok=True)
        start = {n: self.plan.start_ticks[n] for n in self.names}
        records, aborted = [], None
        try:
            self.preflight()
            for pose in self.plan.poses:
                where = f'pose {pose.index}'
                self.log(f'{where}: moving the {self.arm} arm to {pose.targets} (expects tags {pose.tags})')
                snap = self.move(pose.targets, where)
                self.sleep(self.settle_s)
                settled = self.snapshot()
                self.check(settled, where + ' (settled)')
                bgr, meta = self.frame(settled.robot_now)
                after = self.snapshot()
                self.check(after, where + ' (after frame)')
                moved = {n: after.ticks[n] - settled.ticks[n] for n in self.names if abs(after.ticks[n] - settled.ticks[n]) > 3}
                if moved:
                    raise Aborted(f'{where}: arm moved while the frame was taken: {moved}')
                name = f'{self.camera}-{pose.index:02d}.png'
                if not cv2.imwrite(str(frames_dir / name), bgr):
                    raise Aborted(f'cannot write {frames_dir / name}')
                records.append({'pose': pose.index, 'file': name, 'targets': pose.targets,
                                'measured': {n: settled.ticks[n] for n in self.names},
                                'frame': {k: meta.get(k) for k in ('camera_id', 'seq', 'captured_at', 'sha256')},
                                'expected_tags': pose.tags})
                self.log(f'{where}: saved {name}; returning to the start pose')
                self.move(start, where + ' return')
        except Aborted as exc:
            aborted = str(exc)
            self.log(f'ABORTED: {aborted}. Nothing more is sent; the arm holds where the owner left it '
                     '(STOP releases it).')
        log = {'arm': self.arm, 'frames': records, 'aborted': aborted, 'tools_called': sorted(set(self.calls)),
               'start_ticks': start, 'stop_count': None if self.initial is None else self.initial.stop_count}
        (self.out / 'capture.json').write_text(json.dumps(log, indent=1, default=str) + '\n')
        return log


# ------------------------------------------------------------------------------------------------ CLI
def _pairs(values):
    out = {}
    for v in values or []:
        k, _, p = v.partition('=')
        if not p:
            raise SystemExit(f'expected ARM=PATH, got {v!r}')
        out[k] = Path(p)
    return out


def load_robot(pilot_root: Path):
    # The pilot's authenticated client, as carton/fold_policy_runner.py loads it. Not in this repo.
    sys.path.insert(0, str(pilot_root.resolve()))
    import importlib
    return importlib.import_module('chat_server').Robot(pilot_root / '.private/robot.json')


def start_from_scene(scene_xml, maps):
    """Offline start pose: the trial's demonstration start (demo.npz next to run/), else the scene's qpos0."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(scene_xml))
    qpos = m.qpos0.copy()
    demo = Path(scene_xml).parent.parent / 'demo.npz'
    if demo.exists():
        qpos = np.load(demo)['qpos'][0]
    out = {}
    for a in ARMS:
        u = maps[a].units
        for s in SUFFIXES:
            v = qpos[m.jnt_qposadr[m.joint(f'{a}_{s}').id]]
            out[f'{a}_arm_{s}'] = int(round(u[s].model_zero_tick + u[s].model_sign * math.degrees(v) * 4096 / 360))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', choices=ARMS, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--pilot-root', type=Path, help='chat pilot checkout with chat_server.Robot and .private/robot.json')
    ap.add_argument('--joint-map', action='append', metavar='ARM=PATH',
                    help=f'per-arm joint map (default {DEFAULT_MAPS.relative_to(SOFTWARE)}/<arm>-joint-map.json)')
    ap.add_argument('--scene', type=Path, default=DEFAULT_SCENE, help='220 mm training scene.xml (planning only)')
    ap.add_argument('--poses', type=int, default=12)
    ap.add_argument('--samples', type=int, default=1500)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--clearance-m', type=float, default=.03)
    ap.add_argument('--duration-s', type=float, default=6.0, help='per move (the owner ramps at most 40 ticks / 0.4 s)')
    ap.add_argument('--settle-s', type=float, default=1.0)
    ap.add_argument('--execute', action='store_true', help='move the arm (default: plan only, nothing is sent)')
    ap.add_argument('--operator', help='name of the person holding STOP (required with --execute)')
    ap.add_argument('--no-solve', action='store_true', help='capture only; run calibrate_wrist_from_tags.py later')
    args = ap.parse_args(argv)
    if args.execute and not (args.operator and args.operator.strip()):
        raise SystemExit('--execute needs --operator (the person holding STOP)')
    if args.execute and args.pilot_root is None:
        raise SystemExit('--execute needs --pilot-root (the robot API client)')
    paths = _pairs(args.joint_map) or {a: DEFAULT_MAPS / f'{a}-joint-map.json' for a in ARMS}
    maps = load_arm_maps(paths)
    robot, ranges = None, None
    if args.pilot_root is not None:
        robot = load_robot(args.pilot_root)
        snap = parse_owner_status(_ok(robot.call('robot_get_execution', {}), 'robot_get_execution'),
                                  local_sent=time.time(), local_received=time.time())
        start, ranges = dict(snap.ticks), snap.ranges
        source = 'robot_get_execution (read-only)'
    else:
        start = start_from_scene(args.scene, maps)
        source = 'training scene start pose (offline; no robot contacted)'
    planner = ViewPlanner(args.scene, args.arm, maps, start, clearance=args.clearance_m, owner_ranges=ranges)
    plan = planner.plan(args.poses, samples=args.samples, seed=args.seed)
    plan.scene = str(args.scene)
    plan.settings['start_source'] = source
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'plan.json').write_text(json.dumps(plan.to_json(), indent=1) + '\n')
    print(f'start pose from {source}; {len(plan.poses)} poses planned ({plan.settings["candidates"]} candidates)')
    for p in plan.poses:
        print(f'  pose {p.index:2d}: {p.targets}  tags {p.tags}  {p.camera_distance_m:.2f} m, '
              f'{p.view_elevation_deg:.0f} deg down, max travel {p.max_travel_ticks} ticks')
    if len(plan.poses) < 8:
        print('REFUSED: fewer than 8 poses with clear views of the tags; check the scene and joint ranges', file=sys.stderr)
        return 2
    if not args.execute:
        print(f'dry-run: nothing sent. Plan in {args.out / "plan.json"}')
        return 0
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    log = Capture(robot, plan, args.out, duration_s=args.duration_s, settle_s=args.settle_s,
                  stop_requested=stop.is_set).run()
    log['operator'] = args.operator
    (args.out / 'capture.json').write_text(json.dumps(log, indent=1, default=str) + '\n')
    if log['aborted']:
        return 1
    if args.no_solve:
        return 0
    from tools import calibrate_wrist_from_tags as solver
    return solver.main(['--images', str(args.out / 'frames'), '--out', str(args.out / f'{args.arm}_wrist-intrinsics.json')])


if __name__ == '__main__':
    raise SystemExit(main())
