"""Evaluate the DCM refit with per-physics-step contact gates. Simulation only.

Accepts eval_fold_policy.py arguments. The old harness remains available for
historical reproduction; new wrist-camera policies must use this harness.
"""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from carton.folding_contact_audit import sample_applied_contacts, score_applied_contacts
from carton.folding_sim import FoldingSimulation
from carton.refit_camera_contract import trial_contract, render_policy_camera
from tools import eval_fold_policy as base

AUDIT_ROOT = None
LATEST = None


def forbidden(a, b):
    return FoldingSimulation.forbidden_contact(None, a, b)


class AuditedEpisode(base.Episode):
    def __init__(self, trial, height, width, **kwargs):
        super().__init__(trial, height, width, **kwargs)
        self.camera_contract = trial_contract(trial)
        global LATEST
        LATEST = self
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = self.option.geomgroup[4] = 0
        self.audit_start = float(self.data.time)
        self.refusal = None
        seed = json.loads((trial/'demo.json').read_text())['seed']
        self.audit_path = AUDIT_ROOT/f'seed-{seed}-contacts.jsonl.gz'
        self.audit_stream = gzip.open(self.audit_path, 'wt', compresslevel=1)

    def images(self):
        images = {}
        jitter = getattr(self, 'image_jitter', None)
        for key, camera in base.CAMERAS.items():
            image = render_policy_camera(self.renderer, self.data, camera, self.option, self.camera_contract)
            images[key] = jitter(image) if jitter is not None else image
        return images

    def step(self, target):
        m, d = self.model, self.data
        target = np.asarray(target, float)
        if target.shape != self.lo.shape or not np.isfinite(target).all():
            self.refusal = 'Invalid policy action'
            return False
        for ctrl in self.ramp(target):
            d.ctrl[self.act_ids] = ctrl
            started = float(d.time)
            mujoco.mj_step(m, d)
            row = sample_applied_contacts(m, d, step_started_at=started, forbidden_contact=forbidden)
            self.audit_stream.write(json.dumps(row, separators=(',', ':'))+'\n')
            self.refusal = row['refusal_reason']
            for c in d.contact[:d.ncon]:
                pair = {int(c.geom1), int(c.geom2)}
                if pair & self.robot_geoms:
                    other = pair - self.robot_geoms
                    if other and other <= self.flap_geoms:
                        self.max_flap_pen = max(self.max_flap_pen, -c.dist*1000)
                    elif len(pair) > 1:
                        self.max_other_pen = max(self.max_other_pen, -c.dist*1000)
            self.max_carton_mm = max(self.max_carton_mm,
                float(np.linalg.norm(d.qpos[12:15]-self.carton0)*1000))
            if max(self.max_flap_pen, self.max_other_pen) > 1:
                self.refusal = self.refusal or 'Robot penetration exceeded 1 mm'
            if self.refusal or not np.isfinite(d.qpos).all():
                self.refusal = self.refusal or 'Nonfinite simulation state'
                return False
        return True

    def finish_audit(self):
        self.audit_stream.close()
        if float(self.data.time) <= self.audit_start:
            score = dict(status='CONTACT_AUDIT_INCOMPLETE', passed=False,
                         coverage_complete=False, observed_steps=0,
                         errors=['No physics steps executed'], physical_validation=False)
        else:
            with gzip.open(self.audit_path, 'rt') as stream:
                score = score_applied_contacts((json.loads(line) for line in stream),
                    expected_start_time=self.audit_start, expected_end_time=float(self.data.time),
                    forbidden_contact=forbidden)
        with self.audit_path.open('rb') as stream:
            sha = hashlib.file_digest(stream, 'sha256').hexdigest()
        return dict(score=score, sha256=sha, path=str(self.audit_path), refusal=self.refusal)


original_run = base.run


def audited_run(*args, **kwargs):
    global LATEST
    LATEST = None
    try:
        result = original_run(*args, **kwargs)
    finally:
        try:
            audit = LATEST.finish_audit() if LATEST is not None else None
        finally:
            if LATEST is not None:
                LATEST.renderer.close()
    result['contact_audit'] = audit
    result['success'] = bool(result['success'] and audit and audit['score']['passed'] and not audit['refusal'])
    return result


def main(argv=None):
    global AUDIT_ROOT
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--out' not in argv:
        raise SystemExit('Supply --out followed by a fresh output directory')
    AUDIT_ROOT = Path(argv[argv.index('--out')+1]).resolve()
    if '--replay' in argv:
        base.CAMERAS.clear()
        base.CAMERAS.update(front='front', left_wrist='left_wrist', right_wrist='right_wrist')
    base.Episode, base.run = AuditedEpisode, audited_run
    base.main(argv)


if __name__ == '__main__':
    main()
