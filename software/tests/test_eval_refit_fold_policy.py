import io
import gzip
from types import SimpleNamespace

import numpy as np

from tools import eval_refit_fold_policy as ev


def episode():
    ep = object.__new__(ev.AuditedEpisode)
    ep.model = SimpleNamespace()
    ep.data = SimpleNamespace(ctrl=np.zeros(12), qpos=np.zeros(15), time=0., contact=[], ncon=0)
    ep.lo, ep.hi = np.full(12, -2.), np.full(12, 2.)
    ep.act_ids = list(range(12))
    ep.substeps = 50
    ep.robot_geoms, ep.flap_geoms = set(), set()
    ep.max_flap_pen = ep.max_other_pen = ep.max_carton_mm = 0.
    ep.carton0 = np.zeros(3)
    ep.audit_stream = io.StringIO()
    ep.refusal = None
    return ep


def test_transient_camera_contact_stops_before_end_of_policy_tick(monkeypatch):
    ep = episode()
    def step(m, d):
        d.time += .002
    def contacts(m, d, **kwargs):
        return {'refusal_reason': 'Loaded camera contact' if d.time >= .006 else None}
    monkeypatch.setattr(ev.mujoco, 'mj_step', step)
    monkeypatch.setattr(ev, 'sample_applied_contacts', contacts)
    assert not ep.step(np.zeros(12))
    assert ep.data.time == .006
    assert len(ep.audit_stream.getvalue().splitlines()) == 3
    assert ep.refusal == 'Loaded camera contact'


def test_nonfinite_policy_action_cannot_advance_physics(monkeypatch):
    ep = episode()
    def forbidden_step(*args):
        raise AssertionError('Invalid action reached physics')
    monkeypatch.setattr(ev.mujoco, 'mj_step', forbidden_step)
    assert not ep.step(np.full(12, np.nan))
    assert ep.data.time == 0


def test_first_action_refusal_is_a_failed_audit_not_an_exception(tmp_path):
    ep = episode()
    ep.audit_start = 0.
    ep.audit_path = tmp_path/'contacts.jsonl.gz'
    ep.audit_stream = gzip.open(ep.audit_path, 'wt')
    assert not ep.step(np.full(12, np.nan))
    audit = ep.finish_audit()
    assert not audit['score']['passed']
    assert not audit['score']['coverage_complete']
    assert audit['score']['observed_steps'] == 0
    assert audit['refusal'] == 'Invalid policy action'
