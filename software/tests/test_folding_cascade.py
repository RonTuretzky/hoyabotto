"""Regression for a support handoff crossed during the contact approach."""
from types import SimpleNamespace

import numpy as np
import pytest

from carton.folding_cascade import press_near_over_short


class Scene:
    def __init__(self):
        self.angle = -15.
        self.log = []
        self.seq = 0
        self.box = np.eye(4)
        self.port = self
        self.data = SimpleNamespace(time=0., body=lambda _: SimpleNamespace(xmat=np.eye(3)))
        self.motion_stats = {}

    def sense(self, label):
        self.seq += 1
        return dict(seq=self.seq, world_from_box=self.box.tolist(), angles={
            'long_near': {'degrees': self.angle},
            'short_left': {'degrees': 90.}, 'short_right': {'degrees': 90.}})

    def move(self, targets, seconds, label, orientation):
        self.log.append(label)
        if label.startswith('Approach near-major'):
            self.angle += 2.
        elif label.startswith('Press near major'):
            self.angle = float(label.rsplit(' ', 1)[1])

    def move_arms(self, targets, seconds, label, orientation):
        self.log.append(label)

    def actual_control_position(self, side):
        return np.array([.1, -.04, .111])

    def ik(self, side, target, orientation):
        return np.zeros(6), 0.

    def truth_angles(self):
        return dict(long_near=self.angle, short_left=90., short_right=90.)

    def require_folded(self, reading, names):
        assert all(80 <= reading['angles'][name]['degrees'] <= 101 for name in names)


def test_support_withdraws_during_approach_before_sweep_and_uses_current_angle(monkeypatch):
    scene = Scene()
    monkeypatch.setattr('carton.folding_cascade.JointPathPlanner',
                        lambda *a, **kw: SimpleNamespace(plan=lambda q: [q]))
    monkeypatch.setattr('carton.folding_cascade.execute_path',
                        lambda sim, side, path, label, **kw: sim.log.append(label))
    result = press_near_over_short(scene, scene, from_open_claw=True,
                                   release_right_at_degrees=-10.)
    withdrawal = next(i for i, label in enumerate(scene.log) if label.startswith('Withdraw open'))
    approaches = [i for i, label in enumerate(scene.log) if label.startswith('Approach near-major')]
    presses = [label for label in scene.log if label.startswith('Press near major')]
    assert len(approaches) == 3 and max(approaches) < withdrawal
    assert presses[0].endswith('-9.0')  # Approach changed the panel from -15 to -9.
    assert result['right_released_after_visual_handoff']
    assert result['full_task_complete'] is False


def test_failed_withdrawal_prevents_further_contact_sweep(monkeypatch):
    scene = Scene()
    monkeypatch.setattr('carton.folding_cascade.JointPathPlanner',
                        lambda *a, **kw: SimpleNamespace(plan=lambda q: [q]))
    def execute(sim, side, path, label, **kwargs):
        if side == 'right':
            raise ValueError('Right withdrawal path blocked')
    monkeypatch.setattr('carton.folding_cascade.execute_path', execute)
    with pytest.raises(ValueError, match='withdrawal path blocked'):
        press_near_over_short(scene, scene, from_open_claw=True,
                              release_right_at_degrees=-10.)
    assert not any(label.startswith('Press near major') for label in scene.log)
