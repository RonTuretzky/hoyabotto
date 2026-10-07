"""No hardware: pinned provenance, failure cleanup, quality and installation gates."""
import contextlib
import copy
import json
import pathlib
import shutil
import sys
import types

import pytest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from carton_robot import upstream_pr3282_calibration as u


def valid_candidate():
    saved = json.loads((u.SOFTWARE / 'calibration/farm_xlerobot/farm_xlerobot.json').read_text())
    return {n: saved['left_arm_' + n] for n in u.JOINTS}


def good_readback(candidate):
    return {n: {'released': True, 'Torque_Enable': 0, 'Status': 0, 'Operating_Mode': 0,
                **{reg: candidate[n][key] for key, reg in u.REGISTERS.items()}} for n in u.JOINTS}


def test_original_sources_hashes_are_pinned(tmp_path):
    assert u.verify_sources()['commit'] == u.COMMIT
    source = tmp_path / 'source'
    shutil.copytree(u.SOURCE, source)
    (source / 'workflow.py').write_text('changed')
    with pytest.raises(ValueError, match='source changed'):
        u.verify_sources(source)


def test_plan_has_no_hardware_loader(monkeypatch, capsys):
    monkeypatch.setattr(u, 'original_workflow', lambda *_: pytest.fail('Plan opened hardware'))
    assert u.main(['--arm', 'left']) == 0
    assert json.loads(capsys.readouterr().out)['velocity'] == 300


def test_original_loader_restores_installed_driver_without_connecting():
    import lerobot.motors.feetech as package
    original = package.FeetechMotorsBus
    captured = []
    with u.original_workflow(captured) as workflow:
        assert workflow._fold_arm.__code__.co_filename == str(u.SOURCE / 'workflow.py')
        assert set(workflow.SO_FOLLOWER_MOTORS) == set(u.JOINTS)
        assert captured == []
        assert package.FeetechMotorsBus is not original
    assert package.FeetechMotorsBus is original


@pytest.mark.parametrize('fault', ['missing', 'short', 'mismatch', 'torque', 'mode', 'status', 'id'])
def test_bad_candidate_or_readback_is_not_valid(fault):
    c = valid_candidate()
    r = good_readback(c)
    if fault == 'missing': c.pop('gripper')
    if fault == 'short':
        c['shoulder_pan']['range_min'] = 1550
        c['shoulder_pan']['range_max'] = 2544
        r = good_readback(c)
    if fault == 'mismatch': r['elbow_flex']['Homing_Offset'] += 1
    if fault == 'torque': r['wrist_flex']['released'] = False
    if fault == 'mode': r['elbow_flex']['Operating_Mode'] = 1
    if fault == 'status': r['gripper']['Status'] = 8
    if fault == 'id': c['elbow_flex']['id'] = 4
    assert u.check_candidate(c, r, 'left')


def test_cleanup_attempts_every_motor_after_failures():
    class Bus:
        def __init__(self): self.writes = []
        def write(self, reg, name, value, **kwargs):
            self.writes.append((reg, name))
            if name == 'shoulder_pan': raise RuntimeError('No packet')
        def read(self, reg, name, **kwargs):
            if name == 'shoulder_pan': raise RuntimeError('No packet')
            return 0
    bus = Bus()
    result = u.release_and_read(bus)
    assert len(bus.writes) == 12
    assert not result['shoulder_pan']['released']
    assert result['gripper']['released']


def test_candidate_compares_saved_other_arm_travel():
    c = valid_candidate()
    other = {f'right_arm_{n}': copy.deepcopy(v) for n, v in c.items()}
    assert not u.check_candidate(c, good_readback(c), 'left', other)
    other['right_arm_elbow_flex']['range_max'] -= 500
    assert any('left/right' in p for p in u.check_candidate(c, good_readback(c), 'left', other))


@pytest.mark.parametrize('case', ['success', 'short', 'failure', 'release_failure', 'other_off'])
def test_only_verified_complete_run_installs(tmp_path, monkeypatch, case):
    candidate = valid_candidate()
    if case == 'short': candidate['shoulder_pan'].update(range_min=1550, range_max=2544)
    live = tmp_path / 'live.json'
    original = {'head_motor_1': {'id': 7, 'note': 'preserve'}, 'right_arm_gripper': {'id': 6, 'note': 'preserve'}}
    live.write_text(json.dumps(original))
    reads = good_readback(candidate)
    @contextlib.contextmanager
    def fake_workflow(captured):
        class Bus:
            def __init__(self, port, motors):
                self.port = port; self.is_connected = False; captured.append(self)
            def connect(self, **kwargs):
                assert case != 'other_off' or self.port != '/right', 'Must not open powered-off bus'
                self.is_connected = True
            def disconnect(self, **kwargs): self.is_connected = False
            def write(self, reg, name, value, **kwargs): pass
            def read(self, reg, name, **kwargs):
                if case == 'release_failure' and running[0] and self.port == '/left':
                    raise RuntimeError('Missing feedback')
                return reads[name][reg]
        running = [False]
        workflow = types.SimpleNamespace(FeetechMotorsBus=Bus, SO_FOLLOWER_MOTORS={n: None for n in u.JOINTS})
        def run(port, **kwargs):
            running[0] = True
            Bus(port, {}).connect()
            if case == 'failure': return 1
            path = workflow.HF_LEROBOT_CALIBRATION / 'robots/so_follower/left.json'
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(candidate))
            return 0
        workflow.run_full_calibration = run
        yield workflow
    class Lock:
        def __init__(self, *_): pass
        def acquire(self): return self
        def close(self): pass
    from carton_robot import servo_ownership
    monkeypatch.setattr(servo_ownership, 'ServoOwnership', Lock)
    monkeypatch.setattr(u, 'original_workflow', fake_workflow)
    monkeypatch.setattr(u, 'install_calibration_reply_guard', lambda *_: None)
    monkeypatch.setattr('builtins.input', lambda *_: 'yes')
    directory = tmp_path / 'run'
    rc = u.main(['--arm', 'left', '--execute', '--clearance-confirmed', '--install',
                 '--port-left', '/left', '--port-right', '/right',
                 '--calibration-file', str(live), '--backup-dir', str(directory)] +
                (['--other-arm-powered-off'] if case == 'other_off' else []))
    result = json.loads((directory / 'result.json').read_text())
    if case in ('success', 'other_off'):
        assert rc == 0 and result['installed']
        updated = json.loads(live.read_text())
        assert all(updated[k] == v for k, v in original.items())
        assert updated['left_arm_elbow_flex'] == candidate['elbow_flex']
    else:
        assert rc != 0 and not result['installed']
        assert json.loads(live.read_text()) == original
