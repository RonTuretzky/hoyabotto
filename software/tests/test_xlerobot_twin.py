"""XLeRobot digital twin: mapping math (no MuJoCo needed) and rendering (skipped without model)."""
import io
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from farm.sim import xlerobot_twin as twin

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
MOTORS = [f'{side}_arm_{j}' for side in ('left', 'right') for j in ARM] + ['head_motor_1', 'head_motor_2']
RANGES = {m: (1000, 3000) for m in MOTORS}
RANGES['left_arm_gripper'] = (1275, 2819)
NEUTRAL = {m: (lo + hi) // 2 for m, (lo, hi) in RANGES.items()}
JAW = {'Jaw_L': (-21.46, 100.0), 'Jaw_R': (-21.46, 100.0)}


# ---------------------------------------------------------------- mapping math

def test_midpoint_is_zero_and_offsets_reach_the_model():
    angles, unmapped, model = twin.motor_angles(NEUTRAL, RANGES)
    assert unmapped == []
    for motor, (joint, offset, _) in twin.JOINT_TABLE.items():
        if offset is not None:
            assert angles[motor] == pytest.approx(0.0)
            assert model[joint] == pytest.approx(offset)


def test_sign_and_scale_4096_ticks_per_turn():
    ticks = dict(NEUTRAL, left_arm_shoulder_lift=2000 + 1024, right_arm_elbow_flex=2000 - 512)
    angles, _, model = twin.motor_angles(ticks, RANGES)
    assert angles['left_arm_shoulder_lift'] == pytest.approx(90.0)
    assert model['Pitch_L'] == pytest.approx(0.0)  # model Pitch = 90 - lift
    assert angles['right_arm_elbow_flex'] == pytest.approx(-45.0)
    assert model['Elbow_R'] == pytest.approx(45.0)  # model Elbow = 90 + elbow


def test_agrees_with_repo_candidate_degrees():
    pytest.importorskip('lerobot')
    from farm.kinematics.tag_registration import candidate_degrees
    names = [f'left_arm_{j}' for j in ARM[:5]]
    ticks = {n: t for n, t in zip(names, (1100, 1777, 2000, 2500, 2950))}
    ranges = {n: {'min_ticks': 1000, 'max_ticks': 3000} for n in names}
    angles, _, _ = twin.motor_angles(ticks, {n: (1000, 3000) for n in names})
    # LeRobot divides by 4095 instead of 4096: identical within 0.05 deg.
    for name, reference in zip(names, candidate_degrees(ticks, ranges, names)):
        assert angles[name] == pytest.approx(reference, abs=0.05)


def test_joint_map_overrides_zero_and_sign():
    joint_map = {'validated': False, 'joints': {
        'left_arm_elbow_flex': {'zero_tick': 2100, 'sign': -1},
        'left_arm_wrist_flex': {'sign': -1},
    }}
    ticks = dict(NEUTRAL, left_arm_elbow_flex=2200, left_arm_wrist_flex=2256)
    angles, _, model = twin.motor_angles(ticks, RANGES, joint_map)
    assert angles['left_arm_elbow_flex'] == pytest.approx(-100 * 360 / 4096)
    assert model['Elbow_L'] == pytest.approx(90 - 100 * 360 / 4096)
    assert angles['left_arm_wrist_flex'] == pytest.approx(-256 * 360 / 4096)  # midpoint kept, sign flipped
    assert angles['right_arm_elbow_flex'] == pytest.approx(0.0)  # untouched motors unchanged


def test_joint_map_zero_tick_needs_no_saved_range():
    ranges = {k: v for k, v in RANGES.items() if k != 'head_motor_1'}
    angles, unmapped, _ = twin.motor_angles(NEUTRAL, ranges, {'joints': {'head_motor_1': {'zero_tick': 1900}}})
    assert 'head_motor_1' not in unmapped
    assert angles['head_motor_1'] == pytest.approx(100 * 360 / 4096)


@pytest.mark.parametrize('bad', [
    {'joints': {'left_arm_elbow': {'sign': 1}}},          # unknown motor
    {'joints': {'left_arm_elbow_flex': {'sign': 2}}},     # bad sign
    {'joints': {'left_arm_elbow_flex': {'zero_tick': 5000}}},
    ['not', 'a', 'dict'],
])
def test_bad_joint_map_is_refused(bad):
    with pytest.raises(ValueError):
        twin.motor_angles(NEUTRAL, RANGES, bad)
    with pytest.raises(ValueError):
        twin.render_twin(NEUTRAL, RANGES, joint_map=bad)  # refused before any rendering


def test_gripper_linear_over_calibrated_range():
    lo, hi = RANGES['left_arm_gripper']
    for tick, fraction in ((lo, 0.0), (hi, 1.0), ((lo + hi) / 2, 0.5)):
        angles, _, model = twin.motor_angles(dict(NEUTRAL, left_arm_gripper=tick), RANGES, jaw_range_deg=JAW)
        assert angles['left_arm_gripper'] == pytest.approx(fraction * 121.46)
        assert model['Jaw_L'] == pytest.approx(-21.46 + fraction * 121.46)


def test_gripper_joint_map_gives_opening_from_closed():
    joint_map = {'joints': {'left_arm_gripper': {'zero_tick': 1300, 'sign': 1}}}
    angles, _, model = twin.motor_angles(dict(NEUTRAL, left_arm_gripper=1300 + 512), RANGES, joint_map, JAW)
    assert angles['left_arm_gripper'] == pytest.approx(45.0)
    assert model['Jaw_L'] == pytest.approx(-21.46 + 45.0)


def test_unmapped_reported_and_wheels_ignored():
    ticks = dict(NEUTRAL, base_left_wheel=17, base_right_wheel=4000, mystery_motor=5, left_arm_wrist_roll=None)
    ticks.pop('right_arm_gripper')
    ranges = {k: v for k, v in RANGES.items() if k != 'head_motor_2'}
    angles, unmapped, _ = twin.motor_angles(ticks, ranges)
    assert set(unmapped) == {'mystery_motor', 'left_arm_wrist_roll', 'right_arm_gripper', 'head_motor_2'}
    assert not {'base_left_wheel', 'base_right_wheel'} & (set(angles) | set(unmapped))
    assert set(angles) == set(MOTORS) - set(unmapped)


def test_missing_model_gives_clear_error(monkeypatch, tmp_path):
    monkeypatch.setenv(twin.MODEL_ENV, str(tmp_path / 'nope.xml'))
    with pytest.raises(FileNotFoundError, match=twin.MODEL_ENV):
        twin.find_model()


# ---------------------------------------------------------------- rendering

def _model_available():
    try:
        import mujoco  # noqa: F401
        twin.find_model()
        return True
    except (ImportError, FileNotFoundError):
        return False


render = pytest.mark.skipif(not _model_available(), reason='mujoco or the XLeRobot twin model is not available')


def _decode(data):
    from PIL import Image
    import numpy as np
    image = Image.open(io.BytesIO(data))
    assert image.format == 'JPEG'
    return np.asarray(image.convert('RGB'), dtype=float)


@render
def test_output_shape_views_and_jpeg():
    ticks = dict(NEUTRAL, base_left_wheel=1, base_right_wheel=2)
    result = twin.render_twin(ticks, RANGES)
    assert set(result) == {'images', 'angles_deg', 'unmapped', 'mapping', 'mapping_validated', 'model', 'claws'}
    assert result['claws'] == twin.claw_positions(ticks, RANGES)
    assert [i['view'] for i in result['images']] == list(twin.VIEWS)
    for image in result['images']:
        assert image['mime_type'] == 'image/jpeg' and image['data'][:3] == b'\xff\xd8\xff'
        assert _decode(image['data']).shape == (480, 640, 3)
    assert result['unmapped'] == [] and set(result['angles_deg']) == set(MOTORS)
    assert result['mapping'] == 'feetech_degrees_v1' and result['mapping_validated'] is False
    assert result['model']

    some = twin.render_twin(NEUTRAL, RANGES, views=('top', 'front'), size=(320, 240))
    assert [i['view'] for i in some['images']] == ['top', 'front']
    assert _decode(some['images'][0]['data']).shape == (240, 320, 3)
    with pytest.raises(ValueError):
        twin.render_twin(NEUTRAL, RANGES, views=('back',))


@render
def test_joint_map_flags():
    result = twin.render_twin(NEUTRAL, RANGES, views=('top',), joint_map={'validated': False, 'joints': {}})
    assert result['mapping'] == 'feetech_degrees_v1+joint_map' and result['mapping_validated'] is False
    result = twin.render_twin(NEUTRAL, RANGES, views=('top',), joint_map={'validated': True, 'joints': {}})
    assert result['mapping_validated'] is True


@render
def test_neutral_pose_is_so101_zero_pose():
    """All-zero angles: upper arms up, forearms and grippers forward (robot faces model -x)."""
    def probe():
        t = twin._twin(twin.find_model()[0])
        _, _, model_deg = twin.motor_angles(NEUTRAL, RANGES, None, t.jaw_deg)
        t.render(model_deg, ('top',), (64, 48))
        return {b: t.data.body(b).xpos.copy() for b in
                ('Upper_Arm', 'Lower_Arm', 'Wrist_Pitch_Roll', 'Fixed_Jaw', 'Upper_Arm_2', 'Lower_Arm_2', 'Fixed_Jaw_2')}
    p = twin._executor().submit(probe).result()
    for sfx in ('', '_2'):
        up = p['Lower_Arm' + sfx] - p['Upper_Arm' + sfx]
        assert up[2] > 0.1 and abs(up[1]) < 0.01            # upper arm vertical
        fwd = p['Fixed_Jaw' + sfx] - p['Lower_Arm' + sfx]
        assert fwd[0] < -0.18 and abs(fwd[2]) < 0.01 and abs(fwd[1]) < 0.01  # forearm + wrist forward, level
    assert p['Upper_Arm'][1] < 0 < p['Upper_Arm_2'][1]       # left arm on the robot's left (-y)


@render
def test_cached_between_calls_and_fast():
    twin.render_twin(NEUTRAL, RANGES)
    first = dict(twin._TWINS)
    start = time.perf_counter()
    twin.render_twin(NEUTRAL, RANGES)
    warm = time.perf_counter() - start
    assert twin._TWINS == first and len(first) == 1
    assert warm < 1.0


@render
def test_second_call_faster_in_fresh_process():
    code = ('import time,json; from farm.sim import xlerobot_twin as t\n'
            f'n={NEUTRAL!r}; r={RANGES!r}\n'
            'ts=[]\n'
            'for _ in range(2):\n    s=time.perf_counter(); t.render_twin(n, r); ts.append(time.perf_counter()-s)\n'
            'print(json.dumps(ts))')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=120,
                         cwd=Path(__file__).resolve().parents[1], env=dict(os.environ, PYTHONPATH='.'))
    assert out.returncode == 0, out.stderr
    cold, warm = json.loads(out.stdout.strip().splitlines()[-1])
    assert warm < cold and warm < 1.0


@render
def test_render_from_non_main_threads():
    results, errors = [], []

    def call():
        try:
            results.append(twin.render_twin(NEUTRAL, RANGES))
        except Exception as exc:  # pragma: no cover - surfaced below
            errors.append(exc)
    threads = [threading.Thread(target=call) for _ in range(3)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=60)
    assert not errors and len(results) == 3
    assert all(len(r['images']) == len(twin.VIEWS) for r in results)


@render
def test_changing_a_joint_changes_the_image():
    import numpy as np

    def changed(a, b):  # fraction of pixels that changed clearly (ignores JPEG noise)
        return (np.abs(_decode(a['data']) - _decode(b['data'])).max(axis=2) > 40).mean()
    bent_ticks = dict(NEUTRAL, left_arm_elbow_flex=NEUTRAL['left_arm_elbow_flex'] + 700)
    base = twin.render_twin(NEUTRAL, RANGES)
    bent = twin.render_twin(bent_ticks, RANGES)
    same = twin.render_twin(NEUTRAL, RANGES)
    flipped = twin.render_twin(bent_ticks, RANGES, joint_map={'joints': {'left_arm_elbow_flex': {'sign': -1}}})
    for a, b, c, d in zip(base['images'], bent['images'], same['images'], flipped['images']):
        assert changed(a, c) < 0.0005   # deterministic
        assert changed(a, b) > 0.003    # elbow visibly moved (measured 0.6-1.8 % of pixels)
        assert changed(b, d) > 0.003    # joint map sign flip bends it the other way


# ---------------------------------------------------------------- claw positions (kinematics only)

def _ticks(**degrees):
    """NEUTRAL with some motors moved by feetech_degrees_v1 degrees."""
    ticks = dict(NEUTRAL)
    for motor, deg in degrees.items():
        ticks[motor] = NEUTRAL[motor] + round(deg * 4096 / 360)
    return ticks


LEVEL = _ticks(left_arm_shoulder_lift=90, left_arm_elbow_flex=-90, right_arm_shoulder_lift=90, right_arm_elbow_flex=-90)
FOLDED = _ticks(left_arm_shoulder_lift=45, left_arm_elbow_flex=60, left_arm_wrist_flex=30,
                right_arm_shoulder_lift=45, right_arm_elbow_flex=60, right_arm_wrist_flex=30)
ARM_KEYS = {'forward_m', 'left_m', 'up_m', 'reach_m', 'shoulder_up_m', 'shoulder_left_m', 'tip_site'}


@render
def test_claw_positions_shape_and_frame():
    claws = twin.claw_positions(dict(NEUTRAL, base_left_wheel=3, mystery=9), RANGES)
    assert set(claws) == {'left_arm', 'right_arm', 'frame', 'mapping', 'mapping_validated', 'unmapped', 'model'}
    for arm in ('left_arm', 'right_arm'):
        assert set(claws[arm]) == ARM_KEYS
        assert all(isinstance(claws[arm][k], float) for k in ARM_KEYS - {'tip_site'})
    assert claws['left_arm']['tip_site'] == 'Fixed_Jaw/twin_tip_L'
    assert claws['right_arm']['tip_site'] == 'Fixed_Jaw_2/twin_tip_R'
    assert claws['frame'] == twin.FRAME and 'floor' in claws['frame'] and 'forward' in claws['frame']
    assert claws['mapping'] == 'feetech_degrees_v1' and claws['mapping_validated'] is False
    assert claws['unmapped'] == ['mystery'] and claws['model']
    json.dumps(claws, allow_nan=False)


@render
def test_zero_pose_tips_are_mirrored_and_plausible():
    """Upper arm up, forearm and gripper forward: tips ahead of and above the shoulders, mirrored in left_m."""
    claws = twin.claw_positions(NEUTRAL, RANGES)
    left, right = claws['left_arm'], claws['right_arm']
    assert left['left_m'] > 0.1 > -0.1 > right['left_m']
    assert abs(left['left_m'] + right['left_m']) < 0.001
    assert abs(left['forward_m'] - right['forward_m']) < 0.001 and abs(left['up_m'] - right['up_m']) < 0.001
    assert left['shoulder_left_m'] == pytest.approx(-right['shoulder_left_m'], abs=1e-6)
    assert 0.3 < left['forward_m'] < 0.42          # forearm + wrist + jaw ahead of the shoulder line
    assert left['up_m'] > left['shoulder_up_m'] > 0.7  # above the shoulder; shoulders ~0.9 m up on the cart
    assert left['up_m'] < 1.1


@render
def test_raising_shoulder_lift_moves_tip_up_monotonically():
    ups = [twin.claw_positions(_ticks(left_arm_shoulder_lift=deg), RANGES)['left_arm']['up_m']
           for deg in (60, 40, 20, 0, -20, -40, -60)]  # negative lift = upper arm tilting back/up in the model
    assert all(b > a + 0.01 for a, b in zip(ups, ups[1:]))
    rights = [twin.claw_positions(_ticks(left_arm_shoulder_lift=deg), RANGES)['right_arm']['up_m'] for deg in (60, 0)]
    assert rights[0] == pytest.approx(rights[1])  # the other arm is untouched


@render
def test_reach_within_the_so101_envelope():
    for ticks in (NEUTRAL, LEVEL, FOLDED):
        for arm in ('left_arm', 'right_arm'):
            claw = twin.claw_positions(ticks, RANGES)[arm]
            assert 0.10 <= claw['reach_m'] <= 0.45
    level = twin.claw_positions(LEVEL, RANGES)['left_arm']
    assert level['forward_m'] > 0.4 and abs(level['up_m'] - level['shoulder_up_m']) < 0.05  # straight out, level
    folded = twin.claw_positions(FOLDED, RANGES)['left_arm']
    assert folded['reach_m'] < level['reach_m'] - 0.2


@render
def test_claw_positions_respect_joint_map():
    bent = _ticks(left_arm_shoulder_lift=40)
    plain = twin.claw_positions(bent, RANGES)
    flipped = twin.claw_positions(bent, RANGES, joint_map={'validated': True, 'joints': {
        'left_arm_shoulder_lift': {'sign': -1}}})
    mirror = twin.claw_positions(_ticks(left_arm_shoulder_lift=-40), RANGES)
    assert flipped['left_arm']['up_m'] == pytest.approx(mirror['left_arm']['up_m'])
    assert flipped['left_arm']['up_m'] != pytest.approx(plain['left_arm']['up_m'])
    assert flipped['mapping'] == 'feetech_degrees_v1+joint_map' and flipped['mapping_validated'] is True
    with pytest.raises(ValueError):
        twin.claw_positions(NEUTRAL, RANGES, joint_map={'joints': {'nope': {}}})


@render
def test_claw_positions_fast_and_thread_safe():
    twin.claw_positions(NEUTRAL, RANGES)  # model load
    times = []
    for _ in range(20):
        start = time.perf_counter()
        twin.claw_positions(LEVEL, RANGES)
        times.append(time.perf_counter() - start)
    assert sorted(times)[len(times) // 2] < 0.02
    results, errors = [], []

    def call(ticks):
        try:
            results.append((ticks is LEVEL, twin.claw_positions(ticks, RANGES)['left_arm']['forward_m']))
        except Exception as exc:  # pragma: no cover - surfaced below
            errors.append(exc)
    threads = [threading.Thread(target=call, args=(LEVEL if i % 2 else NEUTRAL,)) for i in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=60)
    assert not errors and len(results) == 6
    expect = {True: twin.claw_positions(LEVEL, RANGES)['left_arm']['forward_m'],
              False: twin.claw_positions(NEUTRAL, RANGES)['left_arm']['forward_m']}
    assert all(value == expect[is_level] for is_level, value in results)
