"""Offline reference fixtures are synthetic; none commission a physical arm."""
import copy
import hashlib
import json
from pathlib import Path

import pytest

from farm.adapters.base import arm_joint
from farm.kinematics.analytical_reference import AnalyticalReference, JOINTS
from farm.skills.arm import ArmModel, ArmPose
from farm.vendor import so101_kinematics


@pytest.fixture
def reference_record(tmp_path):
    calibration = {arm_joint('right', name): {'range_min': 900+i*10, 'range_max': 3300-i*10}
                   for i, name in enumerate(JOINTS)}
    path = tmp_path/'calibration.json'
    path.write_text(json.dumps(calibration))
    angles = dict(zip(JOINTS, (18, 20, 30, -25, -11)))
    record = dict(schema=1, model='so101_analytical', arm='right',
                  registration_id='synthetic-test', evidence='SYNTHETIC TEST FIXTURE',
                  calibration_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(Path(so101_kinematics.__file__).read_bytes()).hexdigest(),
                  joints={name: dict(reference_tick=2200+i*30, reference_degrees=angles[name],
                                     model_sign=-1 if name in ('elbow_flex','wrist_roll') else 1)
                          for i, name in enumerate(JOINTS)})
    return record, path


def test_registered_pose_and_negative_direction_roundtrip(reference_record):
    record, path = reference_record
    reference = AnalyticalReference(record, path)
    angles = {name: spec['reference_degrees'] for name, spec in record['joints'].items()}
    normalized = reference.normalized_from_degrees(angles)
    assert reference.degrees_from_normalized(normalized) == pytest.approx(angles)
    for name, units in reference.units.items():
        assert units.normalized_to_ticks(normalized[arm_joint('right', name)]) == pytest.approx(record['joints'][name]['reference_tick'])
    assert normalized['right_arm_shoulder_lift'] != pytest.approx(20)
    changed = {**angles, 'elbow_flex': 36}
    assert reference.units['elbow_flex'].normalized_to_ticks(reference.normalized_from_degrees(changed)['right_arm_elbow_flex']) == pytest.approx(2260-6*4096/360)


def test_arm_ik_and_feedback_fk_use_registered_units(reference_record):
    record, path = reference_record
    reference = AnalyticalReference(record, path)
    kin = so101_kinematics.SO101Kinematics()
    x, y = kin.forward_kinematics(20, 30)
    model = ArmModel('right', reference=reference, x_range=(-1, 1), y_range=(-1, 1))
    joints = model.joints_for(ArmPose(x=x, y=y, pitch=25, pan=18, roll=-11, gripper=42))
    expected = reference.normalized_from_degrees(dict(zip(JOINTS, (18, 20, 30, -25, -11))))
    assert {key: joints[key] for key in expected} == pytest.approx(expected)
    model.sync_from_joints(joints)
    assert (model.pose.x, model.pose.y, model.pose.pitch, model.pose.pan, model.pose.roll, model.pose.gripper) == pytest.approx((x, y, 25, 18, -11, 42))
    with pytest.raises(ValueError, match='another arm'):
        ArmModel('left', reference=reference)


@pytest.mark.parametrize('mutation', ['evidence', 'model_sha256', 'calibration_sha256', 'missing_angle', 'missing_sign', 'missing_joint', 'outside_reference'])
def test_registration_requires_measured_complete_bound_reference(reference_record, mutation):
    original, path = reference_record
    record = copy.deepcopy(original)
    if mutation in ('evidence', 'model_sha256', 'calibration_sha256'):
        record[mutation] = ''
    elif mutation == 'missing_angle':
        del record['joints']['shoulder_lift']['reference_degrees']
    elif mutation == 'missing_sign':
        del record['joints']['shoulder_lift']['model_sign']
    elif mutation == 'missing_joint':
        del record['joints']['wrist_roll']
    else:
        record['joints']['shoulder_lift']['reference_tick'] = 899
    with pytest.raises(ValueError):
        AnalyticalReference(record, path)


def test_changed_calibration_and_unreachable_angles_refuse(reference_record):
    record, path = reference_record
    reference = AnalyticalReference(record, path)
    angles = {name: spec['reference_degrees'] for name, spec in record['joints'].items()}
    with pytest.raises(ValueError, match='outside calibration'):
        reference.normalized_from_degrees({**angles, 'shoulder_lift': 300})
    with pytest.raises(ValueError, match='All five'):
        reference.degrees_from_normalized({'right_arm_shoulder_lift': 0})
    path.write_text(path.read_text()+'\n')
    with pytest.raises(ValueError, match='calibration changed'):
        AnalyticalReference(record, path)


def test_physical_runner_still_refuses_cartesian_without_commissioning():
    from farm.config import ArmsCfg, LimitsCfg
    from farm.safety.rules import SafetyStop
    from farm.skills.runner import SkillRunner
    runner = SkillRunner(None, LimitsCfg(), ArmsCfg(), None)
    with pytest.raises(SafetyStop, match='Cartesian'):
        runner.move_arm_pose('right', ArmPose())


def test_offline_cli_template_refuses_unfilled_then_validates_measured_fixture(reference_record, tmp_path, capsys):
    from farm.kinematics.analytical_reference import main
    record, calibration = reference_record
    path = tmp_path/'reference.json'
    assert main(['template', '--arm', 'right', '--calibration', str(calibration), '--out', str(path)]) == 0
    blank = json.loads(path.read_text())
    assert all(spec['reference_tick'] is None and spec['reference_degrees'] is None and spec['model_sign'] is None for spec in blank['joints'].values())
    assert main(['validate', '--reference', str(path), '--calibration', str(calibration)]) == 2
    # Template command cannot overwrite a filled or unfilled record.
    assert main(['template', '--arm', 'right', '--calibration', str(calibration), '--out', str(path)]) == 2
    path.write_text(json.dumps(record))
    assert main(['validate', '--reference', str(path), '--calibration', str(calibration)]) == 0
    assert 'physical Cartesian execution remains uncommissioned' in capsys.readouterr().out


def test_vertical_upper_arm_horizontal_forearm_partial_reference_convention():
    import math
    lift = -math.degrees(math.atan2(.028, .11257))
    elbow = math.degrees(math.atan2(.0052, .1349)+math.atan2(.028, .11257))
    kin = so101_kinematics.SO101Kinematics()
    assert kin.forward_kinematics(lift, elbow) == pytest.approx((kin.l2, kin.l1), abs=1e-12)
