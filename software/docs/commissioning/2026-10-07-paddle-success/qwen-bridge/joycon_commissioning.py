"""Read-only commissioning guards and a binding of existing saved native units."""
import json
import math
from pathlib import Path

POSITION_NAMES = tuple(side + '_arm_' + joint for side in ('left', 'right')
                       for joint in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')) + ('head_motor_1', 'head_motor_2')
MOTOR_NAMES = set(POSITION_NAMES) | {'base_left_wheel', 'base_right_wheel'}
NATIVE_EVIDENCE = 'Saved native calibration for operator-authorized Joy-Con commissioning; no motor or calibration writes'
NATIVE_ROBOT_ID = 'farm_xlerobot'


def require_released(status):
    if not isinstance(status, dict):
        raise ValueError('Fresh sole-owner status required')
    age = status.get('age_s', status.get('status_age_s'))
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 1:
        raise ValueError('Sole-owner status is stale')
    if status.get('hardware_server') is not True or status.get('ok') is not True or status.get('phase') != 'idle':
        raise ValueError('Healthy idle hardware owner required')
    rows = status.get('rows', {})
    if set(rows) != MOTOR_NAMES or status.get('missing_buses') or status.get('enabled_motors'):
        raise ValueError('All 16 motors must be present and released')
    if any(row.get('Torque_Enable') != 0 for row in rows.values()):
        raise ValueError('All 16 torque-off readbacks required')
    if (status.get('teleop') or {}).get('active') or status.get('calibration_mismatches'):
        raise ValueError('Active manual session or calibration mismatch blocks commissioning')
    return status


def native_binding(calibration):
    from joycon_reference import NativeReference, native_record
    return NativeReference(native_record(calibration, NATIVE_ROBOT_ID, NATIVE_EVIDENCE))


def prepare_native_reference(calibration_path, status, directory, *, dry_run=False):
    require_released(status)
    calibration = json.loads(Path(calibration_path).read_text())
    reference = native_binding(calibration)
    for name in POSITION_NAMES:
        row = status['rows'][name]
        saved = reference.calibration[name]
        if list(row.get('firmware_position_limits', [])) != [saved['range_min'], saved['range_max']]:
            raise ValueError(name + ': live limits differ from saved calibration')
        reference.from_ticks(name, row['Present_Position'])
    destination = Path(directory) / (reference.reference_id + '.json')
    if not dry_run:
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if json.loads(destination.read_text()) != reference.record:
                raise ValueError('Existing reference does not match its fingerprint')
        else:
            with destination.open('x') as stream:
                stream.write(json.dumps(reference.record, indent=2) + '\n')
    return reference, destination
