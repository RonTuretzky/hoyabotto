"""Explicit physical reference for the legacy SO101 analytical degree convention.

URDF and analytical zeros are separate registrations. Saved range endpoints or
a convenient rest pose do not establish either. Loading this record performs
no motor I/O and does not commission collision/contact or station coordinates.
"""
import hashlib
import json
import argparse
from pathlib import Path

from .units import JointUnits, finite
from ..adapters.base import arm_joint
from ..vendor import so101_kinematics

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')


class AnalyticalReference:
    def __init__(self, record, calibration_file):
        if (record.get('schema')!=1 or record.get('model')!='so101_analytical'
                or record.get('arm') not in ('left','right')
                or not isinstance(record.get('registration_id'),str) or not record['registration_id'].strip()
                or not isinstance(record.get('evidence'),str) or not record['evidence'].strip()):
            raise ValueError('Explicit analytical reference identity and measured evidence required')
        model_hash=hashlib.sha256(Path(so101_kinematics.__file__).read_bytes()).hexdigest()
        if record.get('model_sha256')!=model_hash:
            raise ValueError('Analytical reference belongs to a different model convention')
        raw=Path(calibration_file).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=record.get('calibration_sha256'):
            raise ValueError('Analytical reference motor calibration changed')
        calibration=json.loads(raw)
        if set(record.get('joints',{}))!=set(JOINTS):
            raise ValueError('All five measured joint references are required')
        self.arm=record['arm'];self.registration_id=record['registration_id'];self.units={}
        for name in JOINTS:
            measured=record['joints'][name]
            tick=finite(measured.get('reference_tick'),'reference tick')
            degrees=finite(measured.get('reference_degrees'),'reference analytical degrees')
            sign=measured.get('model_sign')
            if type(sign) is not int or sign not in (-1,1):
                raise ValueError('Measure each joint direction; model_sign must be ±1')
            saved=calibration[arm_joint(self.arm,name)]
            # The zero is derived from a declared angle at the same measured
            # encoder pose, never guessed from range midpoint or raw homing.
            zero=tick-sign*degrees*4096/360
            units=JointUnits(saved['range_min'],saved['range_max'],zero,sign)
            units.ticks_to_normalized(tick)
            self.units[name]=units

    @classmethod
    def load(cls, path, calibration_file):
        return cls(json.loads(Path(path).read_text()),calibration_file)

    def normalized_from_degrees(self, degrees):
        if set(degrees)!=set(JOINTS):
            raise ValueError('Supply all five analytical joint angles')
        return {arm_joint(self.arm,name):units.ticks_to_normalized(units.model_degrees_to_ticks(degrees[name]))
                for name,units in self.units.items()}

    def degrees_from_normalized(self, normalized):
        if not {arm_joint(self.arm,n) for n in JOINTS}<=set(normalized):
            raise ValueError('All five measured normalized joints are required')
        return {name:units.ticks_to_model_degrees(units.normalized_to_ticks(normalized[arm_joint(self.arm,name)]))
                for name,units in self.units.items()}


def reference_template(arm, calibration_file):
    """Identity-bound blank record. Null angles/ticks/signs deliberately fail validation."""
    if arm not in ('left', 'right'):
        raise ValueError('Select left or right arm')
    raw = Path(calibration_file).read_bytes()
    calibration = json.loads(raw)
    for name in JOINTS:
        saved = calibration[arm_joint(arm, name)]
        JointUnits(saved['range_min'], saved['range_max'])
    return dict(schema=1, model='so101_analytical', arm=arm,
                registration_id='', evidence='',
                model_sha256=hashlib.sha256(Path(so101_kinematics.__file__).read_bytes()).hexdigest(),
                calibration_sha256=hashlib.sha256(raw).hexdigest(),
                joints={name: dict(reference_tick=None, reference_degrees=None, model_sign=None)
                        for name in JOINTS})


def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline SO101 analytical reference; no motor I/O')
    sub = parser.add_subparsers(dest='op', required=True)
    template = sub.add_parser('template')
    template.add_argument('--arm', choices=('left', 'right'), required=True)
    template.add_argument('--calibration', required=True)
    template.add_argument('--out', required=True)
    validate = sub.add_parser('validate')
    validate.add_argument('--reference', required=True)
    validate.add_argument('--calibration', required=True)
    args = parser.parse_args(argv)
    try:
        if args.op == 'template':
            record = reference_template(args.arm, args.calibration)
            # Never overwrite an existing measured record.
            with Path(args.out).open('x') as stream:
                json.dump(record, stream, indent=2)
                stream.write('\n')
            print(f'Unfilled template: {args.out}; this is not a commissioned reference')
        else:
            ref = AnalyticalReference.load(args.reference, args.calibration)
            print(f'Valid unit reference: {ref.arm}/{ref.registration_id}; physical Cartesian execution remains uncommissioned')
        return 0
    except (ValueError, KeyError, TypeError, OSError) as error:
        print(f'Refused: {error}')
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
