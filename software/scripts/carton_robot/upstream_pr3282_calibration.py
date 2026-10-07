"""Pinned upstream calibration, staged results, exclusive ownership and verified cleanup.

The three upstream sources are hash-checked and never patched. Post-run quality checks
are separate from upstream stall detection. Plan-only is the default; no servo I/O occurs.
"""
import argparse
import contextlib
import hashlib
import importlib.util
import json
import pathlib
import shutil
import sys
import time

SOFTWARE = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOFTWARE / 'scripts'))
from carton_robot.guarded_pr3282_calibration import CAL, PORTS, install_calibration_reply_guard

SOURCE = SOFTWARE / 'farm/vendor/autocal/upstream_pr3282'
COMMIT = '1c8e185e3694def2469d7d312c78dab217e336da'
SOURCE_HASHES = {
    'auto_calibration.py': '9c383e087c4caec0c2fb98f0c6171a84fa16dc7147ed19977635fa3aeeab72cd',
    'calibration_defaults.py': '05342632e50a2f17d3435dc5f7bff20530c5c562d216551398b917040a332e3e',
    'workflow.py': '76db4f4256bf9cd631a292a0c6e496621cf81fe20c8aa063740cc079eb4d6552',
}
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
REGISTERS = {'homing_offset': 'Homing_Offset', 'range_min': 'Min_Position_Limit', 'range_max': 'Max_Position_Limit'}


def verify_sources(source=SOURCE):
    manifest = json.loads((source / 'provenance.json').read_text())
    if manifest['commit'] != COMMIT or set(manifest['files']) != {'workflow.py', 'auto_calibration.py', 'calibration_defaults.py'}:
        raise ValueError('Unexpected upstream provenance')
    for name, record in manifest['files'].items():
        if record['sha256'] != SOURCE_HASHES[name] or hashlib.sha256((source / name).read_bytes()).hexdigest() != SOURCE_HASHES[name]:
            raise ValueError(f'Upstream source changed: {name}')
    return manifest


@contextlib.contextmanager
def original_workflow(captured):
    verify_sources()
    import lerobot.motors.feetech as package
    from lerobot.motors.feetech import FeetechMotorsBus as stock
    aliases = ('lerobot.motors.feetech.calibration_defaults', 'lerobot.motors.feetech.auto_calibration', '_farm_original_pr3282_workflow')
    previous = {name: sys.modules.get(name) for name in aliases}
    def load(name, filename):
        spec = importlib.util.spec_from_file_location(name, SOURCE / filename)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module
    try:
        load(aliases[0], 'calibration_defaults.py')
        mixin = load(aliases[1], 'auto_calibration.py')
        class RecordingBus(mixin.FeetechCalibrationMixin, stock):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                captured.append(self)
        package.FeetechMotorsBus = RecordingBus
        yield load(aliases[2], 'workflow.py')
    finally:
        package.FeetechMotorsBus = stock
        for name, module in previous.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def read_settings(bus):
    return {name: {reg: bus.read(reg, name, normalize=False) for reg in
                  ('Torque_Enable', 'Status', 'Operating_Mode', *REGISTERS.values())} for name in JOINTS}


def release_and_read(bus):
    """Try every motor even if an earlier stop fails; never claim missing replies are off."""
    report = {}
    for name in JOINTS:
        row = {}
        for reg in ('Goal_Velocity', 'Torque_Enable'):
            try:
                bus.write(reg, name, 0, normalize=False, num_retry=3)
            except BaseException as exc:
                row.setdefault('write_errors', []).append(f'{reg}: {exc}')
        try:
            row.update({reg: bus.read(reg, name, normalize=False) for reg in
                        ('Torque_Enable', 'Status', 'Operating_Mode', *REGISTERS.values())})
        except BaseException as exc:
            row['read_error'] = str(exc)
        row['released'] = row.get('Torque_Enable') == 0
        report[name] = row
    return report


def check_candidate(candidate, readback, arm, reference=None):
    from farm.tools.calibration_report import analyse, MISMATCH_DEG
    problems = []
    if set(candidate) != set(JOINTS):
        return ['Candidate must contain exactly six arm joints']
    for motor_id, name in enumerate(JOINTS, 1):
        c = candidate[name]
        if not isinstance(c, dict) or any(type(c.get(key)) is not int for key in ('id', 'drive_mode', *REGISTERS)):
            problems.append(f'{name}: malformed calibration')
            continue
        if c['id'] != motor_id or c['drive_mode'] != 0 or not -2047 <= c['homing_offset'] <= 2047 or not 0 <= c['range_min'] < c['range_max'] <= 4095:
            problems.append(f'{name}: invalid calibration values')
            continue
        row = readback.get(name, {})
        if not row.get('released') or row.get('Status') != 0 or row.get('Operating_Mode') != 0:
            problems.append(f'{name}: release/health/position mode not verified')
        if any(row.get(reg) != c[key] for key, reg in REGISTERS.items()):
            problems.append(f'{name}: hardware does not match candidate')
    if problems:
        return problems
    # These are post-run report thresholds, not changes to upstream limit-seeking.
    report = analyse({f'{arm}_arm_{name}': c for name, c in candidate.items()})
    # A staged arm intentionally has no head/other-arm entries. Inspect its rows,
    # rather than treating the whole-robot report's missing-entry list as a fault.
    problems = [f"{row['joint']}: {', '.join(row['flags'])}" for row in report['rows']
                if any(flag in ('wrapped', 'short', 'too wide') for flag in row['flags'])]
    other_arm = 'right' if arm == 'left' else 'left'
    for name, c in candidate.items():
        other = (reference or {}).get(f'{other_arm}_arm_{name}', {})
        if all(type(other.get(k)) is int for k in ('range_min', 'range_max')):
            difference = abs((c['range_max'] - c['range_min']) -
                             (other['range_max'] - other['range_min'])) * 360 / 4096
            if difference > MISMATCH_DEG:
                problems.append(f'{name}: left/right saved travel differs by {difference:.1f} degrees')
    return problems


class Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, value):
        for stream in self.streams:
            stream.write(value)
            stream.flush()
        return len(value)
    def flush(self):
        for stream in self.streams:
            stream.flush()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', choices=('left', 'right'), required=True)
    p.add_argument('--velocity', type=int, default=300)
    p.add_argument('--timeout', type=float, default=20)
    p.add_argument('--backup-dir', type=pathlib.Path)
    p.add_argument('--port-left', default=PORTS[0])
    p.add_argument('--port-right', default=PORTS[1])
    p.add_argument('--calibration-file', type=pathlib.Path, default=CAL)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--clearance-confirmed', action='store_true')
    p.add_argument('--other-arm-powered-off', action='store_true', help='Physical confirmation: the other arm has 12 V OFF; skip its feedback preflight')
    p.add_argument('--install', action='store_true', help='Atomically merge only after full candidate validation and verified release')
    a = p.parse_args(argv)
    if not 1 <= a.velocity <= 1000 or not 1 <= a.timeout <= 45:
        p.error('velocity must be 1..1000 and timeout 1..45')
    if a.port_left == a.port_right:
        p.error('Left and right ports must differ')
    verify_sources()
    if not a.execute:
        print(json.dumps({'plan_only': True, 'commit': COMMIT, 'arm': a.arm,
                          'velocity': a.velocity, 'timeout': a.timeout, 'install': a.install}))
        return 0
    if not a.clearance_confirmed:
        p.error('--execute requires --clearance-confirmed')
    from farm.tools.auto_calibrate import CHECKLIST, merge_arm
    from carton_robot.servo_ownership import ServoOwnership
    print(CHECKLIST.format(arm=a.arm))
    print('Watch the entire sweep. The cart front is not modelled. A stopped fold is not proof of clearance.')
    if input('Type yes when people, cart and cables are clear and 12 V stop is reachable: ').strip().lower() != 'yes':
        return 2
    directory = a.backup_dir or SOFTWARE / 'data/calibration_runs' / f'{a.arm}-{time.time_ns()}'
    directory.mkdir(parents=True, exist_ok=False)
    result = {'arm': a.arm, 'commit': COMMIT, 'velocity': a.velocity, 'timeout': a.timeout,
              'routine_completed': False, 'full_range_validated': False, 'installed': False,
              'other_arm_power_off_confirmed': a.other_arm_powered_off}
    captured = []
    readback = {}
    candidate = None
    lock = None
    ports = (a.port_left, a.port_right)
    rc = 1
    with (directory / 'run.log').open('w') as log, contextlib.redirect_stdout(Tee(sys.stdout, log)), contextlib.redirect_stderr(Tee(sys.stderr, log)):
        try:
            lock = ServoOwnership(ports).acquire()
            if a.calibration_file.exists():
                shutil.copy2(a.calibration_file, directory / 'calibration-before.json')
            with original_workflow(captured) as workflow:
                preflight = {}
                for arm, port in zip(('left', 'right'), ports):
                    if arm != a.arm and a.other_arm_powered_off:
                        preflight[arm] = {}
                        continue
                    b = workflow.FeetechMotorsBus(port=port, motors=workflow.SO_FOLLOWER_MOTORS.copy())
                    try:
                        b.connect(handshake=False)
                        install_calibration_reply_guard(b)
                        preflight[arm] = read_settings(b)
                    finally:
                        if b.is_connected:
                            b.disconnect(disable_torque=False)
                (directory / 'preflight.json').write_text(json.dumps(preflight, indent=2))
                if any(row['Torque_Enable'] != 0 or row['Status'] != 0 for arm in preflight.values() for row in arm.values()):
                    raise RuntimeError('Preflight requires both arms released and healthy')
                workflow.HF_LEROBOT_CALIBRATION = directory / 'candidate'
                code = workflow.run_full_calibration(ports[0 if a.arm == 'left' else 1], save=True,
                    robot_id=a.arm, velocity_limit=a.velocity, timeout_s=a.timeout, interactive=True)
                result['upstream_return'] = code
                result['routine_completed'] = code == 0
                if code != 0:
                    raise RuntimeError(f'Upstream routine incomplete: {code}')
                candidate = json.loads((directory / 'candidate/robots/so_follower' / f'{a.arm}.json').read_text())
        except BaseException as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
            print(result['error'])
        finally:
            # The original routine may leave its port open when disconnect fails.
            selected = ports[0 if a.arm == 'left' else 1]
            for b in captured:
                if b.is_connected and str(b.port) == selected:
                    readback = release_and_read(b)
                    with contextlib.suppress(BaseException):
                        b.disconnect(disable_torque=False)
            if lock is not None and not readback:
                from farm.vendor.autocal.workflow import FeetechMotorsBus, SO_FOLLOWER_MOTORS
                b = FeetechMotorsBus(port=selected, motors=SO_FOLLOWER_MOTORS.copy())
                try:
                    b.connect(handshake=False)
                    install_calibration_reply_guard(b)
                    readback = release_and_read(b)
                except BaseException as exc:
                    result['cleanup_error'] = str(exc)
                finally:
                    if b.is_connected:
                        with contextlib.suppress(BaseException):
                            b.disconnect(disable_torque=False)
            result['release_verified'] = set(readback) == set(JOINTS) and all(row.get('released') for row in readback.values())
            (directory / 'release-readback.json').write_text(json.dumps(readback, indent=2))
            try:
                if not result['release_verified']:
                    if lock is not None:
                        print(f'SWITCH {a.arm.upper()} 12 V OFF: torque release could not be verified.')
                elif candidate is not None and result['routine_completed']:
                    before = directory / 'calibration-before.json'
                    reference = json.loads(before.read_text()) if before.exists() else {}
                    problems = check_candidate(candidate, readback, a.arm, reference)
                    result['problems'] = problems
                    result['full_range_validated'] = not problems
                    rc = 2 if problems else 0
                    if not problems and a.install:
                        if before.exists() and a.calibration_file.read_bytes() != before.read_bytes():
                            raise RuntimeError('Live calibration changed during run; refusing overwrite')
                        if not before.exists() and a.calibration_file.exists():
                            raise RuntimeError('Live calibration appeared during run; refusing overwrite')
                        merge_arm(a.calibration_file, a.arm, candidate)
                        result['installed'] = True
                    if problems:
                        print('Candidate retained; not installed. Resolve physical clearance/range issues before retrying:', problems)
            except BaseException as exc:
                result['install_error'] = str(exc)
                rc = 1
            finally:
                if lock is not None:
                    lock.close()
                (directory / 'result.json').write_text(json.dumps(result, indent=2))
                print(json.dumps(result), '\nEvidence:', directory)
    return rc


if __name__ == '__main__':
    sys.exit(main())
