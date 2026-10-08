"""Regression checks for unexpected movement during calibration startup."""
import pytest
from farm.vendor.autocal import workflow as w

class Bus:
    def __init__(self, bad=None):
        self.motors = {'gripper': object()}
        self.values = {}
        self.log = []
        self.bad = bad
    def write(self, reg, motor, value, **kw):
        self.log.append((reg, motor, value))
        self.values[reg, motor] = value
    def read(self, reg, motor, **kw):
        if reg == 'Present_Position': return 2200
        if reg == self.bad: return -1
        return self.values[reg, motor]
    def write_position_limits(self, motor, low, high): self.limits = (low, high)
    def read_position_limits(self, motor): return self.limits


def test_startup_only_enables_selected_joint_after_verified_hold():
    bus = Bus()
    w._run_init(bus)
    assert {m for _, m, _ in bus.log} == {'gripper'}
    assert bus.log.index(('Goal_Position', 'gripper', 2200)) < bus.log.index(('Torque_Enable', 'gripper', 1))


@pytest.mark.parametrize('bad', ['Torque_Enable', 'Homing_Offset', 'Goal_Position'])
def test_failed_verification_never_enables_torque(bad):
    bus = Bus(bad)
    with pytest.raises(RuntimeError): w._run_init(bus)
    assert ('Torque_Enable', 'gripper', 1) not in bus.log


def test_single_joint_uses_restricted_bus(monkeypatch):
    calls = []
    monkeypatch.setattr(w, '_run_with_bus', lambda *a, **kw: calls.append(kw) or 0)
    assert w.calibrate_single_motor('/fake', 'gripper', interactive=False) == 0
    assert calls == [{'motor_names': ['gripper']}]


def test_velocity_preparation_import_resolves(monkeypatch):
    from farm.vendor.autocal.auto_calibration import FeetechCalibrationMixin
    FeetechCalibrationMixin._prepare_motors_for_range_measure(object(), [])

@pytest.mark.parametrize('reason', ['timeout(1s)', 'stall confirmed(2x): communication error'])
def test_invalid_limit_stops_and_aborts(reason, monkeypatch):
    from farm.vendor.autocal.auto_calibration import FeetechCalibrationMixin
    from farm.vendor.autocal import auto_calibration as ac
    monkeypatch.setattr(ac.time, 'sleep', lambda _: None)
    class SeekingBus:
        def __init__(self): self.stops = []
        def sync_write(self, *a, **kw): pass
        def _wait_for_stall(self, *a): return reason
        def _read_with_retry(self, *a): return 2200
        def write(self, reg, motor, value, **kw): self.stops.append((reg, motor, value))
    bus = SeekingBus()
    with pytest.raises(RuntimeError, match='limit not established'):
        FeetechCalibrationMixin._run_direction_until_stall(bus, ['gripper'], 100)
    assert bus.stops == [('Goal_Velocity', 'gripper', 0)]

@pytest.mark.parametrize('ranges', [(2047,2047),(12,4082)])
def test_rejects_false_shoulder_limits_before_writes(ranges):
    class Measurement:
        def measure_ranges_of_motion_multi(self, *args, **kw):
            return {'shoulder_lift': (*ranges, 2047, 0, 0, 0)}
    with pytest.raises(RuntimeError, match='invalid measured travel'):
        w._calibrate_motors(Measurement(), ['shoulder_lift'])
