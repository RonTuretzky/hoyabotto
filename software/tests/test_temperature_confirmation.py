import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
import pytest
from temperature_confirmation import confirm_temperature

def sample(temp=36, torque=0, status=0):
    return dict(temperature=temp, torque=torque, status=status)

def test_normal_temperature_does_not_release_or_poll():
    def unexpected(*args): raise AssertionError('unexpected callback')
    assert confirm_temperature(36,55,unexpected,unexpected,unexpected)==[]

def test_spike_requires_release_then_three_cool_fault_free_samples():
    calls=[]
    def release(): calls.append('release')
    def read(): calls.append('read'); return sample()
    result=confirm_temperature(82,55,release,read,lambda _:None)
    assert calls==['release','read','read','read']
    assert len(result)==3

@pytest.mark.parametrize('bad', [sample(82),sample(36,torque=1),sample(36,status=4)])
def test_bad_followup_fails_with_no_resume(bad):
    calls=[]
    with pytest.raises(RuntimeError):
        confirm_temperature(82,55,lambda:calls.append('release'),lambda:bad,lambda _:None)
    assert calls==['release']

def test_communication_failure_stays_released():
    calls=[]
    def failed(): raise OSError('no reply')
    with pytest.raises(OSError):
        confirm_temperature(82,55,lambda:calls.append('release'),failed,lambda _:None)
    assert calls==['release']
