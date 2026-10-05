import json
import pytest
from test_carton_servo import FileMotorOwner
from carton.servo.common import Limits, Refused
from carton.servo.transport import SessionTransport


class EndpointOwner(FileMotorOwner):
    def __init__(self, folder, displacement):
        super().__init__(folder)
        self.displacement=displacement
        self.config['joints']=['right_arm_elbow_flex','right_arm_shoulder_pan']
    def sleep(self, seconds):
        prior=self.seen
        super().sleep(seconds)
        if self.seen!=prior:
            command=json.loads((self.folder/'command.json').read_text())
            if command['op']=='move':
                joint=next(iter(command['delta_ticks']))
                self.q[joint]=2000+self.displacement
                self.publish()


@pytest.mark.parametrize('ticks,displacement',[(68,47),(-68,-47)])
def test_explicit_elbow24_accepts_stable47_of68_in_correct_direction(tmp_path,ticks,displacement):
    owner=EndpointOwner(tmp_path/'session',displacement)
    with SessionTransport(owner.config,Limits(),True,owner.clock,owner.sleep) as transport:
        q,_=transport.move('right_arm_elbow_flex',ticks,reviewed_settle_ticks=24)
        assert q['right_arm_elbow_flex']==2000+displacement


@pytest.mark.parametrize('displacement',[-2,20,72,40])
def test_wrong_direction_small_motion_overshoot_or_oversized_error_refused(tmp_path,displacement):
    owner=EndpointOwner(tmp_path/'session',displacement)
    with SessionTransport(owner.config,Limits(command_timeout_s=.3),True,owner.clock,owner.sleep) as transport:
        with pytest.raises(Refused):transport.move('right_arm_elbow_flex',68,reviewed_settle_ticks=24)


def test_other_joint_cannot_select24_and_emits_no_command(tmp_path):
    owner=EndpointOwner(tmp_path/'session',47)
    with SessionTransport(owner.config,Limits(),True,owner.clock,owner.sleep) as transport:
        with pytest.raises(Refused):transport.move('right_arm_shoulder_pan',68,reviewed_settle_ticks=24)
        assert not (owner.folder/'command.json').exists()


def test_default_elbow_settling_is_still_strict(tmp_path):
    owner=EndpointOwner(tmp_path/'session',47)
    with SessionTransport(owner.config,Limits(command_timeout_s=.3),True,owner.clock,owner.sleep) as transport:
        with pytest.raises(Refused):transport.move('right_arm_elbow_flex',68)


def test_endpoints_inside24_still_require_stable_actual_position(tmp_path):
    owner=EndpointOwner(tmp_path/'session',47)
    sleep=owner.sleep
    def oscillating(seconds):
        was_commanded=owner.seen is not None
        sleep(seconds)
        if was_commanded:
            joint='right_arm_elbow_flex'
            owner.q[joint]=2057 if owner.q[joint]==2047 else 2047
            owner.publish()
    with SessionTransport(owner.config,Limits(command_timeout_s=.3),True,owner.clock,oscillating) as transport:
        with pytest.raises(Refused):transport.move('right_arm_elbow_flex',68,reviewed_settle_ticks=24)
