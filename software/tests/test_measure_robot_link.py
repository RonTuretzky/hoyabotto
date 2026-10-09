"""tools/measure_robot_link.py on a fake robot API (read-only; no network)."""
from tools import measure_robot_link as M


class FakeRobot:
    def __init__(self, poll_s):
        self.t, self.poll_s = 1000.0, poll_s

    def call(self, name, args):
        assert name == 'robot_get_execution'
        self.t += 0.1
        rows = {f'm{i}': {'captured_at': self.t - self.poll_s * (1 - i / 11)} for i in range(12)}
        return {'ok': True, 'result': {'time': self.t, 'status_age_s': 0.0, 'rows': rows}}


def run(poll_s):
    robot = FakeRobot(poll_s)
    samples = M.measure(robot, count=5, clock=lambda: robot.t, sleep=lambda s: None)
    return M.summarize(samples)['owner_loop_period_estimate_s']


def test_owner_loop_period_is_poll_spread_plus_the_owner_sleep():
    fast, slow = run(0.010), run(0.030)
    assert abs(fast['median'] - 0.030) < 1e-6 and fast['fold_policy_stream_ok'] is True
    assert abs(slow['median'] - 0.050) < 1e-6 and slow['fold_policy_stream_ok'] is False
