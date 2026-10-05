import numpy as np
import pytest

from carton.servo.common import Refused
from carton.servo.trajectory import JointTrajectory


def trajectory(points, times=None):
    return JointTrajectory([{"time_s": t, "positions": {"joint": p}} for t, p in
                            zip(times or range(len(points)), points)], ["joint"], {"joint": [1000, 3000]},
                           {"joint": 180}, {"joint": 360})


def test_forty_degree_path_is_one_continuous_action_without_six_degree_stops():
    t = trajectory([2000, 2114, 2228, 2342, 2456])
    assert t.q[-1, 0]-t.q[0, 0] == 456  # about 40 degrees, not capped at 68 ticks
    assert all(t.sample(stamp)[1]["joint"] > 0 for stamp in t.times[1:-1])
    assert t.sample(0)[1]["joint"] == pytest.approx(0)
    assert t.sample(t.duration)[1]["joint"] == pytest.approx(0)
    assert t.duration < 6


@pytest.mark.parametrize("points", [[2000, 2456], [2000, 2010, 2400, 2401], [2400, 2200, 2000],
                                     [2000, 2300, 2100, 2500], [2000, 2000, 2100]])
def test_limits_hold_between_knots_without_position_overshoot(points):
    t = trajectory(points)
    samples = [t.sample(s) for s in np.linspace(0, t.duration, 2001)]
    assert max(abs(v["joint"]) for _, v, _ in samples) <= 180.000001
    assert max(abs(a["joint"]) for _, _, a in samples) <= 360.000001
    for stamp in np.linspace(0, t.duration, 2001):
        i = min(np.searchsorted(t.times, stamp, side="right")-1, len(points)-2)
        q = t.sample(stamp)[0]["joint"]
        assert min(points[i:i+2])-.000001 <= q <= max(points[i:i+2])+.000001


def test_reversal_stops_but_same_direction_knot_does_not():
    t = trajectory([2000, 2100, 2200, 2100])
    assert t.sample(t.times[1])[1]["joint"] > 0
    assert t.sample(t.times[2])[1]["joint"] == pytest.approx(0)


def test_bad_start_and_tracking_error_refuse():
    t = trajectory([2000, 2400])
    with pytest.raises(Refused, match="start"):
        t.assert_start({"joint": 1900})
    with pytest.raises(Refused, match="tracking"):
        t.check_following({"joint": 2000}, t.duration, 24)


@pytest.mark.parametrize("points,times", [([2000, 3100], None), ([2000, float("nan")], None),
                                         ([2000, 2100], [1, 2]), ([2000, 2100], [0, 0]),
                                         ([2000, 2100], [0, 60])])
def test_invalid_whole_path_refuses_before_dispatch(points, times):
    with pytest.raises(Refused):
        trajectory(points, times)
