"""Emoji show: gesture plans, the performance sequence against a fake owner, the queue and the web routes."""
import json
import threading
import time
import urllib.error
import urllib.request

import pytest

from robot_emoji import gestures as G
from robot_emoji import server as S
from robot_emoji.performer import Busy, PerformError, Performer
from robot_emoji.robot import FakeRobot

RIGHT = G.arm_motors('right')
START = {'right_arm_shoulder_pan': 1930, 'right_arm_shoulder_lift': 2375, 'right_arm_elbow_flex': 1350,
         'right_arm_wrist_flex': 3000, 'right_arm_wrist_roll': 1949, 'right_arm_gripper': 1413}
# Live commandable ranges read on 8 October 2026 (40-tick margin).
RANGES = {'right_arm_shoulder_pan': (713, 3381), 'right_arm_shoulder_lift': (864, 3230), 'right_arm_elbow_flex': (972, 3122),
          'right_arm_wrist_flex': (964, 3130), 'right_arm_wrist_roll': (170, 3924), 'right_arm_gripper': (1309, 2785)}
RANGES = {n: {'min_ticks': lo, 'max_ticks': hi, 'margin_ticks': 40} for n, (lo, hi) in RANGES.items()}


def fake(**kw):
    positions = {f'left_arm_{j}': 2047 for j in G.ARM_JOINTS}
    positions.update(START)
    ranges = {f'left_arm_{j}': {'min_ticks': 900, 'max_ticks': 3200} for j in G.ARM_JOINTS}
    ranges.update(RANGES)
    return FakeRobot(positions, ranges, time_scale=0, **kw)


def names(robot):
    return [c[0] for c in robot.calls]


def test_default_wave_loads_and_fits_live_ranges():
    catalog = G.load()
    wave = catalog['wave']
    assert wave.emoji == '👋' and wave.arm == 'right' and wave.verified_on_hardware is False
    plan = G.plan([wave], START, RANGES)
    assert [p['part'] for p in plan['paths']] == ['raise', 'motion', 'return']
    assert plan['motors'] == RIGHT
    # the gesture never touches the gripper or the wrist roll; home is where the arm rested
    assert plan['home'] == {n: START[n] for n in ('right_arm_elbow_flex', 'right_arm_shoulder_lift', 'right_arm_shoulder_pan', 'right_arm_wrist_flex')}
    for p in plan['paths']:
        assert 0 < p['duration_s'] <= G.MAX_PATH_S and len(p['waypoints']) <= G.MAX_PATH_WAYPOINTS


def test_motion_legs_must_be_one_owner_piece():
    data = {'g': {'emoji': 'x', 'label': 'x', 'arm': 'right', 'raise': [{'wrist_flex': 1600}],
                  'motion': [{'wrist_flex': 1600 + G.MAX_LEG_TICKS + 1}]}}
    with pytest.raises(G.GestureError, match='exceeds'):
        G.parse(data)
    data['g']['motion'] = [{'elbow_flex': 1500}]
    with pytest.raises(G.GestureError, match='not set by the raise'):
        G.parse(data)
    data['g']['motion'] = [{'gripper': 1500}]
    with pytest.raises(G.GestureError, match='excluded'):
        G.parse(data)


def test_targets_outside_live_range_refused_but_home_is_clamped():
    wave = G.load()['wave']
    narrow = json.loads(json.dumps(RANGES))
    narrow['right_arm_elbow_flex']['min_ticks'] = 1200
    with pytest.raises(G.GestureError, match='right_arm_elbow_flex=1150'):
        G.plan([wave], START, narrow)
    sagged = dict(START, right_arm_shoulder_lift=3250)       # released joint resting past the commandable max
    assert G.plan([wave], sagged, RANGES)['home']['right_arm_shoulder_lift'] == 3230


def test_path_seconds_follow_owner_pace():
    start = {'a': 0}
    assert G.path_seconds([{'a': 340}], start) == 3.6            # 9 steps of 40 ticks, 0.4 s each
    assert G.path_seconds([{'a': 1400}], start) == 14.0          # split into 5 pieces of 280 = 7 steps each


def test_performance_sequence_and_release():
    robot = fake()
    phases = []
    summary = Performer(robot, G.load(), log=lambda m: None).perform(['wave'], lambda p, g=None: phases.append(p))
    assert names(robot) == ['robot_get_motion', 'robot_get_state', 'robot_set_motor_enable',
                            'robot_move_path', 'robot_move_path', 'robot_move_path', 'robot_set_motor_enable']
    enable, release = robot.calls[2][1], robot.calls[-1][1]
    assert enable == {'names': RIGHT, 'enabled': True} and release == {'names': RIGHT, 'enabled': False}
    assert all(c[1]['arm'] == 'right' and c[1]['wait'] is True for c in robot.calls if c[0] == 'robot_move_path')
    assert phases == ['enable', 'raise', 'motion', 'return', 'release']
    assert robot.positions['right_arm_wrist_flex'] == 3000 and not robot.enabled
    assert [p['completed'] for p in summary['paths']] == [True, True, True]


def test_busy_robot_gets_no_writes():
    robot = fake(busy_motors=['left_arm_shoulder_pan'])
    with pytest.raises(Busy):
        Performer(robot, G.load(), log=lambda m: None).perform(['wave'])
    assert names(robot) == ['robot_get_motion']


def test_enable_refused_moves_nothing():
    robot = fake()
    robot.fail['robot_set_motor_enable'] = 'Pickup phone feed paused; motor activation refused'
    with pytest.raises(PerformError, match='nothing moved'):
        Performer(robot, G.load(), log=lambda m: None).perform(['wave'])
    assert 'robot_move_path' not in names(robot)


def test_refused_move_while_holding_goes_home_and_releases():
    robot = fake()
    robot.fail['robot_move_path'] = 'Movement is in progress'
    with pytest.raises(PerformError):
        Performer(robot, G.load(), log=lambda m: None).perform(['wave'])
    tail = robot.calls[-2:]
    assert tail[0][0] == 'robot_move_path' and tail[0][1]['waypoints'][0]['right_arm_wrist_flex'] == 3000
    assert tail[1] == ('robot_set_motor_enable', {'names': RIGHT, 'enabled': False})
    assert 'robot_stop' not in names(robot)


def test_owner_fault_sends_nothing_more():
    robot = fake()
    robot.fail['robot_move_path'] = 'Owner stopped: Following error'
    with pytest.raises(PerformError):
        Performer(robot, G.load(), log=lambda m: None).perform(['wave'])
    assert names(robot)[-2:] == ['robot_move_path', 'robot_get_motion']


def test_operator_stop_mid_move():
    robot = FakeRobot(dict(START, **{f'left_arm_{j}': 2047 for j in G.ARM_JOINTS}), {**RANGES, **{f'left_arm_{j}': {'min_ticks': 900, 'max_ticks': 3200} for j in G.ARM_JOINTS}}, time_scale=1)
    performer = Performer(robot, G.load(), log=lambda m: None)
    errors = []
    t = threading.Thread(target=lambda: errors.append(pytest.raises(PerformError, performer.perform, ['wave'])))
    t.start()
    while 'robot_move_path' not in names(robot):
        time.sleep(.01)
    performer.stop()
    t.join(5)
    assert not t.is_alive() and 'Stopped by the operator' in str(errors[0].value)
    assert names(robot)[-1] == 'robot_stop' and names(robot).count('robot_move_path') == 1


def test_clean_name():
    assert S.clean_name('  Ada \n  Lovelace ') == 'Ada Lovelace'
    assert S.clean_name('Zo​e') == 'Zoe'
    with pytest.raises(ValueError):
        S.clean_name('   ')
    with pytest.raises(ValueError):
        S.clean_name('x' * 25)


@pytest.fixture
def web(monkeypatch):
    monkeypatch.setattr(S, 'THANKS_S', 0)
    robot = fake()
    show = S.Show(robot, G.load())
    show.start()
    server = S.serve(show, '127.0.0.1', 0, 'secret-token')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_address[1]}'
    yield show, robot, base
    show.close()
    server.shutdown()


def post(url, body, headers=None):
    req = urllib.request.Request(url, json.dumps(body).encode(), {'Content-Type': 'application/json', **(headers or {})})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def test_web_queue_waits_for_arm_then_performs(web):
    show, robot, base = web
    status, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    assert status == 201 and ticket['position'] == 1 and ticket['emojis'] == ['👋']
    assert post(base + '/api/requests', {'name': 'Bob', 'gestures': ['dance']})[0] == 400
    time.sleep(.3)
    assert robot.calls == []                                   # starts paused: nothing touches the robot
    assert get(base + '/api/state')['queue'][0]['name'] == 'Ada'
    assert post(base + '/api/operator/arm', {'armed': True})[0] == 200   # loopback counts as the operator
    deadline = time.time() + 5
    while get(base + f"/api/requests/{ticket['id']}")['state'] != 'done' and time.time() < deadline:
        time.sleep(.05)
    assert get(base + f"/api/requests/{ticket['id']}")['state'] == 'done'
    assert names(robot).count('robot_move_path') == 3 and not robot.enabled


def test_web_failure_pauses_the_show(web):
    show, robot, base = web
    robot.fail['robot_set_motor_enable'] = 'Pickup phone feed paused'
    show.set_armed(True)
    _, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    deadline = time.time() + 5
    while get(base + f"/api/requests/{ticket['id']}")['state'] != 'failed' and time.time() < deadline:
        time.sleep(.05)
    assert get(base + f"/api/requests/{ticket['id']}")['state'] == 'failed'
    assert get(base + '/api/state')['armed'] is False


def test_operator_routes_need_token_off_loopback(web):
    show, robot, base = web
    handler = S.make_handler(show, 'secret-token')
    h = handler.__new__(handler)
    h.client_address = ('192.168.1.20', 5000)
    h.headers = {}
    assert not h.operator_ok({})
    assert not h.operator_ok({'token': 'wrong'})
    assert h.operator_ok({'token': 'secret-token'})
    h.headers = {'X-Operator-Token': 'secret-token'}
    assert h.operator_ok({})
