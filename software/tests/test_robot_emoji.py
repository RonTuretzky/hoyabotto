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
from robot_emoji.robot import FakeRobot, RobotTransportError

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
    positions.update(head_motor_1=2085, head_motor_2=2600)
    ranges = {f'left_arm_{j}': {'min_ticks': 900, 'max_ticks': 3200} for j in G.ARM_JOINTS}
    ranges.update(RANGES)
    ranges.update(head_motor_1={'min_ticks':1059,'max_ticks':3111},head_motor_2={'min_ticks':1972,'max_ticks':2625})
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
    # The gesture never touches the gripper or wrist roll; home is where the arm rested.
    assert plan['home'] == {n: START[n] for n in ('right_arm_elbow_flex', 'right_arm_shoulder_lift', 'right_arm_shoulder_pan', 'right_arm_wrist_flex')}
    for p in plan['paths']:
        assert 0 < p['duration_s'] <= G.MAX_PATH_S and len(p['waypoints']) <= G.MAX_PATH_WAYPOINTS


HEAD_LOOK = {'_speed_profile': 'demo', 'look': {'emoji': '👀', 'label': 'look', 'arm': 'head', 'relative_joints': ['pan'],
             'raise': [{'pan': 0}], 'motion': [{'pan': -80}, {'pan': 80}] * 5 + [{'pan': 0}]}}


def test_six_distinct_arm_gestures():
    catalog = G.load()
    assert list(catalog) == ['wave', 'up', 'flex', 'shake', 'point', 'circle']
    assert len({g.emoji for g in catalog.values()}) == 6
    # right arm only: the head stays at its registered pose
    assert {g.arm for g in catalog.values()} == {'right'}
    # normal arm speed: the first demo-speed enable on the real robot was followed by an owner fault (10 Oct)
    assert all(g.speed_profile == 'normal' and g.verified_on_hardware is False for g in catalog.values())
    # each gesture has its own motion signature: which joints move during the motion, and how far
    signatures = set()
    for g in catalog.values():
        pose = {}
        for w in g.raise_path:
            pose.update(w)
        travel = {}
        for w in g.motion:
            for j, q in w.items():
                travel[j] = travel.get(j, 0) + abs(q - pose[j])
            pose.update(w)
        assert max(travel.values()) >= 150, g.key            # visibly large, not a jiggle
        signatures.add(tuple(sorted(j for j, t in travel.items() if t >= 150)))
    assert len(signatures) >= 5
    for g in catalog.values():
        plan = G.plan([g], START, RANGES)
        assert plan['speed_profile'] == 'normal'
        assert all('right_arm_gripper' not in w for p in plan['paths'] for w in p['waypoints'])
        assert sum(p['duration_s'] for p in plan['paths']) < 45


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
    narrow['right_arm_elbow_flex']['min_ticks'] = 2200
    with pytest.raises(G.GestureError, match='right_arm_elbow_flex=2092'):
        G.plan([wave], START, narrow)
    sagged = dict(START, right_arm_shoulder_lift=3250)       # released joint resting past the commandable max
    assert G.plan([wave], sagged, RANGES)['home']['right_arm_shoulder_lift'] == 3230


def test_retired_preset_history_survives_catalog_removal(tmp_path):
    path = tmp_path / 'queue.json'
    data = json.loads(G.DEFAULT_PATH.read_text())
    data['full_wave'] = data['wave']
    old = S.Show(fake(), G.parse(data), state_path=path)
    done = old.submit('Previous turn', ['full_wave'])
    request = old.queue.popleft()
    request['state'] = 'done'
    request.pop('emojis')  # Tickets from before emoji snapshots were persisted.
    old.recent.appendleft(request)
    pending = old.submit('Old selection', ['full_wave'])
    mixed = old.submit('Short and long', ['wave'])
    old.requests[mixed['id']].update(gestures=['wave','full_wave'], emojis=['👋','👋'])
    old._persist()
    robot = fake()
    restored = S.Show(robot, G.load(), state_path=path)
    assert restored.request_status(done['id'])['emojis'] == ['👋']
    assert restored.snapshot()['recent'][0]['state'] == 'done'
    assert restored.request_status(pending['id'])['state'] == 'removed'
    assert restored.request_status(mixed['id'])['emojis'] == ['👋']
    assert restored.queue[0]['gestures'] == ['wave'] and len(restored.queue) == 1
    assert not robot.calls


@pytest.mark.parametrize('key', ['wave', 'up', 'flex', 'shake', 'point', 'circle'])
def test_new_arm_gestures_return_and_release(key):
    robot = fake()
    before = dict(robot.positions)
    result = Performer(robot, G.load(), log=lambda _:None).perform([key])
    assert all(p['completed'] for p in result['paths'])
    assert robot.speed_profile == 'normal'
    assert not robot.enabled
    assert robot.positions == before


def test_look_around_uses_only_bounded_head_moves_and_restores_pan():
    robot = fake()
    before = dict(robot.positions)
    result = Performer(robot, G.parse(HEAD_LOOK), log=lambda _:None).perform(['look'])
    assert result['arm'] == 'head'
    assert 'robot_move_path' not in names(robot)
    moves = [args for name,args in robot.calls if name == 'robot_move_head']
    assert len(moves) == 11
    current = before['head_motor_1']
    for args in moves:
        assert list(args['positions']) == ['head_motor_1']
        target = args['positions']['head_motor_1']
        assert abs(target-current) <= 200 and args['duration_s'] >= abs(target-current)/100
        current = target
    assert robot.positions == before and not robot.enabled
    enables = [args for name,args in robot.calls if name=='robot_set_motor_enable']
    assert all(args['names']==['head_motor_1'] for args in enables)


def test_head_range_or_step_violation_is_rejected_before_enabling():
    robot = fake()
    robot.positions['head_motor_1'] = 3100
    with pytest.raises(PerformError, match='outside commandable'):
        Performer(robot,G.parse(HEAD_LOOK),log=lambda _:None).perform(['look'])
    assert all(name.startswith('robot_get_') for name in names(robot))
    data = json.loads(json.dumps(HEAD_LOOK))
    data['look']['motion'] = [{'pan':-110},{'pan':110}]
    with pytest.raises(G.GestureError, match='exceeds 200'):
        G.parse(data)


def test_default_fake_robot_can_run_the_head_preset():
    robot = FakeRobot(time_scale=0)
    before = dict(robot.positions)
    result = Performer(robot,G.parse(HEAD_LOOK),log=lambda _:None).perform(['look'])
    assert result['arm']=='head' and robot.positions==before and not robot.enabled
    # the head never takes the arm demo profile
    assert all('speed_profile' not in args for name, args in robot.calls if name == 'robot_set_motor_enable')


def test_path_seconds_follow_owner_pace():
    start = {'a': 0}
    assert G.path_seconds([{'a': 340}], start) == 3.6            # 9 steps of 40 ticks, 0.4 s each
    assert G.path_seconds([{'a': 1400}], start) == 14.0          # split into 5 pieces of 280 = 7 steps each
    assert G.path_seconds([{'a': 340}], start, G.RATES['demo']) == 1.2   # same 9 steps at 300 ticks/s


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


def stale_base_robot(*, moving=False, torque=0, delta=0, cached=False):
    robot = fake()
    original = robot.call
    samples = 0
    def call(name, arguments=None, timeout=30):
        nonlocal samples
        result = original(name, arguments, timeout)
        if name == 'robot_get_motion':
            result.update(phase='moving' if moving else 'idle', moving=moving, base_drive_phase='braking')
        elif name == 'robot_get_state':
            result.update(cached=cached, all_16_released=torque == 0)
            result['motors'].extend({'name':n,'Torque_Enable':torque,'Status':0,'Present_Position':100 + samples * delta}
                                   for n in ('base_left_wheel','base_right_wheel'))
            samples += 1
        return result
    robot.call = call
    return robot


def test_inactive_last_base_phase_requires_two_fresh_released_stationary_samples():
    robot = stale_base_robot()
    assert Performer(robot, G.load(), log=lambda _:None).preflight(['wave'])['arm'] == 'right'
    assert names(robot) == ['robot_get_motion','robot_get_state','robot_get_state']
    assert robot.calls[1][1] == robot.calls[2][1] == {'fresh':True}


@pytest.mark.parametrize('condition', [{'moving':True}, {'torque':1}, {'delta':5}, {'cached':True}])
def test_base_braking_still_blocks_active_powered_rolling_or_stale_state(condition):
    robot = stale_base_robot(**condition)
    with pytest.raises(Busy):
        Performer(robot, G.load(), log=lambda _:None).preflight(['wave'])
    assert all(name.startswith('robot_get_') for name in names(robot))


def test_enable_refused_moves_nothing():
    robot = fake()
    robot.fail['robot_set_motor_enable'] = 'Pickup phone feed paused; motor activation refused'
    with pytest.raises(PerformError, match='nothing moved'):
        Performer(robot, G.load(), log=lambda m: None).perform(['wave'])
    assert 'robot_move_path' not in names(robot)


def test_uncertain_enable_is_recovered_without_replaying_enable():
    robot = fake()
    original = robot.call
    def call(name, arguments=None, timeout=30):
        result = original(name,arguments,timeout)
        if name=='robot_set_motor_enable' and arguments['enabled']:
            raise RobotTransportError('Enable acknowledgement lost')
        return result
    robot.call = call
    with pytest.raises(PerformError,match='outcome was unknown'):
        Performer(robot,G.load(),log=lambda _:None).perform(['wave'])
    assert len([args for name,args in robot.calls if name=='robot_set_motor_enable' and args['enabled']])==1
    assert not robot.enabled
    assert names(robot).count('robot_move_path')==1  # Recovery home only, never raise/gesture.


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


def test_stop_is_not_cleared_by_starting_another_performance():
    robot = fake()
    show = S.Show(robot,G.load())
    show.stop()
    count = len(robot.calls)
    with pytest.raises(PerformError,match='Stopped by the operator'):
        show.performer.perform(['wave'])
    assert len(robot.calls)==count
    show.set_armed(True)
    assert not show.performer.aborted.is_set()


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


def test_visitors_can_submit_only_one_emoji(web):
    show, robot, base = web
    assert post(base + '/api/requests', {'name':'Ada','gestures':['wave','wiggle']})[0] == 400
    assert not show.queue and not robot.calls


LEFT = G.arm_motors('left')


def wait_state(base, ticket, states, seconds=5):
    deadline = time.time() + seconds
    while get(base + f"/api/requests/{ticket['id']}")['state'] not in states and time.time() < deadline:
        time.sleep(.05)
    return get(base + f"/api/requests/{ticket['id']}")['state']


def test_left_arm_mirrors_the_right_arm_gestures():
    robot = fake()
    for g in G.load().values():
        plan = G.plan([g], robot.positions, robot.ranges, arm='left')
        assert plan['arm'] == 'left' and plan['motors'] == LEFT
        right = G.plan([g], robot.positions, robot.ranges)
        for pl, pr in zip(plan['paths'][:-1], right['paths'][:-1]):
            for wl, wr in zip(pl['waypoints'], pr['waypoints']):
                for jr, q in wr.items():
                    j = jr[len('right_arm_'):]
                    if j in g.relative_joints:
                        continue
                    mid_r = sum(robot.ranges[jr][k] for k in ('min_ticks', 'max_ticks')) / 2
                    mid_l = sum(robot.ranges[f'left_arm_{j}'][k] for k in ('min_ticks', 'max_ticks')) / 2
                    sign = -1 if j == 'shoulder_pan' else 1
                    assert abs(wl[f'left_arm_{j}'] - (mid_l + sign * (q - mid_r))) <= 0.5


def test_right_arm_fault_reported_by_the_robot_uses_the_left_arm(web):
    show, robot, base = web
    robot.status['right_arm_elbow_flex'] = 32
    show.set_armed(True)
    _, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    assert wait_state(base, ticket, ('done', 'failed')) == 'done'
    enables = [a for n, a in robot.calls if n == 'robot_set_motor_enable' and a['enabled']]
    assert enables and all(a['names'] == LEFT for a in enables)
    assert get(base + '/api/state')['armed'] is True


def test_right_arm_refused_before_moving_falls_back_to_left(web):
    show, robot, base = web
    robot.fail['robot_set_motor_enable'] = 'right_arm_shoulder_pan: fault or health limit'
    show.set_armed(True)
    _, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['flex']})
    assert wait_state(base, ticket, ('done', 'failed')) == 'done'
    enables = [a['names'] for n, a in robot.calls if n == 'robot_set_motor_enable' and a['enabled']]
    assert enables == [RIGHT, LEFT]
    assert get(base + '/api/state')['armed'] is True and not robot.enabled


def test_three_refusals_restart_the_robot_then_the_request_runs(web, monkeypatch):
    monkeypatch.setattr(S, 'RETRY_S', .01)
    show, robot, base = web
    robot.fail_always['robot_set_motor_enable'] = 'HARDWARE_OWNER_UNAVAILABLE'
    show.set_armed(True)
    _, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    assert wait_state(base, ticket, ('done', 'failed')) == 'done'
    assert [n for n, _ in robot.calls].count('restart_owner') == 1
    assert get(base + '/api/state')['armed'] is True


def test_restart_that_does_not_help_pauses_the_show(web, monkeypatch):
    monkeypatch.setattr(S, 'RETRY_S', .01)
    show, robot, base = web
    robot.restart_result = False
    robot.fail_always['robot_set_motor_enable'] = 'HARDWARE_OWNER_UNAVAILABLE'
    show.set_armed(True)
    _, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    assert wait_state(base, ticket, ('done', 'failed')) == 'failed'
    assert get(base + '/api/state')['armed'] is False
    assert [n for n, _ in robot.calls].count('restart_owner') == 1


def test_mid_motion_failure_is_reported_but_the_show_keeps_going(web, monkeypatch):
    monkeypatch.setattr(S, 'RETRY_S', .01)
    show, robot, base = web
    robot.fail['robot_move_path'] = 'Owner stopped: contact guard'
    show.set_armed(True)
    _, first = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']})
    assert wait_state(base, first, ('done', 'failed')) == 'failed'
    _, second = post(base + '/api/requests', {'name': 'Bo', 'gestures': ['point']})
    assert wait_state(base, second, ('done', 'failed')) == 'done'
    assert get(base + '/api/state')['armed'] is True


def test_operator_stop_still_pauses(web):
    show, robot, base = web
    show.stop()
    assert get(base + '/api/state')['armed'] is False


def test_read_only_transport_outage_keeps_ticket_queued_then_recovers(monkeypatch):
    monkeypatch.setattr(S,'BUSY_RETRY_S',.02)
    monkeypatch.setattr(S,'THANKS_S',0)
    robot = fake()
    original = robot.call
    outage = threading.Event()
    outage.set()
    def call(name, arguments=None, timeout=30):
        if name=='robot_get_motion' and outage.is_set():
            raise RobotTransportError('Read-only connection unavailable')
        return original(name,arguments,timeout)
    robot.call = call
    show = S.Show(robot,G.load(),armed=True)
    ticket = show.submit('Ada',['wave'])
    show.start()
    deadline = time.time()+2
    while not show.robot_note.startswith('waiting:') and time.time()<deadline:
        time.sleep(.01)
    assert show.request_status(ticket['id'])['state']=='queued' and show.armed
    assert show.request_status(ticket['id'])['waiting_reason']
    assert not robot.calls
    outage.clear()
    deadline = time.time()+2
    while show.request_status(ticket['id'])['state']!='done' and time.time()<deadline:
        time.sleep(.01)
    show.close()
    assert show.request_status(ticket['id'])['state']=='done'
    assert names(robot).count('robot_move_path')==3


def test_second_read_only_preflight_failure_does_not_fail_or_enable(monkeypatch):
    monkeypatch.setattr(S,'BUSY_RETRY_S',.02)
    monkeypatch.setattr(S,'THANKS_S',0)
    robot = fake()
    original = robot.call
    outage = threading.Event();outage.set()
    reads = 0
    def call(name, arguments=None, timeout=30):
        nonlocal reads
        if name=='robot_get_motion':
            reads += 1
            if reads>=2 and outage.is_set():
                raise RobotTransportError('Second preflight read unavailable')
        return original(name,arguments,timeout)
    robot.call = call
    show = S.Show(robot,G.load(),armed=True)
    ticket = show.submit('Ada',['wave']);show.start()
    deadline=time.time()+2
    while not show.robot_note.startswith('waiting:') and time.time()<deadline:time.sleep(.01)
    assert show.request_status(ticket['id'])['state']=='queued' and show.armed
    assert all(name.startswith('robot_get_') for name in names(robot))
    outage.clear()
    deadline=time.time()+2
    while show.request_status(ticket['id'])['state']!='done' and time.time()<deadline:time.sleep(.01)
    show.close()
    assert show.request_status(ticket['id'])['state']=='done'
    assert names(robot).count('robot_move_path')==3


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


@pytest.fixture
def public_web():
    robot = fake()
    show = S.Show(robot, G.load())
    show.start()
    server = S.serve(show, '127.0.0.1', 0, 'secret-token', public_only=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield show, robot, f'http://127.0.0.1:{server.server_port}'
    show.close()
    server.shutdown()
    server.server_close()


def test_public_listener_never_authorizes_operator(public_web):
    show, robot, base = public_web
    for action in ('arm', 'stop', 'remove'):
        assert post(base + '/api/operator/' + action, {'armed': True}, {'X-Operator-Token': 'secret-token'})[0] == 403
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(base + '/operator?token=secret-token')
    assert err.value.code == 403
    state = get(base + '/api/state?token=secret-token')
    assert 'log' not in state and 'robot_api' not in state
    assert show.armed is False and robot.calls == []


def test_public_queue_and_cors(public_web):
    show, robot, base = public_web
    headers = {'Origin': 'https://hoyabotto.com', 'CF-Connecting-IP': '203.0.113.1'}
    req = urllib.request.Request(base + '/api/requests', method='OPTIONS', headers={
        'Origin': headers['Origin'], 'Access-Control-Request-Method': 'POST',
        'Access-Control-Request-Headers': 'content-type'})
    with urllib.request.urlopen(req) as response:
        assert response.headers['Access-Control-Allow-Origin'] == headers['Origin']
        assert 'POST' in response.headers['Access-Control-Allow-Methods']
        assert 'Access-Control-Allow-Credentials' not in response.headers
    status, ticket = post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']}, headers)
    assert status == 201 and get(base + '/api/requests/' + ticket['id'])['state'] == 'queued'
    assert post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']}, headers)[0] == 429
    assert post(base + '/api/requests', {'name': 'Bob', 'gestures': ['wave']}, {
        **headers, 'CF-Connecting-IP': '203.0.113.2'})[0] == 201
    assert len(show.queue) == 2 and robot.calls == []


def test_public_rejects_unrelated_origin(public_web):
    show, robot, base = public_web
    assert post(base + '/api/requests', {'name': 'Ada', 'gestures': ['wave']}, {'Origin': 'https://example.com'})[0] == 403
    assert not show.queue and robot.calls == []


def test_private_listener_rejects_tunnel_loopback_privilege(web):
    show, robot, base = web
    assert post(base + '/api/operator/arm', {'armed': True}, {'CF-Connecting-IP': '203.0.113.1'})[0] == 403
    assert post(base + '/api/operator/arm', {'armed': True}, {'Origin': 'https://hoyabotto.com'})[0] == 403
    assert post(base + '/api/operator/arm', {'armed': True}, {'Origin': base})[0] == 200


def test_queue_survives_restart_without_replaying_interrupted_motion(tmp_path):
    path = tmp_path / 'queue.json'
    first = S.Show(fake(), G.load(), state_path=path)
    ticket = first.submit('Ada', ['wave'])
    next_ticket = first.submit('Bob', ['wave'])
    # Model a process loss AFTER dispatch was recorded, including an uncertain motor outcome.
    with first.lock:
        req = first.queue.popleft()
        req['state'] = 'performing'
        first.current = req
        first._persist()
    robot = fake()
    restored = S.Show(robot, G.load(), armed=True, state_path=path)
    assert restored.armed is False
    assert restored.request_status(ticket['id'])['state'] == 'failed'
    assert [r['id'] for r in restored.queue] == [next_ticket['id']]
    assert robot.calls == []
    assert restored.remove(next_ticket['id'])
    assert not S.Show(fake(), G.load(), state_path=path).queue


def test_proxy_visitor_identity_requires_authentication(public_web):
    show, robot, _ = public_web
    server = S.serve(show, '127.0.0.1', 0, 'secret-token', public_only=True, proxy_token='cloud-secret')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_port}'
    headers = {'CF-Connecting-IP':'198.51.100.1', 'X-Hoya-Visitor-IP':'203.0.113.1', 'X-Hoya-Proxy-Token':'wrong'}
    try:
        assert post(base + '/api/requests', {'name':'Ada','gestures':['wave']}, headers)[0] == 201
        assert post(base + '/api/requests', {'name':'Bob','gestures':['wave']}, {**headers,'X-Hoya-Visitor-IP':'203.0.113.2'})[0] == 429
        assert post(base + '/api/requests', {'name':'Bob','gestures':['wave']}, {**headers,'X-Hoya-Visitor-IP':'203.0.113.2','X-Hoya-Proxy-Token':'cloud-secret'})[0] == 201
        assert not show.armed and not robot.calls
    finally:
        server.shutdown()
        server.server_close()


def test_paired_client_adapter_does_not_retry_unknown_command_outcomes(monkeypatch):
    import sys
    from types import SimpleNamespace
    from robot_emoji.robot import RobotClient, RobotError
    class PairedRobot:
        def __init__(self, path):
            self.link = 'relay'
            self.calls = []
            self.fail = False
        def lan_reachable(self, url):
            return True
        def call(self, name, arguments, request_id=None):
            self.calls.append((name, arguments, request_id))
            if self.fail:
                raise OSError('Transport outcome unknown')
            return {'ok':True, 'result':{'source':'paired-client'}}
    monkeypatch.setattr(sys, 'path', list(sys.path))
    monkeypatch.setitem(sys.modules, 'chat_server', SimpleNamespace(Robot=PairedRobot))
    robot = RobotClient(prefer_lan=False)
    assert not robot.client.lan_reachable('https://example.local')
    assert robot.call('robot_get_state', {'fresh':False})['source'] == 'paired-client'
    robot.client.fail = True
    with pytest.raises(RobotError, match='unreachable'):
        robot.call('robot_move_path', {'arm':'right'})
    assert len(robot.client.calls) == 2  # One dispatch per invocation, even after a network failure.
