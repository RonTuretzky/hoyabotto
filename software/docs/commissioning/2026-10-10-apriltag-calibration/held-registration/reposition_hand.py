"""Raise the right hand so gripper tag 2 sits inside the OAK view (for tag registration).

Uses the pilot's ordinary motion tools (same path the LLM pilot uses): enable the six
right-arm motors, robot_move_joint_targets on the elbow in the observed lifting direction
(decreasing ticks), check tag 2 in the OAK frame between stages, then release.
Records oak/phone/right_wrist through the chat recorder for the whole attempt.
"""
import sys, json, time, base64, urllib.request
from pathlib import Path

ROOT = Path('/Users/wk/conductor/workspaces/xlerobot-farm/seville-v2')
PILOT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot')
sys.path[:0] = [str(ROOT/'software'), str(PILOT)]
from chat_server import Robot
from farm.perception.gemma_tags import TagRobot

OUT = ROOT/'.context/tag-registration-20261010'/sys.argv[1]
OUT.mkdir(parents=True, exist_ok=False)
STEPS = [int(x) for x in sys.argv[2].split(',')]        # elbow deltas, e.g. -150,-150
JOINT = 'right_arm_elbow_flex'
SIX = ['right_arm_shoulder_pan', 'right_arm_shoulder_lift', 'right_arm_elbow_flex',
       'right_arm_wrist_flex', 'right_arm_wrist_roll', 'right_arm_gripper']
OWNER_STARTED = 1791551739.406972
ORIGIN = 'http://127.0.0.1:1241'
LOG = (OUT/'calls.jsonl').open('a')


def log(kind, **row):
    row.update(kind=kind, t=time.time())
    LOG.write(json.dumps(row)+'\n'); LOG.flush()
    return row


def save(name, value):
    (OUT/name).write_text(json.dumps(value, indent=1)+'\n')
    return value


def api(path, body=None, csrf=[None]):
    headers = {'Content-Type': 'application/json', 'Origin': ORIGIN}
    if body is not None:
        if csrf[0] is None:
            req = urllib.request.Request(ORIGIN+'/api/session', headers={'Origin': ORIGIN, 'X-Chat-Session': 'refresh'})
            with urllib.request.urlopen(req, timeout=5) as r:
                csrf[0] = json.load(r)['csrf']
        headers['X-Chat-CSRF'] = csrf[0]
    req = urllib.request.Request(ORIGIN+path, data=None if body is None else json.dumps(body).encode(), headers=headers)
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


robot = Robot(PILOT/'.private/robot.json')
tags = TagRobot(robot)


def call(name, args):
    log('call', tool=name, args=args)
    res = robot.call(name, args)
    log('result', tool=name, payload={k: v for k, v in res.items() if k != 'images'})
    return res


def execution():
    e = call('robot_get_execution', {})['result']
    pose = {n: r['Present_Position'] for n, r in e['rows'].items()}
    return e, pose


def tag_view(tag):
    """OAK tag 2 position summary."""
    cap = tags.call('robot_get_tags', {'cameras': ['oak'], 'tag_ids': [1, 2]})
    row = cap['result']['observations']['oak']
    det = {t['tag_id']: t for t in row.get('tags', []) if t.get('status') == 'DETECTED'}
    for i, pic in enumerate(cap.get('images', [])):
        (OUT/f'{tag}-{pic["view"]}.jpg').write_bytes(base64.b64decode(pic['data_base64']))
    summary = {'frame': row.get('frame', {}).get('seq'), 'ids': sorted(det),
               'tag2_center_px': det[2].get('center_px') or det[2].get('center') if 2 in det else None,
               'tag2': {k: v for k, v in det[2].items() if k in ('center_px', 'center', 'corners_px', 'corners', 'decision_margin', 'quality', 'border_clearance_px')} if 2 in det else None,
               'all_tags': [(t['tag_id'], t.get('status')) for t in row.get('tags', [])]}
    save(f'{tag}-tags.json', {k: v for k, v in cap.items() if k != 'images'})
    return summary


before, pose0 = execution()
save('before.json', before)
assert before['phase'] == 'idle' and not before['enabled_motors'] and before['started'] == OWNER_STARTED \
    and before['stop_latched'] is False, 'owner not idle/released'
stop0, writes0 = before['stop_count'], before['motor_writes']
print('start pose', {n: pose0[n] for n in SIX})
print('tags before', json.dumps(tag_view('before')))

rec = api('/api/record', {'cameras': ['oak', 'phone', 'right_wrist'], 'seconds': 150})
save('recording.json', rec)
print('recording', rec.get('ok'), rec.get('label'), rec.get('started'))
time.sleep(1.5)

en = call('robot_set_motor_enable', {'names': SIX, 'enabled': True})
assert en.get('ok'), en
results = []
try:
    target = pose0[JOINT]
    for i, delta in enumerate(STEPS):
        target += delta
        dur = float(max(2, min(8, abs(delta)/60)))
        mv = call('robot_move_joint_targets', {'arm': 'right', 'positions': {JOINT: target}, 'duration_s': dur, 'wait': True})
        time.sleep(1.0)
        e, pose = execution()
        view = tag_view(f'stage{i+1}')
        row = {'step': i+1, 'delta': delta, 'target': target, 'ok': mv.get('ok'),
               'result': {k: v for k, v in (mv.get('result') or {}).items() if k in ('completed', 'closure_outcome', 'halted', 'contact', 'measured', 'final', 'error', 'reason', 'positions')},
               'pose': {n: pose[n] for n in SIX}, 'elbow_load': e['rows'][JOINT].get('Present_Load'),
               'phase': e['phase'], 'stop_count': e['stop_count'], 'tags': view}
        results.append(row); save('stages.json', results)
        print(json.dumps(row))
        if not mv.get('ok') or e['stop_count'] != stop0:
            print('stopping: move not ok or STOP occurred'); break
finally:
    rel = call('robot_set_motor_enable', {'names': SIX, 'enabled': False})
    save('release.json', rel)
    time.sleep(3)
    after, pose1 = execution()
    save('after.json', after)
    print('released', after['phase'], after['enabled_motors'], 'all torque off:',
          all(r['Torque_Enable'] == 0 for r in after['rows'].values()))
    print('pose after release', {n: pose1[n] for n in SIX})
    print('tags after release', json.dumps(tag_view('after')))
    save('summary.json', {'stages': results, 'pose_before': pose0, 'pose_after_release': pose1,
                          'stop_count': [stop0, after['stop_count']], 'motor_writes': [writes0, after['motor_writes']]})
