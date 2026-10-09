"""Position the right hand into the OAK view with the pilot's move tool, keep the six motors holding,
then run the automatic tag registration from that held pose (held_start)."""
import sys, json, time, base64, urllib.request
from pathlib import Path
ROOT = Path('/Users/wk/conductor/workspaces/xlerobot-farm/seville-v2')
PILOT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot')
sys.path[:0] = [str(ROOT/'software'), str(PILOT)]
from chat_server import Robot
from farm.perception.gemma_tags import TagRobot
from farm.perception.gemma_calibration import CalibrationRobot

OUT = ROOT/'.context/tag-registration-20261010'/sys.argv[1]; OUT.mkdir(parents=True, exist_ok=False)
TARGETS = json.loads(sys.argv[2])   # ordered list of [joint, target_ticks, duration_s]
MIN_CLEAR_PX = 60
SIX = ['right_arm_shoulder_pan', 'right_arm_shoulder_lift', 'right_arm_elbow_flex',
       'right_arm_wrist_flex', 'right_arm_wrist_roll', 'right_arm_gripper']
OWNER_STARTED = 1791551739.406972
ORIGIN = 'http://127.0.0.1:1241'
LOG = (OUT/'calls.jsonl').open('a')
def log(kind, **row):
    row.update(kind=kind, t=time.time()); LOG.write(json.dumps(row)+'\n'); LOG.flush()
def save(name, value):
    (OUT/name).write_text(json.dumps(value, indent=1, default=str)+'\n'); return value
def api(path, body=None, csrf=[None]):
    headers = {'Content-Type': 'application/json', 'Origin': ORIGIN}
    if body is not None:
        if csrf[0] is None:
            req = urllib.request.Request(ORIGIN+'/api/session', headers={'Origin': ORIGIN, 'X-Chat-Session': 'refresh'})
            with urllib.request.urlopen(req, timeout=5) as r: csrf[0] = json.load(r)['csrf']
        headers['X-Chat-CSRF'] = csrf[0]
    req = urllib.request.Request(ORIGIN+path, data=None if body is None else json.dumps(body).encode(), headers=headers)
    with urllib.request.urlopen(req, timeout=10) as r: return json.load(r)

raw = Robot(PILOT/'.private/robot.json')
tags = TagRobot(raw)
calib = CalibrationRobot(tags)
calib.catalog()
def call(name, args):
    log('call', tool=name, args=args); res = raw.call(name, args)
    log('result', tool=name, payload={k: v for k, v in res.items() if k != 'images'}); return res
def execution():
    e = call('robot_get_execution', {})['result']
    return e, {n: r['Present_Position'] for n, r in e['rows'].items()}
def tag_view(label):
    cap = tags.call('robot_get_tags', {'cameras': ['oak'], 'tag_ids': [1, 2]})
    row = cap['result']['observations']['oak']
    det = {t['tag_id']: t for t in row.get('tags', []) if t.get('status') == 'DETECTED'}
    for pic in cap.get('images', []): (OUT/f'{label}-{pic["view"]}.jpg').write_bytes(base64.b64decode(pic['data_base64']))
    save(f'{label}-tags.json', {k: v for k, v in cap.items() if k != 'images'})
    clear = None
    if 2 in det:
        w, h = row['image_size_px']
        clear = min(min(x, y, w-1-x, h-1-y) for x, y in det[2]['corners_px'])
    return {'frame': row.get('frame', {}).get('seq'), 'ids': sorted(det), 'tag2_center_px': det[2]['center_px'] if 2 in det else None, 'tag2_border_clearance_px': clear}

before, pose0 = execution(); save('before.json', before)
assert before['phase'] == 'idle' and not before['enabled_motors'] and before['started'] == OWNER_STARTED and before['stop_latched'] is False
stop0 = before['stop_count']
print('start', {n: pose0[n] for n in SIX}); print('tags', json.dumps(tag_view('before')))
rec = api('/api/record', {'cameras': ['oak', 'phone', 'right_wrist'], 'seconds': 150}); save('recording.json', rec)
print('recording', rec.get('ok'), rec.get('label'), rec.get('started'), rec.get('error'))
time.sleep(1.0)
en = call('robot_set_motor_enable', {'names': SIX, 'enabled': True}); assert en.get('ok'), en
stages = []
ok = True
for joint, target, dur in TARGETS:
    mv = call('robot_move_joint_targets', {'arm': 'right', 'positions': {joint: int(target)}, 'duration_s': float(dur), 'wait': True})
    time.sleep(0.8)
    e, pose = execution(); view = tag_view(f'after-{joint}')
    row = {'joint': joint, 'target': target, 'ok': mv.get('ok'), 'outcome': (mv.get('result') or {}).get('closure_outcome'),
           'pose': {n: pose[n] for n in SIX}, 'load': e['rows'][joint].get('Present_Load'), 'phase': e['phase'], 'stop_count': e['stop_count'], 'tags': view}
    stages.append(row); save('stages.json', stages); print(json.dumps(row))
    if not mv.get('ok') or e['stop_count'] != stop0: ok = False; break
if not ok or 2 not in stages[-1]['tags']['ids'] or stages[-1]['tags']['tag2_border_clearance_px'] < MIN_CLEAR_PX:
    print('NOT READY: releasing'); rel = call('robot_set_motor_enable', {'names': SIX, 'enabled': False}); save('release.json', rel); sys.exit(2)
print('held pose ready; starting registration', time.time())
res = calib.call('robot_calibrate_tags', {'mode': 'registration', 'held_start': True})
res.pop('images', None); save('registration-result.json', res)
r = res.get('result', {})
print('registration ok', res.get('ok'), 'status', r.get('status'), 'error', r.get('error'))
print(json.dumps({k: r.get(k) for k in ('residuals', 'calibration_commands_sent', 'commanded_path_ticks', 'cleanup', 'output', 'held_start')}, indent=1, default=str)[:3000])
time.sleep(3)
after, pose1 = execution(); save('after.json', after)
print('final', after['phase'], after['enabled_motors'], 'all torque off', all(x['Torque_Enable'] == 0 for x in after['rows'].values()), 'stop_count', after['stop_count'], {n: pose1[n] for n in SIX})
