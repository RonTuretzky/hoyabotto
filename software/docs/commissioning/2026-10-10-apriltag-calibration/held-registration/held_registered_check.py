"""Hold the right hand in the OAK view and read the installed registration (robot_get_registered_tags), then release."""
import sys, json, time, base64
from pathlib import Path
ROOT = Path('/Users/wk/conductor/workspaces/xlerobot-farm/seville-v2'); PILOT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot')
sys.path[:0] = [str(ROOT/'software'), str(PILOT)]
from chat_server import Robot
from farm.perception.gemma_tags import TagRobot
from farm.perception.gemma_calibration import CalibrationRobot
OUT = ROOT/'.context/tag-registration-20261010'/sys.argv[1]; OUT.mkdir(parents=True, exist_ok=False)
TARGETS = json.loads(sys.argv[2])
SIX = ['right_arm_shoulder_pan','right_arm_shoulder_lift','right_arm_elbow_flex','right_arm_wrist_flex','right_arm_wrist_roll','right_arm_gripper']
raw = Robot(PILOT/'.private/robot.json'); calib = CalibrationRobot(TagRobot(raw)); calib.catalog()
def save(n, v): (OUT/n).write_text(json.dumps(v, indent=1, default=str)+'\n')
e = raw.call('robot_get_execution', {})['result']; assert e['phase']=='idle' and not e['enabled_motors'], e['phase']
save('before.json', e)
assert raw.call('robot_set_motor_enable', {'names': SIX, 'enabled': True}).get('ok')
reads = []
try:
    for joint, target, dur in TARGETS:
        mv = raw.call('robot_move_joint_targets', {'arm': 'right', 'positions': {joint: int(target)}, 'duration_s': float(dur), 'wait': True}); assert mv.get('ok'), mv
    time.sleep(1.0)
    for i in range(3):
        r = calib.call('robot_get_registered_tags', {}); r.pop('images', None); reads.append(r); save(f'registered-{i}.json', r)
        res = r.get('result', {})
        print('read', i, 'ok', r.get('ok'), 'error', res.get('error'))
        print(json.dumps({k: res.get(k) for k in res if k in ('status','tags','gripper','gripper_tag','disagreement','frame_id','freshness','consistency','gripper_disagreement_mm','observed_vs_fk')}, indent=1, default=str)[:2500])
        time.sleep(1.0)
finally:
    rel = raw.call('robot_set_motor_enable', {'names': SIX, 'enabled': False}); save('release.json', rel); time.sleep(3)
    e = raw.call('robot_get_execution', {})['result']; save('after.json', e)
    print('released', e['phase'], e['enabled_motors'], 'torque all off', all(x['Torque_Enable']==0 for x in e['rows'].values()), {n: e['rows'][n]['Present_Position'] for n in SIX})
