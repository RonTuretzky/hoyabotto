"""Private, disarmed mTLS robot tools; serial reads share the canonical owner lock."""
import base64
import contextlib
import hashlib
import io
import json
import os
import ssl
import sys
import threading
import time
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = Path('/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20')
UTILITY = OLD / 'work/carton-visual-controller/software'
SCRIPTS = UTILITY / 'scripts/carton_robot'
TLS = ROOT / 'work/gemma-mtls'
SESSION = ROOT / 'work/gemma-hardware-session'
OAK_RAW_DIR = Path(os.environ.get('XLEROBOT_OAK_RAW_DIR', str(ROOT / 'work/oak-stream')))
OAK_RECTIFIED_DIR = Path(os.environ.get('XLEROBOT_OAK_RECTIFIED_DIR', str(ROOT / 'work/oak-rectified-stream')))
PASSIVE_RECOVERY = os.environ.get('XLEROBOT_PASSIVE_RECOVERY') == '1'
os.environ.update(CARTON_WORKSPACE_ROOT=str(ROOT),
                  CARTON_LIVE_SOFTWARE=str(OLD / 'xlerobot-farm/software'),
                  CARTON_UTILITY_SOFTWARE=str(UTILITY))
sys.path.insert(0, str(SCRIPTS))
from carton_preflight import run as read_preflight
# Runtime keeps live farm first; resolve the canonical carton tool package explicitly.
sys.path.insert(0, str(UTILITY))
from carton.servo.common import atomic_json
from gemma_execution_binding import TrustedExecutionBinding
from gemma_direct_client import DirectJointClient
LEGACY_CONTINUOUS_BINDING = TrustedExecutionBinding(SESSION)

def bind_trusted_execution(adapter, reference_provider, *, source):
    """Local station integration only; deliberately absent from HTTP schemas."""
    LEGACY_CONTINUOUS_BINDING.bind(adapter, reference_provider, source=source)

SERIAL_READ = threading.Lock()
CACHE_LOCK = threading.Lock()
REQUESTS = OrderedDict()
LAST_GOOD = None
calibration = Path('/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json')
CAL = json.loads(calibration.read_text())
DIRECT_CLIENT = DirectJointClient(SESSION, CAL)
PEER_DER = ssl.PEM_cert_to_DER_cert((TLS / 'gateway-server.pem').read_text())
PEER_SHA = hashlib.sha256(PEER_DER).hexdigest()


def tool(name, description, properties=None, required=None):
    return {'type': 'function', 'function': {'name': name, 'description': description,
        'parameters': {'type': 'object', 'properties': properties or {},
                       'required': required or [], 'additionalProperties': False}}}


POSITION_NAMES = [n for n in CAL if n.startswith(('left_arm_', 'right_arm_', 'head_motor_'))]
ARM_NAMES = [n for n in POSITION_NAMES if '_arm_' in n]
ARM_ALIASES = sorted({n.split('_arm_', 1)[1] for n in ARM_NAMES})
def target_schema(names, description):
    return {'type': 'object', 'description': description, 'minProperties': 1,
            'properties': {n: ({'type': 'integer', 'minimum': CAL[n]['range_min']+4, 'maximum': CAL[n]['range_max']-4} if n in CAL else {'type':'integer'}) for n in names}, 'additionalProperties': False}
TARGET = target_schema(POSITION_NAMES, 'Absolute raw encoder ticks by canonical motor name; query robot_get_capabilities.commandable_ranges first. Endpoints are not geometric full extension.')
ARM_TARGET = target_schema(ARM_NAMES + ARM_ALIASES, 'Canonical names preferred, e.g. right_arm_shoulder_lift. With arm=right, shoulder_lift is also accepted. Do not mix aliases for the same joint; wrong-arm keys rejected. Example shape: {"right_arm_shoulder_lift": 2000}; select targets from fresh state and commandable_ranges.')
HEAD_TARGET = target_schema([n for n in POSITION_NAMES if n.startswith('head_motor_')], 'Canonical head motor names and integer encoder ticks.')

def commandable_ranges():
    return {n: {'min_ticks': CAL[n]['range_min'] + 4, 'max_ticks': CAL[n]['range_max'] - 4, 'margin_ticks': 4} for n in POSITION_NAMES}

def paddle_target_segments(targets):
    """Convert model targets into bounded pickup-profile segments from live goals."""
    readiness = DIRECT_CLIENT.readiness()
    if readiness.get('execution_profile') != 'paddle-success-v1':
        return [targets]
    state = DIRECT_CLIENT.status()
    goals = state.get('goals', {})
    rows = state.get('rows', {})
    segments = []
    for name, target in targets.items():
        start = goals.get(name, rows.get(name, {}).get('Present_Position'))
        if type(start) is not int:
            raise ValueError('Pickup start encoder unavailable for '+name+'; refresh robot_get_state')
        delta = target - start
        if abs(delta) <= 2:
            continue
        step = 341 if delta > 0 else -341
        cursor = start
        while abs(target-cursor) > 341:
            cursor += step; segments.append({name: cursor})
        if target != cursor: segments.append({name: target})
    if not segments:
        return []
    return segments

def normalize_targets(targets, arm=None, head=False):
    result = {}
    for key, q in targets.items():
        n = arm + '_arm_' + key if arm and key in ARM_ALIASES else key
        if n not in POSITION_NAMES:
            raise ValueError('Unknown position joint: ' + key + '; valid canonical names: ' + ', '.join(POSITION_NAMES))
        if arm and not n.startswith(arm + '_arm_'):
            raise ValueError('Wrong-arm joint: ' + key + '; requested arm: ' + arm)
        if head and not n.startswith('head_motor_'):
            raise ValueError('Head tool accepts head motors only: ' + key)
        if n in result:
            raise ValueError('Duplicate/conflicting aliases for joint: ' + n)
        result[n] = q
    for n, q in result.items():
        bounds = commandable_ranges()[n]
        if not bounds['min_ticks'] <= q <= bounds['max_ticks']:
            raise ValueError(f"Target out of bounds: {n}={q}; commandable inclusive range [{bounds['min_ticks']}, {bounds['max_ticks']}] ticks (4-tick margin)")
    return result

MOTOR_NAMES = {'type': 'array', 'items': {'type': 'string', 'enum': list(CAL)}, 'minItems': 1, 'maxItems': 16, 'uniqueItems': True}
TOOLS = [
    tool('robot_list_motors', 'List all16 configured motors and saved ranges. Live owner telemetry when available; explicitly aged historical rows in passive recovery, never asserted as current torque state. No serial owner duplication.'),
    tool('robot_set_motor_enable', 'Explicitly enable or release named motors through the single hardware owner. Wheel activation holds the current encoder in existing position mode0; no wheel movement/mode change. Never automatically activates at server startup.', {'names': MOTOR_NAMES, 'enabled': {'type': 'boolean'}}, ['names', 'enabled']),
    tool('robot_move_motor_targets', 'Raw integer encoder targets for the14 arm/head/gripper position motors already enabled. Sole owner enforces ranges, rates, following, watchdog and measured completion. No wheel movement or automatic activation.', {'positions': TARGET, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25}}, ['positions', 'duration_s']),
    tool('robot_get_state', 'Read all16 servo states while released; during an owner session return live selected-arm telemetry and explicitly aged cached remaining motors. Never creates a second serial owner.', {'fresh': {'type': 'boolean'}}),
    tool('robot_get_cameras', 'Fresh OAK RGB (prefer actual rectified stream, explicit distorted fallback) and phone snapshots. Reject stale feeds. Phone timestamp is receipt, not capture; images do not authorize motion.', {'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': ['oak', 'phone']}, 'minItems': 1, 'maxItems': 2}}),
    tool('robot_get_capabilities', 'Report actual joint ranges, units, supported controller protocol and concrete motion blockers.'),
    tool('robot_get_depth', 'Fresh OAK depth PNG paired with actual rectified or raw RGB manifest. Reject stale feeds; RGB-depth registration and robot transform remain unverified.'),
    tool('robot_get_handoff', 'Retrieve the user-authorized complete paddle-task handoff, historical evidence and guards, plus current camera and saved servo ages. Context transfer never arms or binds execution.'),
    tool('robot_get_readiness', 'Saved calibration and physical reference readiness; historical evidence is explicitly dated.'),
    tool('robot_get_keyframes', 'Inspect real and simulated named keyframes; existence never authorizes blind replay.'),
    tool('robot_get_skills', 'Inventory validated physical visual skills and source implementations. Does not execute skills.'),
    tool('robot_get_evidence', 'Read bounded deployment evidence and recording metadata without arbitrary filesystem access.'),
    tool('robot_get_execution', 'Read the bound sole-owner execution status; no motor connection.'),
    tool('robot_stop', 'Independent STOP for the current bound owner; never enables motors, clears STOP, or restarts an owner.'),
    tool('robot_move_joint_targets', 'Direct encoder targets through the existing sole owner. Owner enforces saved range margins, speed/acceleration/torque/health/watchdog and measured completion. No continuous commissioning or Cartesian transform required. Does not start or arm an owner.', {'arm': {'type': 'string', 'enum': ['left', 'right']}, 'positions': ARM_TARGET, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25}}, ['arm', 'positions', 'duration_s']),
    tool('robot_move_head', 'Direct head targets when the current owner explicitly supports those head motors. Does not start or arm an owner; saved ranges and supervision enforced.', {'positions': HEAD_TARGET, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25}}, ['positions']),
    tool('robot_set_gripper', 'Direct gripper encoder target within current owner selected scope; saved range and supervision enforced. Stall does not establish grasp success.', {'arm': {'type': 'string', 'enum': ['left', 'right']}, 'position_ticks': {'type': 'integer'}, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25}}, ['arm', 'position_ticks']),
    tool('robot_move_base', 'Base drive request; currently unsupported by the parked arm owner. No wheel activation.', {'linear_m_s': {'type': 'number'}, 'angular_rad_s': {'type': 'number'}, 'duration_s': {'type': 'number'}}, ['linear_m_s', 'angular_rad_s', 'duration_s']),
]
from gemma_reach_planner import POSE_SCHEMA, validate_poses, inspect_or_plan
TOOLS.extend([
    tool('robot_plan_reach', 'Read-only existing repository LeRobot/Placo reach planner using fresh sole-owner encoders and measured configuration. Returns proposals only when actual calibration permits; otherwise exact missing configuration. No execution or collision certification.', {'arm': {'type':'string','enum':['left','right']}, 'frame': {'type':'string','enum':['arm_base','station']}, 'units': {'type':'string','enum':['metres']}, 'orientation': {'type':'string','enum':['constrained','position_only']}, 'tool_poses': POSE_SCHEMA}, ['arm','frame','units','orientation','tool_poses']),
    tool('robot_get_arm_pose', 'Read-only FK tool pose in arm-base metres from measured repository configuration and live encoders; reports missing geometry explicitly.', {'arm': {'type':'string','enum':['left','right']}}, ['arm'])
])
# Arm-dependent limits also bound short aliases and reject wrong-arm keys in local JSON Schema validation.
for entry in TOOLS:
    fn=entry['function']; params=fn['parameters']
    if fn['name']=='robot_move_joint_targets':
        params['allOf']=[]
        for arm in ('left','right'):
            names=[n for n in ARM_NAMES if n.startswith(arm+'_arm_')]
            arm_spec=target_schema(names,'Selected arm canonical names or explicit-arm aliases')
            for n in names: arm_spec['properties'][n.split('_arm_',1)[1]]=dict(arm_spec['properties'][n])
            params['allOf'].append({'if':{'properties':{'arm':{'const':arm}},'required':['arm']},'then':{'properties':{'positions':arm_spec}}})
    if fn['name']=='robot_set_gripper':
        ranges={a:commandable_ranges()[a+'_arm_gripper'] for a in ('left','right')}
        message='; '.join(f"{a}: {b['min_ticks']}..{b['max_ticks']} inclusive ticks" for a,b in ranges.items())
        fn['description'] += ' Validates first, then enables only this gripper if released and moves through the sole owner; failure triggers STOP cleanup. Right-gripper execution uses fixed measured-progress waypoints up to48ticks, a1s no-progress guard, and20tick final endpoint tolerance with directed-travel and three fresh stable samples; reports raw endpoint error, not verified jaw state. Other position tools do not auto-enable. Commandable gripper ranges: '+message+'. Raw calibration endpoints are invalid command targets; no clamping.'
        params['properties']['position_ticks']['description']='Commandable integer encoder ticks: '+message
        params['allOf']=[{'if':{'properties':{'arm':{'const':a}},'required':['arm']},'then':{'properties':{'position_ticks':{'minimum':b['min_ticks'],'maximum':b['max_ticks']}}}} for a,b in ranges.items()]
SCHEMAS = {t['function']['name']: t['function']['parameters'] for t in TOOLS}


def execution():
    p = SESSION / 'status.json'
    if not p.exists():
        return {'phase': 'unavailable', 'active': False}
    s = json.loads(p.read_text())
    age = time.time() - s['time']
    s['status_age_s'] = age
    s['active'] = (s.get('hardware_server') is True or s.get('phase') in ('holding', 'moving', 'starting', 'startup')) and 0 <= age <= 1
    return s


def _state(fresh=True):
    global LAST_GOOD
    s = execution()
    if s['active'] and s.get('hardware_server') is True:
        return {'source': 'canonical_hardware_owner', 'time': s['time'], 'read_age_s': s['status_age_s'],
                'cached': False, 'motors': [{'name': name, **row} for name, row in s.get('rows', {}).items()],
                'control_mode': s.get('control_mode'), 'enabled_motors': s.get('enabled_motors', []),
                'all_16_released': len(s.get('rows', {})) == 16 and all(row.get('Torque_Enable') == 0 for row in s['rows'].values())}
    if s['active']:
        return {'source': 'existing_sole_owner', 'selected_arm': s.get('arm'),
                'live_rows': s.get('rows', {}), 'owner_time': s.get('time'),
                'remaining_motor_cache': LAST_GOOD,
                'cache_warning': 'Inactive motors are not freshly read while owner holds serial ports'}
    if not fresh and LAST_GOOD is not None:
        return {**LAST_GOOD, 'read_age_s': time.time() - LAST_GOOD['time'], 'cached': True}
    if PASSIVE_RECOVERY:
        if fresh:
            raise RuntimeError('HARDWARE_OWNER_UNAVAILABLE: passive Qwen recovery does not open servo ports; main-thread handoff required')
        rows = s.get('rows', {})
        return {'source': 'historical_owner_snapshot', 'time': s.get('time'),
                'read_age_s': s.get('status_age_s'), 'cached': True,
                'motors': [{'name': n, **row} for n, row in rows.items()],
                'all_16_released': False, 'current_torque_state_confirmed': False,
                'all_16_released_at_snapshot': len(rows) == 16 and all(row.get('Torque_Enable') == 0 for row in rows.values()),
                'hardware_owner_active': False,
                'blocker': 'Main-thread servo-port handoff required; no serial fallback in passive recovery'}
    if not SERIAL_READ.acquire(blocking=False):
        raise RuntimeError('A state read is already active')
    try:
        # Canonical read_preflight also acquires the cross-process exact-port lock.
        with contextlib.redirect_stdout(io.StringIO()):
            result = read_preflight()
        if result.get('error'):
            raise RuntimeError(result['error'])
        if len(result['motors']) != 16:
            raise RuntimeError('Incomplete servo state read')
        LAST_GOOD = result
        return {**result, 'read_age_s': time.time() - result['time'], 'cached': False}
    finally:
        SERIAL_READ.release()



def state(fresh=True):
    result=dict(_state(fresh))
    if 'motors' in result:result['motors']=[{k:v for k,v in row.items() if k!='coherent_read_evidence'} for row in result['motors']]
    if 'live_rows' in result:result['live_rows']={n:{k:v for k,v in row.items() if k!='coherent_read_evidence'} for n,row in result['live_rows'].items()}
    result['commandable_ranges']=commandable_ranges()
    result['raw_calibration_ranges']={n:{'min_ticks':v['range_min'],'max_ticks':v['range_max']} for n,v in CAL.items()}
    result['range_semantics']='raw_calibration_ranges and motor range are saved hardware limits, not command targets; use commandable_ranges (inclusive, 4-tick margin)'
    return result


def camera_status():
    result = {}
    for name, path, field in [
        ('oak_rectified', OAK_RECTIFIED_DIR / 'oak.json', 'captured_at'),
        ('oak_raw', OAK_RAW_DIR / 'oak.json', 'captured_at'),
        ('phone', ROOT / 'work/phone_camera/latest.json', 'received_at')]:
        try:
            meta = json.loads(path.read_text())
            age = time.time() - meta[field]
            result[name] = {'available': True, 'fresh': 0 <= age <= 1, 'age_s': age,
                field: meta[field], 'seq': meta.get('seq'), 'stream_id': meta.get('stream_id'),
                'projection': meta.get('projection'),
                'rgb_depth_pixel_registration_verified': meta.get('rgb_depth_pixel_registration_verified', False),
                'robot_frame_calibrated': meta.get('robot_frame_calibrated', False)}
            if name == 'phone':
                result[name]['timestamp_semantics'] = 'server receipt; capture delay unknown'
                result[name]['captured_at'] = None
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result[name] = {'available': False, 'fresh': False, 'error': str(exc)}
    result['continuous_visual_registration_ready'] = False
    return result


def select_oak_manifest():
    # Never synthesize fresh timestamps or relabel distorted RGB as rectified.
    errors = []
    for source, folder in [('rectified', OAK_RECTIFIED_DIR),
                           ('raw_fallback', OAK_RAW_DIR)]:
        try:
            meta = json.loads((folder / 'oak.json').read_text())
            age = time.time() - meta['captured_at']
            if not 0 <= age <= 1:
                errors.append(source + ' stale age_s=' + str(round(age, 3)))
                continue
            if source == 'rectified' and meta.get('projection') != 'rectified_pinhole':
                errors.append('rectified stream does not declare its actual rectified projection')
                continue
            return folder, meta, source
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(source + ': ' + str(exc))
    raise RuntimeError('No fresh OAK stream: ' + '; '.join(errors))


def manifest_image_path(folder, filename):
    path = (folder / filename).resolve()
    if not path.is_relative_to(folder.resolve()):
        raise ValueError('Image manifest path outside camera directory')
    return path


def cameras_strict(names):
    images, metadata = [], {}
    for name in names:
        for _ in range(15):
            try:
                if name == 'oak':
                    folder, m, source = select_oak_manifest()
                    data = manifest_image_path(folder, m['image']).read_bytes()
                    assert hashlib.sha256(data).hexdigest() == m['sha256']
                    stamp = m['captured_at']
                    identity = m['camera_id']
                    image = {'camera_id': identity, 'mime_type': 'image/jpeg',
                             'captured_at': stamp, 'received_at': None, 'seq': m['seq'], 'stream_id': m['stream_id'],
                             'source': source, 'projection': m.get('projection'),
                             'rgb_depth_pixel_registration_verified': m.get('rgb_depth_pixel_registration_verified', False),
                             'robot_frame_calibrated': m.get('robot_frame_calibrated', False)}
                elif name == 'phone':
                    folder = ROOT / 'work/phone_camera'
                    m = json.loads((folder / 'latest.json').read_text())
                    data = (folder / 'latest.jpg').read_bytes()
                    assert m['seq'] == json.loads((folder / 'latest.json').read_text())['seq']
                    stamp = m['received_at']
                    image = {'camera_id': 'phone_overview', 'mime_type': 'image/jpeg',
                             'captured_at': None, 'received_at': stamp, 'seq': m['seq'],
                             'timestamp_semantics': 'server receipt; capture delay unknown'}
                else:
                    raise ValueError('Unsupported camera')
                if not 0 <= time.time() - stamp <= 1:
                    raise RuntimeError(name + ' image is stale')
                image.update(data_base64=base64.b64encode(data).decode(),
                             sha256=hashlib.sha256(data).hexdigest())
                images.append(image)
                metadata[name] = m
                break
            except (FileNotFoundError, AssertionError, json.JSONDecodeError):
                time.sleep(.02)
        else:
            raise RuntimeError(name + ' consistent snapshot unavailable')
    return metadata, images



def cameras(names):
    # One offline camera must not discard independently fresh other images.
    metadata, images, errors = {}, [], {}
    for name in names:
        try:
            current, frames = cameras_strict([name])
            metadata.update(current)
            images.extend(frames)
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
            errors[name] = {'error': str(exc), 'fresh': False}
    return {'cameras': metadata, 'camera_errors': errors,
            'all_requested_cameras_fresh': not errors,
            'continuous_visual_registration_ready': False}, images



def saved_motor_readiness():
    # Read an existing canonical preflight snapshot; this never opens serial.
    path = ROOT / 'work/carton-preflight.json'
    try:
        snapshot = json.loads(path.read_text())
        age = time.time() - snapshot['time']
        return {'source': 'saved canonical lock-protected carton_preflight snapshot',
                'captured_at': snapshot['time'], 'age_s': age, 'fresh': 0 <= age <= 1,
                'all_16_released_at_snapshot': snapshot.get('all_16_released'),
                'current_release_confirmed_by_this_metadata_read': False,
                'out_of_range_at_snapshot': [row for row in snapshot.get('motors', []) if row.get('in_range') is False],
                'head_at_snapshot': [row for row in snapshot.get('motors', []) if row['name'].startswith('head_motor_')]}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {'available': False, 'fresh': False, 'error': str(exc)}


def capabilities():
    ready = DIRECT_CLIENT.readiness()
    return {'mode': 'direct_joint', 'control_mode': 'direct_joint',
            'execution_adapter_bound': True, 'armed': ready['operator_armed'],
            'motion_ready': ready['motion_ready'],
            'available_to_accept_authorized_command': ready['available_to_accept_authorized_command'],
            'execution_binding': ready, 'joint_blockers': ready['joint_blockers'],
            'camera_status': camera…6605 tokens truncated…in self.commandable_names],'ranges':self.ranges,'capabilities':['read','enable_motors','direct_joint','stop','release'],'pickup_required_enabled_motors':self.position_names if paddle_profile else [],'pickup_motion_segment_budget':20 if paddle_profile else None,'pickup_idle_hold_seconds':120 if paddle_profile else 30,'execution_profile':'paddle-success-v1' if paddle_profile else 'legacy-direct','motor_writes':0,'software_temperature_limit_c':SOFTWARE_TEMPERATURE_LIMIT_C}
 def read(self,n,f):return int(self.by_name[n].read(f,n,normalize=False,num_retry=2))
 def write(self,n,f,v):
  self.by_name[n].write(f,n,v,normalize=False,num_retry=2);self.state['motor_writes']+=1
  actual=self.read(n,f)
  self.state.setdefault('last_write_readbacks',{}).setdefault(n,{})[f]={'requested':v,'readback':actual,'time':self.wall()}
  if actual!=v:raise RuntimeError(n+': '+f+' readback mismatch')
 def inspect(self):
  for n in self.names:
   if self.read(n,'Torque_Enable')!=0:raise RuntimeError(n+': already powered before hardware-owner startup')
   self.limits[n]=[self.read(n,'Min_Position_Limit'),self.read(n,'Max_Position_Limit')]
   if n in self.cal:
    c=self.cal[n]
    expected={'Homing_Offset':c.homing_offset,'Min_Position_Limit':c.range_min,'Max_Position_Limit':c.range_max}
    actual={f:self.read(n,f) for f in expected}
    if actual!=expected:
     self.state.setdefault('calibration_mismatches',{})[n]={'expected':expected,'actual':actual}
     if n in self.commandable_names:raise RuntimeError(n+': saved calibration differs from hardware')
  self.state['released_register_diagnostics']={n:self.register_diagnostics(n) for n in self.names if n.endswith('gripper')}
  self.poll()
  n='right_arm_gripper'
  if n in self.by_name and not self.paddle_profile:
   coherent=dict(self.rows[n])
   independent=self.read(n,'Present_Temperature')
   self.state['independent_temperature_check']={'motor':n,'time':self.wall(),'register':63,'length':1,'coherent_temperature':coherent['Present_Temperature'],'independent_temperature':independent,'coherent_evidence':coherent.get('coherent_read_evidence'),'independent_reply':getattr(self.by_name[n],'last_reply_evidence',None),'matches':independent==coherent['Present_Temperature'],'motor_writes':0}
 def register_diagnostics(self,n):
  return {'captured_at':self.wall(),'registers':{f:self.read(n,f) for f in DIAGNOSTIC_REGISTERS},'read_only':True}
 def read_telemetry(self,n):
  # Retry one transport corruption/timeout only in a fully observed released idle scope.
  idle_released=(not self.enabled and not (self.engine and self.engine.active)
                 and len(self.rows)==len(self.names)
                 and all(row.get('Torque_Enable')==0 and type(row.get('captured_at')) in (int,float)
                         and 0<=self.wall()-row['captured_at']<=1 for row in self.rows.values()))
  for attempt in range(2 if idle_released else 1):
   try:
    row=self.telemetry(self.by_name[n],n)
    evidence=getattr(self.by_name[n],'last_telemetry_sample',None)
    if evidence is not None:row['coherent_read_evidence']=evidence
    if attempt:
     self.state['idle_read_recovery']['recovered_count']+=1
     self.state['idle_read_recovery']['events'][-1]['outcome']='recovered_on_retry'
     self.state['idle_read_recovery']['events'][-1]['recovered_at']=self.wall()
    return row
   except RuntimeError as exc:
    transport_failure=str(exc).startswith('Coherent servo read communication failure: -7')
    if not idle_released or not transport_failure:raise
    report=self.state.setdefault('idle_read_recovery',{'failure_count':0,'recovered_count':0,'events':[],'max_attempts':2,'powered_retries':False})
    report['failure_count']+=1
    report['events'].append({'time':self.wall(),'motor':n,'attempt':attempt+1,'outcome':'retry_pending' if attempt==0 else 'retry_failed','diagnostic':str(exc),'transaction':getattr(self.by_name[n],'last_telemetry_failure',None)})
    report['events']=report['events'][-16:]
    if attempt:raise
    time.sleep(.01)

 def poll(self):
  now=self.clock()
  if self.engine and self.engine.active and not 0<=now-self.last_tick<=(1.0 if self.paddle_profile else .2):raise RuntimeError('Motor-owner watchdog expired')
  poll_elapsed=now-self.last_tick
  self.last_tick=now
  for n in self.names:
   row=self.read_telemetry(n);row['Torque_Enable']=self.read(n,'Torque_Enable');row['Operating_Mode']=self.read(n,'Operating_Mode');row['captured_at']=self.wall();row['firmware_position_limits']=self.limits.get(n);self.rows[n]=row
   if n in self.enabled:
    failures=[('Torque_Enable',row['Torque_Enable'],'must equal',1),('Status',row['Status'],'must equal',0),*([('Present_Temperature',row['Present_Temperature'],'must be <=',SOFTWARE_TEMPERATURE_LIMIT_C)] if not self.paddle_profile else []),('Present_Load',row['Present_Load'],'absolute must be <=',500 if n.endswith('gripper') or not self.paddle_profile else 800)]
    for field,value,rule,limit in failures:
     failed=(value!=limit if field in ('Torque_Enable','Status') else value>limit if field=='Present_Temperature' else abs(value)>limit)
     if failed:
      self.state['health_fault']={'motor':n,'field':field,'value':value,'rule':rule,'limit':limit,'captured_at':row['captured_at']}
      raise RuntimeError(f'{n}: {field}={value}; {rule} {limit}')
    if self.paddle_profile and not 100<=row['Present_Voltage']<=140:raise RuntimeError(n+': pickup supply voltage outside 10..14V')
    if n in self.ranges and not self.ranges[n][0]<=row['Present_Position']<=self.ranges[n][1]:raise RuntimeError(n+': outside saved travel range')
    if not(self.engine and self.engine.active and n in self.engine.joints) and abs(row['Present_Position']-self.goals[n])>(96 if self.paddle_profile else 68):raise RuntimeError(n+': uncommanded holding drift')
  if self.enabled and self.clock()>self.lease:raise RuntimeError('Command heartbeat expired')
  camera_ready=True
  if self.paddle_profile:
   camera_ready=self.camera_gate.update(holding=bool(self.enabled)) if self.enabled else True
   self.state.update(camera_supervision_ok=camera_ready,camera_pause_active=not camera_ready,camera_pauses=self.camera_gate.events)
   if not camera_ready:
    self.lease+=max(0,poll_elapsed)
    if self.engine and self.engine.active:self.engine.pause(max(0,poll_elapsed))
  if self.engine and self.engine.active and camera_ready:
   current={n:self.rows[n]['Present_Position'] for n in self.engine.joints}
   update=self.engine.tick(current,telemetry_at=min(self.rows[n]['captured_at'] for n in self.engine.joints),**({'rows':self.rows} if self.paddle_profile else {}));self.state.update(update)
   if self.paddle_profile and not self.engine.active:self.lease=self.clock()+120
   if self.state.get('local_gripper_probe') and not self.engine.active:
    self.release_all('Local probe complete',latch=False)
  self.publish();return self.state
 def publish(self):
  if not self.latched and not(self.engine and self.engine.active):self.state['phase']='holding' if self.enabled else 'idle'
  self.state.update(time=self.wall(),rows=self.rows,enabled_motors=sorted(self.enabled),goals=self.goals,lease_remaining=max(0,self.lease-self.clock()),stop_latched=self.latched)
 def enable(self,names,enabled):
  if not isinstance(names,list) or not names or len(set(names))!=len(names) or not set(names)<=set(self.names) or type(enabled)is not bool:raise ValueError('Select known distinct motor names and boolean enabled')
  if not enabled:
   if self.engine and self.engine.active:self.release_all('Release requested during movement',latch=False)
   else:
    for n in names:self.release(n)
   self.publish();return
  if self.paddle_profile:
   try:camera_ready=self.camera_gate.update(holding=False)
   except RuntimeError as exc:raise ValueError(str(exc)) from exc
   if not camera_ready:raise ValueError('Pickup phone feed paused; motor activation refused')
  if self.read_only:raise ValueError('READ_ONLY_OWNER: motor activation disabled; calibration mismatch must be resolved deliberately')
  if not set(names)<=self.commandable_names:raise ValueError('UNSUPPORTED_OWNER_SCOPE: requested motors are read-only')
  if self.latched:raise ValueError('STOP is latched; a new deliberate operator session is required')
  if self.engine and self.engine.active:raise ValueError('Movement is in progress')
  # Validate the entire request before enabling any motor.
  for n in names:
   row=self.rows[n];q=row['Present_Position'];lo,hi=self.limits[n]
   if row['Status'] or (not self.paddle_profile and row['Present_Temperature']>SOFTWARE_TEMPERATURE_LIMIT_C) or abs(row['Present_Load'])>(500 if n.endswith('gripper') or not self.paddle_profile else 800):raise ValueError(n+': fault or health limit')
   if self.paddle_profile and not 100<=row['Present_Voltage']<=140:raise ValueError(n+': pickup supply voltage outside 10..14V')
   if row['Operating_Mode']!=0:raise ValueError(n+': current mode is not supported position-hold mode')
   if n in self.ranges and not self.ranges[n][0]+(40 if self.paddle_profile else 4)<=q<=self.ranges[n][1]-(40 if self.paddle_profile else 4):raise ValueError(n+': current position outside saved travel margin')
   if not 0<=q<=4095 or (lo<hi and not lo<=q<=hi):raise ValueError(n+': current position outside firmware position limits')
  for n in names:
   if n in self.enabled:continue
   self.state.setdefault('enable_register_diagnostics',{})[n]={'pre_enable':self.register_diagnostics(n)}
   self.old[n]={f:self.read(n,f) for f in ['Lock','Torque_Limit','Goal_Velocity','Goal_Time','Acceleration','P_Coefficient']}
   self.write(n,'Lock',0)
   torque=(500 if n.endswith('gripper') else 400 if n.endswith('elbow_flex') else 800) if self.paddle_profile else (250 if n.endswith('gripper') else 400)
   self.write(n,'Torque_Limit',min(self.old[n]['Torque_Limit'],torque));self.write(n,'Goal_Velocity',200 if self.paddle_profile and n.endswith('gripper') else 100);self.write(n,'Goal_Time',0);self.write(n,'Acceleration',5 if self.paddle_profile else 10)
   if self.paddle_profile and n.endswith(('shoulder_lift','elbow_flex')):self.write(n,'P_Coefficient',32)
   q=self.read(n,'Present_Position');lo,hi=self.limits[n]
   if n in self.ranges and not self.ranges[n][0]+(40 if self.paddle_profile else 4)<=q<=self.ranges[n][1]-(40 if self.paddle_profile else 4):raise RuntimeError(n+': drifted before enable')
   if not 0<=q<=4095 or(lo<hi and not lo<=q<=hi):raise RuntimeError(n+': drifted outside firmware limits')
   self.write(n,'Goal_Position',q);self.enabled.add(n)
   self.write(n,'Torque_Enable',1);self.write(n,'Lock',1);self.write(n,'Goal_Position',q);self.goals[n]=q
   self.state['enable_register_diagnostics'][n]['applied']=self.register_diagnostics(n)
  self.lease=self.clock()+(120 if self.paddle_profile else 30);self.state['released']=False;self.poll()
 def release(self,n):
  self.by_name[n].disable_torque([n],num_retry=3)
  if self.read(n,'Torque_Enable')!=0:raise RuntimeError(n+': release not confirmed')
  self.rows.setdefault(n,{})['Torque_Enable']=0
  self.rows[n]['released_readback_at']=self.wall()
  self.enabled.discard(n)
  if n in self.old:
   for f in ['Torque_Limit','Goal_Velocity','Goal_Time','Acceleration','P_Coefficient','Lock']:self.write(n,f,self.old[n][f])
   self.old.pop(n)
 def release_all(self,reason,latch=True):
  errors=[]
  for n in list(self.enabled):
   try:self.release(n)
   except Exception as e:errors.append(str(e))
  if self.engine:self.engine.active=False
  self.latched=latch;self.state.setdefault('root_failure',reason) if reason not in ('Operator STOP','Hardware-owner exit','Local probe complete') else None
  self.state.update(phase='stopped' if latch else 'idle',ok=not bool(errors),operator_armed=not latch,error=self.state.get('root_failure',reason),release_errors=errors,released=not errors,stop_latched=latch);self.publish()
 def setpoints(self,goals):
  for n,q in goals.items():
   if n not in self.enabled or n.startswith('base_'):raise RuntimeError('Direct position target is not an enabled arm/head joint')
   lo,hi=self.ranges[n]
   if type(q)is not int or not lo+4<=q<=hi-4 or abs(q-self.goals[n])>(96 if self.paddle_profile else 68):raise RuntimeError('Position target leaves saved limits or sample bound')
   if q!=self.goals[n]:self.write(n,'Goal_Position',q);self.goals[n]=q
 def command(self,c):
  if type(c.get('id'))is not int or c['id']<=0 or c.get('session_started')!=self.started:raise ValueError('Command belongs to another hardware-owner session')
  op=c.get('op')
  if self.read_only and op not in ('stop','enable_motors','hold'):raise ValueError('READ_ONLY_OWNER: motion commands disabled')
  if op=='stop':self.release_all('Operator STOP');self.state['completed']=c['id'];return
  if op=='enable_motors':self.enable(c.get('names'),c.get('enabled'));self.state['completed']=c['id'];return
  if self.latched:raise ValueError('STOP is latched; new operator session required')
  if op=='local_gripper_probe':
   if self.paddle_profile:raise ValueError('Legacy diagnostic probe unavailable under pickup profile')
   if self.enabled or any(r.get('Torque_Enable')!=0 for r in self.rows.values()) or len(self.rows)!=16:raise ValueError('Probe requires all16 observed released')
   if c.get('authorization')!='one-shot-right-gripper-48' or self.state.get('probe_used'):raise ValueError('Local diagnostic authorization missing or already used')
   n='right_arm_gripper';current=int(self.rows[n]['Present_Position'])
   from gripper_response_probe import GripperResponseProbe
   probe=GripperResponseProbe(current,c.get('target_ticks'),self.ranges[n],self.setpoints,clock=self.clock,wall=self.wall)
   self.state['probe_used']=True
   self.enable([n],True)
   applied=self.state['enable_register_diagnostics'][n]['applied']['registers']
   if any(applied[f]!=v for f,v in {'Torque_Limit':250,'Goal_Velocity':100,'Acceleration':10,'P_Coefficient':32,'Operating_Mode':0}.items()):raise RuntimeError('Probe applied settings differ from authorized values')
   self.engine=probe;self.current_command=c['id'];self.state.update(probe.start(c));self.last_tick=self.clock();self.lease=self.clock()+30;self.publish();return
  if op=='hold':self.lease=self.clock()+(120 if self.paddle_profile else 30);self.state['completed']=c['id'];return
  if op not in ('direct_joint','gripper_target'):raise ValueError('Unsupported hardware command')
  positions=c.get('positions')
  if not isinstance(positions,dict) or not positions or not set(positions)<=self.enabled or not set(positions)<=set(self.position_names):raise ValueError('Targets require already-enabled arm/head motors; wheels do not accept position-motion requests')
  if self.engine and self.engine.active:raise ValueError('Previous motion has not completed')
  executor=DirectJointExecutor
  if self.paddle_profile:
   from paddle_joint_executor import PaddleJointExecutor
   executor=PaddleJointExecutor
  elif op=='gripper_target':
   from gripper_waypoint_executor import GripperWaypointExecutor
   executor=GripperWaypointExecutor
  candidate=executor(list(positions),{n:self.ranges[n] for n in positions},self.setpoints,clock=self.clock,wall=self.wall)
  current={n:self.rows[n]['Present_Position'] for n in positions}
  if self.paddle_profile:
   if set(self.position_names)!=self.enabled:raise ValueError('Pickup requires all six right-arm motors explicitly enabled')
   if self.motion_count>=20:raise ValueError('Pickup session motion budget exhausted (20 segments)')
   if not self.camera_gate.update(holding=True):raise ValueError('Pickup phone feed paused; no new target accepted')
  update=candidate.start(c,current,session_started=self.started,**({'held_goals':self.goals} if self.paddle_profile else {}))
  if self.paddle_profile:self.motion_count+=1
  self.state['pickup_motion_segments_used']=self.motion_count
  self.state['local_gripper_probe']=False
  self.engine=candidate;self.current_command=c['id'];self.state.update(update);self.lease=self.clock()+(candidate.deadline+5 if op=='gripper_target' or self.paddle_profile else 30);self.last_tick=self.clock();self.publish()

def atomic(path,value):
 temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,allow_nan=False));temp.replace(path)

def main():
 old=Path('/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20');utility=old/'work/carton-visual-controller/software';root=Path('/Users/teachera/Documents/Codex/2026-10-05/m');folder=root/'work/gemma-hardware-session';folder.mkdir(exist_ok=True)
 os.environ.update(CARTON_LIVE_SOFTWARE=str(old/'xlerobot-farm/software'),CARTON_UTILITY_SOFTWARE=str(utility),CARTON_WORKSPACE_ROOT=str(root),CARTON_PROFILE='paper-tray-v0')
 sys.path.insert(0,str(utility/'scripts/carton_robot'))
 from carton_runtime import PROFILE
 from farm.config import load_profile
 from farm.adapters.robot_lerobot import LeRobotXLeRobot
 from servo_ownership import ServoOwnership
 from strict_servo_replies import guard_replies
 from coherent_servo_telemetry import decode_telemetry, ADDRESS, LENGTH
 def observed_telemetry(bus,name):
  motor=bus.motors[name];started=time.time()
  transaction={'motor':name,'servo_id':motor.id,'model':motor.model,'port':bus.port,'address':ADDRESS,'length':LENGTH,'started_at':started}
  try:
   data,communication,packet_error=bus.packet_handler.readTxRx(bus.port_handler,motor.id,ADDRESS,LENGTH)
   transaction.update(communication=communication,packet_error=packet_error,payload_bytes=list(data),reply=getattr(bus,'last_reply_evidence',None),finished_at=time.time())
   decoded=decode_telemetry(data,communication,packet_error);bus.last_telemetry_sample=transaction;return decoded
  except Exception as exc:
   transaction.update(finished_at=time.time(),error=str(exc));bus.last_telemetry_failure=transaction
   raise RuntimeError(str(exc)+'; transaction='+json.dumps(transaction)) from exc
 r=LeRobotXLeRobot(load_profile(PROFILE).robot).robot;buses=[r.bus1,r.bus2];owner=None;ownership=None;stop=threading.Event()
 signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())
 last=None
 try:
  ownership=ServoOwnership([b.port for b in buses]).acquire()
  for b in buses:guard_replies(b);b.connect(handshake=False)
  owner=HardwareOwner(buses,r.calibration,observed_telemetry,read_only='--read-only' in sys.argv,position_scope=[n for b in buses for n in b.motors if n.startswith('right_arm_')] if '--right-arm-only' in sys.argv else None,paddle_profile='--paddle-profile' in sys.argv);owner.inspect();atomic(folder/'status.json',owner.state)
  if(folder/'command.json').exists():last=json.loads((folder/'command.json').read_text()).get('id')
  print('Hardware owner ready:16 motor reads, all torque off.',flush=True)
  while not stop.is_set():
   try:owner.poll()
   except RuntimeError as e:
    owner.state['failed_command_id']=owner.current_command
    if owner.engine:
     owner.state['direct_fault_diagnostics']=owner.engine.diagnostics
     owner.state['direct_fault_samples']=owner.engine.samples
    owner.state['fault_active_rows']={n:dict(row) for n,row in owner.rows.items()}
    owner.state['fault_at']=time.time()
    owner.release_all(str(e))
    print('Hardware command stopped: '+str(e),flush=True)
   p=folder/'command.json'
   if p.exists():
    c=json.loads(p.read_text())
    if c.get('id')!=last:
     last=c.get('id')
     try:owner.command(c)
     except ValueError as e:owner.state['last_rejected']={'id':last,'reason':str(e)}
   atomic(folder/'status.json',owner.state);time.sleep(.02)
 except BaseException as e:
  print('Hardware owner stopped: '+str(e),flush=True)
  if owner:
   owner.state['root_failure']=str(e);owner.state['failed_command_id']=owner.current_command
   owner.release_all(str(e))
 finally:
  if owner:
   owner.release_all('Hardware-owner exit');atomic(folder/'status.json',owner.state)
  for b in buses:
   if b.is_connected:b.disconnect(disable_torque=False)
  if ownership:ownership.close()
if __name__=='__main__':main()
