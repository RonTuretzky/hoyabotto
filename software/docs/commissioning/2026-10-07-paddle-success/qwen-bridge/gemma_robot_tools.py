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
            'camera_status': camera_status(), 'saved_motor_readiness': saved_motor_readiness(),
            'automatic_motion_on_startup': False, 'passive_recovery': PASSIVE_RECOVERY,
            'motion_units': 'encoder_ticks; positioning-joint degrees use4095 ticks/rev',
            'commandable_ranges': commandable_ranges(),
            'canonical_position_joint_names': POSITION_NAMES,
            'arm_joint_aliases': ARM_ALIASES,
            'ranges': {n: {'min_ticks': v['range_min'], 'max_ticks': v['range_max']} for n, v in CAL.items()},
            'read_tools_ready': True, 'stop_tool_ready': True,
            'controller': 'DirectJointClient atomic file handoff -> canonical sole-owner direct_joint interpolator',
            'direct_joint_requirements': ['existing healthy authorized hardware owner in direct_joint mode',
                'requested joints in explicitly supported owner scope',
                'actual saved range margins and owner speed/acceleration/torque/health/watchdog checks',
                'measured hardware completion feedback; sensor status reported independently'],
            'continuous_profile_required_for_direct_joint': False,
            'cartesian_transform_required_for_direct_joint': False,
            'cartesian_pickup_additional_requirements': ['validated spatial target registration and actual tool/contact offset when requesting a Cartesian plan'],
            'wheel_policy': 'enable/release position-mode hold at current encoder only; wheel motion unavailable', 'automatic_motor_activation': False, 'remote_start_or_stop_reset_supported': False,
            'blockers': ready['blockers']}



def bounded_json(path):
    if not path.exists():
        return {'available': False}
    if path.stat().st_size > 512 * 1024:
        return {'available': True, 'omitted': 'larger than bounded metadata size'}
    return {'available': True, 'file_mtime': path.stat().st_mtime,
            'evidence_age_s': time.time() - path.stat().st_mtime,
            'data': json.loads(path.read_text())}


def readiness():
    direct = DIRECT_CLIENT.readiness()
    return {'mode': 'direct_joint', 'control_mode': 'direct_joint',
            'execution_adapter_bound': True, 'motion_ready': direct['motion_ready'],
            'available_to_accept_authorized_command': direct['available_to_accept_authorized_command'],
            'execution_binding': direct, 'joint_blockers': direct['joint_blockers'],
            'saved_calibration': CAL,
            'calibration_writes': False, 'reference_evidence': {
        n: bounded_json(ROOT / 'outputs' / n) for n in (
            'Arm-Reference-Check.json', 'Reference-Reconciliation.json',
            'Left-Arm-Measured-Reference.json', 'Reach-Integration-State.json')},
        'physical_blockers': capabilities()['blockers'],
        'twenty_degree_candidate': bounded_json(ROOT / 'outputs/Twenty-Degree-Wrist-Candidate.json'),
        'camera_status': camera_status(),
        'certificate_renewal_preparation': bounded_json(ROOT / 'outputs/Gemma-Certificate-Renewal-Preparation.json'),
        'existing_calibration_audit': bounded_json(ROOT / 'outputs/Gemma-Existing-Calibration-Audit.json')}


def keyframes():
    import yaml
    result = {}
    for label, path in [('deployment', UTILITY / 'data/keyframes.yaml'),
                        ('simulation', UTILITY / 'data-carton-sim/keyframes-sim.yaml')]:
        result[label] = {'available': path.exists(), 'simulated': label == 'simulation'}
        if path.exists() and path.stat().st_size < 512 * 1024:
            result[label]['keyframes'] = yaml.safe_load(path.read_text()) or {}
    return {'stores': result, 'blind_replay_authorized': False}


def skills():
    return {'validated_physical_skills': [], 'commissioned_skill_registry': False,
            'source_modules': ['carton.servo.skills.save_alignment', 'farm.skills.arm',
                               'farm.skills.runner', 'farm.skills.keyframes'],
            'validation_contract': 'VERIFIED_VISUAL_ALIGNMENT_ONLY requires matching guarded result, fingerprint, final trace, three fresh observations and errors within tolerance',
            'grasp_verified': False, 'execution_authorized': False}


def evidence():
    fixed = ['Reach-Integration-State.json', 'Paddle-Camera-Target-Assessment.json',
             'Paddle-Measurement-Packet/measurement.json', 'Gemma-Bridge-Preparation.json',
             'Gemma-Existing-Calibration-Audit.json']
    return {'evidence': {n: bounded_json(ROOT / 'outputs' / n) for n in fixed},
            'recordings': {n: bounded_json(ROOT / 'work' / n / 'status.json') for n in
                          ['paddle-reposition-session', 'paddle-wrist-session', 'paddle-response-session']},
            'physical_task_completed': False}


def depth_snapshot():
    for _ in range(15):
        folder, m, source = select_oak_manifest()
        path = manifest_image_path(folder, m['depth_image'])
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != m['depth_sha256']:
            time.sleep(.02)
            continue
        if not 0 <= time.time() - m['captured_at'] <= 1:
            raise RuntimeError('Depth frame is stale')
        return {'manifest': m, 'source': source, 'powered_depth_observer_ready': False,
                'reason': 'RGB-depth registration, robot transform and physical depth commissioning remain unverified'}, [
            {'camera_id': m['camera_id'] + ':depth', 'mime_type': 'image/png',
             'captured_at': m['depth_captured_at'], 'received_at': None, 'stream_id': m['stream_id'],
             'seq': m['seq'], 'sha256': m['depth_sha256'],
             'data_base64': base64.b64encode(data).decode()}]
    raise RuntimeError('Consistent depth snapshot unavailable')


def validate_arguments(name, args):
    if name not in SCHEMAS or not isinstance(args, dict):
        raise ValueError('Unknown tool or invalid arguments')
    schema = SCHEMAS[name]
    if set(args) - set(schema['properties']) or set(schema['required']) - set(args):
        raise ValueError('Unexpected or missing tool arguments')
    for key, value in args.items():
        if name == 'robot_plan_reach' and key == 'tool_poses':
            validate_poses(value)
            continue
        spec = schema['properties'][key]
        kind = spec['type']
        if kind == 'boolean' and type(value) is not bool:
            raise ValueError('Boolean argument required')
        if kind == 'integer' and type(value) is not int:
            raise ValueError('Integer ticks required')
        if kind == 'number' and (type(value) not in (int, float) or not __import__('math').isfinite(value)):
            raise ValueError('Finite numeric argument required')
        if kind == 'number' and ('maximum' in spec and value > spec['maximum'] or 'exclusiveMinimum' in spec and value <= spec['exclusiveMinimum']):
            raise ValueError('Numeric argument outside schema bounds')
        if kind == 'string' and (not isinstance(value, str) or value not in spec.get('enum', [value])):
            raise ValueError('Unsupported argument value')
        if kind == 'object' and (not isinstance(value, dict) or not value or any(type(v) is not int for v in value.values())):
            raise ValueError('Integer joint target object required')
        if kind == 'array':
            if not isinstance(value, list) or not spec.get('minItems',1) <= len(value) <= spec.get('maxItems',16) or any(not isinstance(v,str) for v in value) or len(set(value)) != len(value) or any(v not in spec['items']['enum'] for v in value):
                raise ValueError('Select distinct supported argument names')


def dispatch(name, args):
    validate_arguments(name, args)
    if name in ('robot_plan_reach', 'robot_get_arm_pose'):
        return inspect_or_plan(args, execution(), CAL, calibration, camera_status(), plan=name == 'robot_plan_reach'), None
    if name == 'robot_list_motors':
        return {'state': state(not PASSIVE_RECOVERY), 'saved_calibration': CAL, 'commandable_ranges': commandable_ranges(), 'readiness': DIRECT_CLIENT.readiness()}, None
    if name == 'robot_set_motor_enable':
        return DIRECT_CLIENT.set_motor_enable(args['names'], args['enabled']), None
    if name == 'robot_move_motor_targets':
        return DIRECT_CLIENT.execute(normalize_targets(args['positions']), args['duration_s']), None
    if name == 'robot_get_state':
        return state(args.get('fresh', True)), None
    if name == 'robot_get_cameras':
        return cameras(args.get('cameras', ['oak', 'phone']))
    if name == 'robot_get_capabilities':
        return capabilities(), None
    if name == 'robot_get_depth':
        return depth_snapshot()
    if name == 'robot_get_handoff':
        return {'source': 'primary-saved Gemma-Paddle-Handoff.json (historical task)',
                'current_user_scope': 'Verified physical pickup procedure for Qwen integration; current server recovery is read-only, with no movement test',
                'handoff': bounded_json(ROOT / 'outputs/Gemma-Paddle-Handoff.json'),
                'physical_pickup_handoff': bounded_json(ROOT / 'outputs/Qwen-Paddle-Success-Handoff.json'),
                'integration_reference': 'https://github.com/RonTuretzky/xlerobot-farm/blob/main/software/docs/commissioning/2026-10-07-paddle-success/README.md',
                'retrieved_at': time.time(), 'camera_status_now': camera_status(),
                'saved_motor_readiness_now': saved_motor_readiness(),
                'execution_binding_now': DIRECT_CLIENT.readiness(),
                'certificate_renewal_preparation': bounded_json(ROOT / 'outputs/Gemma-Certificate-Renewal-Preparation.json'),
                'existing_calibration_audit': bounded_json(ROOT / 'outputs/Gemma-Existing-Calibration-Audit.json'),
                'delivery_confirmation': bounded_json(ROOT / 'outputs/Gemma-Handoff-Delivery-Confirmation.json'),
                'context_transfer_arms_execution': False, 'motor_writes': 0}, None
    if name == 'robot_get_readiness':
        return readiness(), None
    if name == 'robot_get_keyframes':
        return keyframes(), None
    if name == 'robot_get_skills':
        return skills(), None
    if name == 'robot_get_evidence':
        return evidence(), None
    if name == 'robot_get_execution':
        return execution(), None
    if name == 'robot_stop':
        return DIRECT_CLIENT.stop(), None
    # No model request can install/arm a binding, start an owner, alter safeguards or access serial.
    if name in ('robot_move_joint_targets', 'robot_move_head'):
        args = dict(args)
        args['positions'] = normalize_targets(args['positions'], arm=args.get('arm'), head=name == 'robot_move_head')
    if name == 'robot_set_gripper':
        n = args['arm'] + '_arm_gripper'
        if not CAL[n]['range_min'] + 4 <= args['position_ticks'] <= CAL[n]['range_max'] - 4:
            b=commandable_ranges()[n]
            raise ValueError(f"Gripper target out of bounds: {n}={args['position_ticks']}; valid inclusive range [{b['min_ticks']}, {b['max_ticks']}] ticks; readiness={json.dumps(DIRECT_CLIENT.readiness())}")
    if name == 'robot_move_joint_targets':
        return DIRECT_CLIENT.execute(args['positions'], args['duration_s']), None
    if name == 'robot_move_head':
        return DIRECT_CLIENT.execute(args['positions'], args.get('duration_s', 3)), None
    if name == 'robot_set_gripper':
        return DIRECT_CLIENT.set_gripper(args['arm'], args['position_ticks'], args.get('duration_s', 3)), None
    return {'accepted': False, 'motor_writes': 0, 'reason': 'UNSUPPORTED_OWNER_SCOPE_OR_WHEELS_DISABLED',
            'requested_tool': name, 'readiness': capabilities()}, None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def peer_ok(self):
        return hashlib.sha256(self.connection.getpeercert(binary_form=True) or b'').hexdigest() == PEER_SHA

    def send_json(self, status, body):
        data = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not self.peer_ok():
            return self.send_json(403, {'ok': False, 'error': 'Unpinned mTLS client'})
        if self.path == '/health':
            return self.send_json(200, {'ok': True, 'service': 'xlerobot-private-tools', 'mode': 'direct_joint', 'control_mode': 'direct_joint',
                                        'execution_adapter_bound': True, 'motion_ready': DIRECT_CLIENT.readiness()['motion_ready'],
                                        'available_to_accept_authorized_command': DIRECT_CLIENT.readiness()['available_to_accept_authorized_command'],
                                        'armed': DIRECT_CLIENT.readiness()['local_operator_gate'],
                                        'execution_binding': DIRECT_CLIENT.readiness(),
                                        'camera_status': camera_status(),
                                        'client_certificate_pinned': True, 'motor_owner_active': execution()['active']})
        if self.path == '/tools':
            return self.send_json(200, {'ok': True, 'tools': TOOLS})
        return self.send_json(404, {'ok': False, 'error': 'Unknown route'})

    def do_POST(self):
        if not self.peer_ok():
            return self.send_json(403, {'ok': False, 'error': 'Unpinned mTLS client'})
        if self.path != '/call':
            return self.send_json(404, {'ok': False, 'error': 'Unknown route'})
        request_id = None
        fingerprint = None
        reserved = False
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024 * 1024:
                raise ValueError('Invalid request size')
            req = json.loads(self.rfile.read(size), parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
            if set(req) != {'name', 'arguments', 'request_id'}:
                raise ValueError('Expected name,arguments,request_id only')
            request_id = req['request_id']
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
                raise ValueError('Unique request_id string required')
            fingerprint = hashlib.sha256(json.dumps(req, sort_keys=True).encode()).hexdigest()
            with CACHE_LOCK:
                if request_id in REQUESTS:
                    old_hash, result = REQUESTS[request_id]
                    if old_hash != fingerprint:
                        raise ValueError('request_id reused for different request')
                    return self.send_json(200, result)
                REQUESTS[request_id] = (fingerprint, {'ok': False, 'result': {'reason': 'REQUEST_IN_PROGRESS'}})
                reserved = True
                while len(REQUESTS) > 64: REQUESTS.popitem(last=False)
            result, images = dispatch(req['name'], req['arguments'])
            body = {'ok': True, 'result': result}
            if images is not None:
                body['images'] = images
            with CACHE_LOCK:
                REQUESTS[request_id] = (fingerprint, body)
                while len(REQUESTS) > 64:
                    REQUESTS.popitem(last=False)
            return self.send_json(200, body)
        except (ValueError, KeyError, TypeError) as e:
            body = {'ok': False, 'result': {'error': str(e)}}
            if reserved:
                with CACHE_LOCK: REQUESTS[request_id] = (fingerprint, body)
            return self.send_json(400, body)
        except Exception as e:
            body = {'ok': False, 'result': {'error': str(e),
                'motor_writes': ('not_observed_by_bridge' if reserved and req.get('name') in ('robot_move_joint_targets', 'robot_move_head', 'robot_set_gripper', 'robot_set_motor_enable', 'robot_move_motor_targets') else 0)}}
            if reserved:
                with CACHE_LOCK: REQUESTS[request_id] = (fingerprint, body)
            return self.send_json(409, body)


def main():
    (ROOT / 'outputs/Gemma-Tool-Schemas.json').write_text(json.dumps(TOOLS, indent=2))
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(str(TLS / 'robot-dual.pem'), str(TLS / 'robot-dual.key'))
    ctx.load_verify_locations(cafile=str(TLS / 'gateway-server.pem'))
    ctx.verify_mode = ssl.CERT_REQUIRED
    servers = []
    for host in ('127.0.0.1',):
        server = ThreadingHTTPServer((host, 1241), Handler)
        server.daemon_threads = True
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
        servers.append(server)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    print('mTLS tools ready: https://127.0.0.1:1241 via approved TCP relay; exact client certificate pinned; DIRECT_JOINT client installed; no owner started', flush=True)
    try:
        threading.Event().wait()
    finally:
        for server in servers:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    main()
