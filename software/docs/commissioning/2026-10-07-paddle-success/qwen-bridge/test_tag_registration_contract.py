"""Contract: the chat Mac's AprilTag registration mover against today's robot server. Fake hardware only.

Runs CalibrationRobot -> run_calibration(..., 'registration') (software/carton/servo) through TagRobot and a
stand-in for the chat client's HTTPS Robot.call, into the real gemma_robot_tools.dispatch, DirectJointClient and
HardwareOwner (--both-arms --paddle-profile --wheels, 2 s soft release) driving a fake servo bus. Camera frames are
rendered from the fake encoders (pinhole projection of tag36h11 markers), published as the OAK manifest the API
reads, and detected on the chat side. A shared virtual clock models the relay round trip and the owner loop.

Robot-Mac-only imports (carton_preflight, gemma_execution_binding, gemma_reach_planner), the saved calibration file
and the mTLS certificate are stubbed. The SO-101 forward kinematics is a synthetic two-axis model, as in
software/tests/test_gemma_calibration.py: the hand-eye fit is real, the arm model is not.

Needs numpy and OpenCV (the farm venv has them; plain python3 re-executes under XLEROBOT_CONTRACT_PYTHON or
software/.venv/bin/python). Uses pupil_apriltags when installed; otherwise a corner-lookup stand-in replaces only
the detector, so the robot-side redeploy check does not need the chat Mac's calibration requirements.
"""
import os, sys
from pathlib import Path
BRIDGE = Path(__file__).resolve().parent
SOFTWARE = BRIDGE.parents[3]
try:
    import numpy as np, cv2
except ImportError:
    for candidate in (os.environ.get('XLEROBOT_CONTRACT_PYTHON'), str(SOFTWARE/'.venv/bin/python')):
        if candidate and Path(candidate).exists() and not os.environ.get('XLEROBOT_CONTRACT_REEXEC'):
            os.environ['XLEROBOT_CONTRACT_REEXEC'] = '1'
            os.execv(candidate, [candidate, __file__, *sys.argv[1:]])
    sys.exit('test_tag_registration_contract.py needs numpy and OpenCV: set XLEROBOT_CONTRACT_PYTHON to the farm venv python')

import copy, hashlib, json, math, tempfile, time, types
from types import SimpleNamespace as C
from unittest import mock

sys.path[:0] = [str(BRIDGE), str(SOFTWARE)]
# Chat-side modules first, so the API's robot-Mac sys.path entries cannot shadow them.
from carton.servo.common import Limits
from carton.servo.tag_kit import marker_image
import carton.servo.tag_calibration as tag_calibration
from farm.kinematics.tag_registration import fit_registration
from farm.perception.gemma_tags import TagObserver, TagRobot
from farm.perception.gemma_calibration import CalibrationRobot
from farm.status import Reading, Status
sys.path.insert(0, str(SOFTWARE/'tools'))

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
RIGHT = ['right_arm_'+j for j in ARM]; LEFT = ['left_arm_'+j for j in ARM]
NAMES = RIGHT + LEFT + ['head_motor_1', 'head_motor_2', 'base_left_wheel', 'base_right_wheel']
LO, HI = 826, 3268
REST = {n: 2048 for n in NAMES}
REST.update({'right_arm_shoulder_lift': 1500, 'right_arm_elbow_flex': 2600, 'right_arm_wrist_flex': 2100})
CAL = {n: {'id': i+1, 'drive_mode': 0, 'homing_offset': 0, 'range_min': 0 if n.startswith('base_') else LO,
           'range_max': 4095 if n.startswith('base_') else HI} for i, n in enumerate(NAMES)}
W, H, F = 960, 720, 820.
K = [[F, 0, W/2], [0, F, H/2], [0, 0, 1]]
CAMERA_ID, STREAM = 'oak-sim-contract', 'oak-sim-stream-1'
TAG_MM = {1: 60., 2: 40.}
MOUNT = {'arm': 'right', 'body': 'fixed_gripper_housing', 'source': 'contract fixture'}
JOINTS = ['shoulder_pan', 'wrist_flex']


def rot(axis, degrees):
    return cv2.Rodrigues(np.asarray(axis, float)*math.radians(degrees))[0]


def pose(r, t):
    out = np.eye(4); out[:3, :3] = r; out[:3, 3] = t
    return out


def base_from_gripper(q):
    """Synthetic two-axis FK, the same model as software/tests/test_gemma_calibration.py."""
    a, b = q['right_arm_shoulder_pan']-REST['right_arm_shoulder_pan'], q['right_arm_wrist_flex']-REST['right_arm_wrist_flex']
    return pose(rot([0, 1, 0], math.degrees(b*math.pi/2048)) @ rot([0, 0, 1], math.degrees(a*math.pi/2048)),
                [.2+a*.0003, .02+b*.0003, .3])


# Tag 2 faces the camera tilted (unambiguous IPPE orientation); tag 1 lies on the table.
CAMERA_FROM_TAG2_AT_REST = pose(rot([1, 0, 0], 180) @ rot([0, 1, 0], 28) @ rot([1, 0, 0], -22), [.03, -.01, .42])
CAMERA_FROM_TAG1 = pose(rot([1, 0, 0], 180) @ rot([1, 0, 0], -35) @ rot([0, 1, 0], 15), [-.17, .12, .58])
BASE_FROM_CAMERA = pose(rot([0.3, -1, 0.2], 70), [.55, -.25, .62])
GRIPPER_FROM_TAG = np.linalg.inv(base_from_gripper(REST)) @ BASE_FROM_CAMERA @ CAMERA_FROM_TAG2_AT_REST


class Clock:
    """Wall time plus simulated waits; never behind real time (DirectJointClient.stop compares with time_ns ids)."""
    def __init__(self):self.skew = 0.
    def __call__(self):return time.time()+self.skew
    def advance(self, dt):self.skew += dt


class Bus:
    """Sixteen fake STS servos; position follows the goal at a bounded rate while torque is on."""
    def __init__(self, clock):
        self.clock, self.motors, self.updated = clock, dict.fromkeys(NAMES), clock()
        self.r = {n: dict(Torque_Enable=0, Operating_Mode=0, Homing_Offset=0, Min_Position_Limit=CAL[n]['range_min'],
                          Max_Position_Limit=CAL[n]['range_max'], Present_Position=REST[n], Goal_Position=REST[n], Lock=1,
                          Torque_Limit=1000, Goal_Velocity=0, Goal_Time=0, Acceleration=0, P_Coefficient=16, Status=0)
                  for n in NAMES}
        self.load, self.blocked, self.log = dict.fromkeys(NAMES, 40), {}, []
        self.stall_reads_stationary = False  # whether a joint stopped by an obstacle reports Moving=0
    def read(self, f, n, **kw):return self.r[n].get(f, 0)
    def write(self, f, n, v, **kw):
        self.r[n][f] = v; self.log.append((n, f, v))
    def disable_torque(self, ns, **kw):
        for n in ns:self.r[n]['Torque_Enable'] = 0
    def integrate(self):
        dt, self.updated = self.clock()-self.updated, self.clock()
        for n, r in self.r.items():
            if r['Torque_Enable'] and r['Torque_Limit']:
                goal = r['Goal_Position']
                if n in self.blocked:  # an obstacle at this encoder value, on the far side from the rest pose
                    stop = self.blocked[n]
                    goal = max(goal, stop) if stop < REST[n] else min(goal, stop)
                step = max(1, int(200*dt))
                r['Present_Position'] += max(-step, min(step, goal-r['Present_Position']))
    def telemetry(self, bus, n):
        r = self.r[n]; moving = abs(r['Goal_Position']-r['Present_Position']) > 1 and r['Torque_Enable']
        if self.stall_reads_stationary and self.blocked.get(n) == r['Present_Position']:moving = False
        return dict(Present_Position=r['Present_Position'], Present_Load=self.load[n], Present_Voltage=124,
                    Present_Temperature=31, Moving=int(bool(moving)), Present_Velocity=40 if moving else 0, Status=r['Status'])


class Camera:
    """Publishes one OAK frame per request into the manifest folder gemma_robot_tools reads."""
    def __init__(self, folder, clock, bus, real_detector):
        self.folder, self.clock, self.bus, self.seq, self.corners = folder, clock, bus, 0, {}
        self.real, self.hide_gripper = real_detector, False
        folder.mkdir(parents=True)
        self.markers = {i: cv2.cvtColor(marker_image(i, 12), cv2.COLOR_BGR2GRAY) for i in TAG_MM}
    def camera_from_tags(self):
        q = {n: r['Present_Position'] for n, r in self.bus.r.items()}
        return {1: CAMERA_FROM_TAG1, 2: np.linalg.inv(BASE_FROM_CAMERA) @ base_from_gripper(q) @ GRIPPER_FROM_TAG}
    def publish(self):
        self.seq += 1
        image = np.full((H, W), 150, np.uint8); found = {}
        for tag_id, c_t in self.camera_from_tags().items():
            if tag_id == 2 and self.hide_gripper:continue
            s = TAG_MM[tag_id]/2000
            def project(points):
                p = (c_t[:3, :3] @ np.asarray(points, float).T).T + c_t[:3, 3]
                return np.array([[F*x/z+W/2, F*y/z+H/2] for x, y, z in p])
            outer, inner = s*10/8, s
            size = self.markers[tag_id].shape[0]
            src = np.float32([[-.5, -.5], [size-.5, -.5], [size-.5, size-.5], [-.5, size-.5]])
            dst = project([[-outer, outer, 0], [outer, outer, 0], [outer, -outer, 0], [-outer, -outer, 0]])
            warp = cv2.getPerspectiveTransform(src, np.float32(dst))
            marker = cv2.warpPerspective(self.markers[tag_id], warp, (W, H), flags=cv2.INTER_AREA, borderValue=0)
            mask = cv2.warpPerspective(np.full_like(self.markers[tag_id], 255), warp, (W, H), flags=cv2.INTER_AREA, borderValue=0)
            image = (image*(1-mask/255.)+marker*(mask/255.)).astype(np.uint8)
            # The corner order pupil_apriltags decodes for this marker orientation.
            found[tag_id] = project([[inner, inner, 0], [-inner, inner, 0], [-inner, -inner, 0], [inner, -inner, 0]])
        if not self.real:
            image[0, :8] = [(self.seq >> (8*k)) & 255 for k in range(8)]  # frame identity for the lookup detector
            self.corners[self.seq] = found
        ok, data = cv2.imencode('.png' if not self.real else '.jpg', cv2.cvtColor(image, cv2.COLOR_GRAY2BGR),
                                [] if not self.real else [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        assert ok
        data = data.tobytes(); name = f'oak-{self.seq}.jpg'; captured = self.clock()-.03  # pipeline latency
        (self.folder/name).write_bytes(data)
        manifest = {'camera_id': CAMERA_ID, 'stream_id': STREAM, 'seq': self.seq, 'image': name,
                    'sha256': hashlib.sha256(data).hexdigest(), 'captured_at': captured, 'width': W, 'height': H,
                    'intrinsics': K, 'projection': 'rectified_pinhole', 'coordinate_frame': 'oak_rgb_optical',
                    'distortion_coefficients': [0, 0, 0, 0, 0]}
        tmp = self.folder/'oak.json.tmp'; tmp.write_text(json.dumps(manifest)); tmp.replace(self.folder/'oak.json')
    def lookup_detector(self, frame):
        seq = sum(int(v) << (8*k) for k, v in enumerate(frame.value[0, :8, 0]))
        found = {tag_id: {'center': corners.mean(axis=0).tolist(), 'corners': corners.tolist(), 'margin': 80., 'hamming': 0}
                 for tag_id, corners in self.corners[seq].items()}
        return Reading(found, Status.OK, frame.t, source='contract.lookup_detector')


def import_robot_tools(root, cal_path):
    """Import the real gemma_robot_tools with its robot-Mac-only dependencies stubbed."""
    stubs = {'carton_preflight': dict(run=lambda: {'error': 'contract: serial preflight unavailable'}),
             'gemma_execution_binding': dict(TrustedExecutionBinding=lambda session: C(bind=None)),
             'gemma_reach_planner': dict(POSE_SCHEMA={'type': 'array'}, validate_poses=lambda poses: None,
                 inspect_or_plan=lambda args, status, cal, calibration, cameras, plan=True: {
                     'status': 'FK_ESTIMATE_ONLY', 'motor_writes': 0, 'execution_supported': False,
                     'configuration': {'config': {'arm': args['arm'], 'mapping': 'feetech_degrees_v1', 'calibration_sha256': 'contract'},
                                       'model_assets': {'verified': True, 'revision': 'contract'}}})}
    for name, attributes in stubs.items():
        module = types.ModuleType(name); module.__dict__.update(attributes); sys.modules[name] = module
    original = Path.read_text
    def read_text(self, *args, **kwargs):
        if self.name == 'farm_xlerobot.json' and self != cal_path:return original(cal_path)
        if self.name == 'gateway-server.pem' and 'gemma-mtls' in str(self):return original(BRIDGE/'gateway-server.pem')
        return original(self, *args, **kwargs)
    # Bind the bridge's own modules first: on the robot Mac the API prepends old checkout paths.
    import gemma_direct_client, paddle_segments, remote_admin, wrist_cameras  # noqa: F401
    environment, path = dict(os.environ), list(sys.path)
    with mock.patch.object(Path, 'read_text', read_text):
        import gemma_robot_tools as tools
    os.environ.clear(); os.environ.update(environment); sys.path[:] = path
    return tools


class Rig:
    """Owner loop + API + relay stand-in, all on one virtual clock. rtt_s is the chat<->robot round trip."""
    def __init__(self, tmp, tools, *, rtt_s=.15, chat_offset_s=0., soft_release_s=2.):
        from gemma_hardware_owner import HardwareOwner, atomic
        from gemma_direct_client import DirectJointClient
        self.tmp, self.tools, self.rtt, self.atomic = tmp, tools, rtt_s, atomic
        self.clock = Clock(); self.bus = Bus(self.clock)
        self.chat_clock = lambda: self.clock()+chat_offset_s
        self.session = tmp/'work/gemma-hardware-session'; self.session.mkdir(parents=True)
        self.phone = {'fresh': True, 'seq': 0}
        def phone():
            self.phone['seq'] += 1
            return {'received_at': self.clock() if self.phone['fresh'] else self.clock()-60, 'seq': self.phone['seq']}
        self.owner = HardwareOwner([self.bus], {n: C(range_min=c['range_min'], range_max=c['range_max'], homing_offset=0) for n, c in CAL.items()},
                                   self.bus.telemetry, clock=self.clock, wall=self.clock, position_scope=RIGHT+LEFT, paddle_profile=True,
                                   camera_metadata=phone, wheels=True, soft_release_s=soft_release_s, sleep=self.clock.advance)
        self.owner.inspect(); atomic(self.session/'status.json', self.owner.state)
        self.last_command, self.last_tick, self.calls, self.faults, self.steps = None, self.clock(), [], [], []
        self.camera = Camera(tmp/'work/oak-rectified-stream', self.clock, self.bus, real_detector=REAL_DETECTOR)
        tools.ROOT, tools.SESSION = tmp, self.session
        tools.OAK_RECTIFIED_DIR, tools.OAK_RAW_DIR = self.camera.folder, tmp/'work/oak-stream'
        tools.time = C(time=self.clock, sleep=self.sleep, time_ns=time.time_ns, monotonic=self.clock)
        tools.DIRECT_CLIENT = DirectJointClient(self.session, CAL, clock=self.clock, sleep=self.sleep)
        tools.REQUESTS.clear()
        self.after_owner_step = None
    def sleep(self, dt):
        self.clock.advance(dt); self.pump()
    def pump(self):
        """One iteration of gemma_hardware_owner.main's loop once its ~50 ms period has passed."""
        if self.clock()-self.last_tick < .05:return
        self.last_tick = self.clock(); self.bus.integrate(); o = self.owner
        try:o.poll()
        except RuntimeError as e:
            o.state['failed_command_id'] = o.current_command; o.release_all(str(e)); self.faults.append(str(e))
        p = self.session/'command.json'
        if p.exists():
            c = json.loads(p.read_text())
            if c.get('id') != self.last_command:
                self.last_command = c.get('id')
                try:o.command(c)
                except ValueError as e:o.state['last_rejected'] = {'id': self.last_command, 'reason': str(e)}
        if self.after_owner_step:self.after_owner_step(self)
        self.atomic(self.session/'status.json', o.state)
    # The chat Mac's chat_server.Robot: GET /tools and POST /call over the relay.
    def catalog(self):
        return json.loads(json.dumps({'tools': self.tools.TOOLS}))
    def call(self, name, args, request_id=None):
        self.calls.append((name, copy.deepcopy(args), self.clock()))
        self.sleep(self.rtt/2)
        if name == 'robot_move_motor_targets':
            self.steps += [(n, q-self.bus.r[n]['Present_Position']) for n, q in args['positions'].items()]
        if name == 'robot_get_cameras':self.camera.publish()
        try:
            result, images = self.tools.dispatch(name, copy.deepcopy(args))
            body = {'ok': True, 'result': result, **({'images': images} if images is not None else {})}
            body = json.loads(json.dumps(body, allow_nan=False))
        except (ValueError, KeyError, TypeError) as e:
            body = {'ok': False, 'http_status': 400, 'result': {'ok': False, 'result': {'error': str(e)}}}
        except Exception as e:
            body = {'ok': False, 'http_status': 409, 'result': {'ok': False, 'result': {'error': str(e)}}}
        self.sleep(self.rtt/2)
        return body
    def moves(self):
        return [a for n, a, _ in self.calls if n == 'robot_move_motor_targets']


def geometry():
    return {'schema': 1, 'family': 'tag36h11', 'camera_ids': [CAMERA_ID],
            'tags': {'1': {'black_square_mm': TAG_MM[1], 'source': 'contract fixture'},
                     '2': {'black_square_mm': TAG_MM[2], 'source': 'contract fixture', 'mount': MOUNT}}}


def chat(rig, folder, config_extra=None):
    """The chat Mac side exactly as tools/calibrate_gemma_tags.py builds it."""
    folder.mkdir(parents=True, exist_ok=True)
    config = dict(schema=1, arm='right', joints=JOINTS, camera='oak', model_directory='contract-model', **(config_extra or {}))
    (folder/'tag-calibration.json').write_text(json.dumps(config))
    detector = None if REAL_DETECTOR else rig.camera.lookup_detector
    observer = TagObserver(clock=rig.chat_clock, geometry=geometry(), **({'detector': detector} if detector else {}))
    robot = CalibrationRobot(TagRobot(rig, observer=observer), folder/'tag-calibration.json', clock=rig.chat_clock)
    robot.catalog()
    return robot


def synthetic_dataset(captures, directory):
    """Replaces only assemble_dataset's SO-101 model lookup with the synthetic FK above."""
    samples = []
    for capture in captures:
        sample = copy.deepcopy(capture['sample'])
        sample['base_from_gripper'] = base_from_gripper(sample['joint_ticks']).tolist()
        samples.append(sample)
    binding = dict(arm='right', gripper_tag_id=2, gripper_tag_mount=MOUNT, camera_id=CAMERA_ID, stream_id=STREAM,
                   camera_calibration_sha256=samples[0]['camera_calibration_sha256'], tag_geometry_sha256=samples[0]['tag_geometry_sha256'],
                   robot_model_sha256='contract-model', motor_calibration_sha256='contract', encoder_mapping_source='synthetic contract FK')
    return dict(schema=1, samples=samples, binding=binding)


try:
    import pupil_apriltags  # noqa: F401
    REAL_DETECTOR = os.environ.get('XLEROBOT_CONTRACT_DETECTOR') != 'lookup'
except ImportError:
    REAL_DETECTOR = False
HAND_EYE = hasattr(cv2, 'calibrateHandEye')
checks = []
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    cal_path = tmp/'farm_xlerobot.json'; cal_path.write_text(json.dumps(CAL))
    tools = import_robot_tools(tmp, cal_path)
    fits = []
    def fit_after_release(dataset):
        fits.append(sorted(CURRENT.owner.enabled))
        if not HAND_EYE:  # OpenCV 5 wheels lack calibrateHandEye; the mover contract is still exercised.
            return {'schema': 1, 'status': 'REGISTRATION_VALIDATED', 'residuals': {}, 'motion_ready': False, 'motor_writes': 0, 'binding': dataset['binding']}
        return fit_registration(dataset)
    tag_calibration.assemble_dataset = synthetic_dataset
    tag_calibration.fit_registration = fit_after_release

    def run(label, rtt=.15, chat_offset=0., setup=None, hook=None):
        global CURRENT
        rig = CURRENT = Rig(tmp/label, tools, rtt_s=rtt, chat_offset_s=chat_offset)
        if setup:setup(rig)
        rig.after_owner_step = hook
        robot = chat(rig, tmp/label/'pilot/.private')
        started = rig.clock()
        response = robot.call('robot_calibrate_tags', {'mode': 'registration'})
        return rig, robot, response, rig.clock()-started

    def succeeded(rig, response):
        assert response['ok'], (json.dumps(response)[:3000], [(n, round(t, 3)) for n, _, t in rig.calls][-30:])
        result = response['result']
        assert result['status'] == 'REGISTRATION_VALIDATED', result
        if HAND_EYE:
            assert result['residuals']['train']['count'] == 8 and result['residuals']['validation']['count'] == 3
            error = np.linalg.norm(np.array(result['base_from_camera'])[:3, 3]-BASE_FROM_CAMERA[:3, 3])*1000
            assert error < 10, error  # fixture sanity; the fit itself passed its held-out residual limits
        assert len(list(Path(result['output']).glob('pose-*.json'))) == 11
        enables = [a for n, a, _ in rig.calls if n == 'robot_set_motor_enable']
        assert enables == [{'names': RIGHT, 'enabled': True}, {'names': RIGHT, 'enabled': False}], enables
        assert rig.steps and all(j in ['right_arm_'+x for x in JOINTS] and 3 <= abs(d) <= 16 for j, d in rig.steps), rig.steps
        assert fits[-1] == []  # the fit ran after every motor was released
        assert not rig.owner.enabled and all(r['Torque_Enable'] == 0 for r in rig.bus.r.values())
        assert all(abs(rig.bus.r[n]['Present_Position']-REST[n]) <= 5 for n in RIGHT)
        assert rig.owner.state['stop_count'] == 0 and not any(n == 'robot_stop' for n, _, _ in rig.calls) and not rig.faults
        assert result['cleanup']['release_confirmed'] is True
        return result

    def failed_safely(rig, response, reason):
        assert not response['ok'] and reason in response['result']['error'], response
        failure = json.loads(next((rig.tmp/'pilot').glob('tag-calibration-runs/*/failure.json')).read_text())
        assert failure['automatic_retry'] is False and failure['cleanup']['release_confirmed'] is True, failure
        assert [n for n, _, _ in rig.calls].count('robot_stop') == 1
        assert not rig.owner.enabled and all(r['Torque_Enable'] == 0 for r in rig.bus.r.values())
        return failure

    # 1. Registration through the relay at 150 ms: status, enable all six, 11 observed poses, return, release, fit.
    rig = CURRENT = Rig(tmp/'status', tools)
    status = chat(rig, tmp/'status/pilot/.private').call('robot_calibration_status', {})
    assert status['ok'] and status['result']['status'] == 'READY_FOR_LOCAL_PROBES' and not status['result']['blockers'], status
    assert not rig.bus.log and not any(n.startswith(('robot_set', 'robot_move', 'robot_stop')) for n, _, _ in rig.calls)
    rig, _, response, elapsed = run('relay', float(os.environ.get('CONTRACT_RTT', .15)))
    succeeded(rig, response)
    assert elapsed < Limits().max_seconds, elapsed
    projection = tag_calibration.registration_timing(rig.rtt)
    assert projection['steps'] == len(rig.steps) and abs(projection['relay_calls']-len(rig.calls)) <= .05*len(rig.calls), (projection, len(rig.calls))
    assert abs(projection['projected_seconds']-elapsed) <= .15*elapsed, (projection, elapsed)
    checks.append(f'relay {rig.rtt*1000:.0f} ms: {len(rig.steps)} steps, {len(rig.calls)} calls, {elapsed:.0f} s (projected {projection["projected_seconds"]:.0f} s)')

    # 2. Same run on a link faster than the owner's ~50 ms poll: the frame bracket re-reads instead of failing.
    rig, _, response, elapsed = run('fast-link', .03)
    succeeded(rig, response)
    checks.append(f'fast link 30 ms: {elapsed:.0f} s')

    # 3. Owner fault while holding (servo status error): owner soft-releases, mover stops and confirms, no retry.
    def fault(rig):
        if len(rig.steps) >= 5 and not rig.faults:rig.bus.r['right_arm_elbow_flex']['Status'] = 8
    rig, _, response, _ = run('owner-fault', hook=fault)
    failed_safely(rig, response, '')
    assert rig.faults and 'Status=8' in rig.faults[0] and rig.owner.state['stop_count'] >= 2 and len(rig.steps) <= 6, (rig.faults, rig.owner.state['stop_count'], len(rig.steps), response)
    checks.append('owner fault: soft release, STOP confirmed, no retry')

    # 4. Gripper tag lost while holding: the mover's STOP eases torque off over 2 s (soft release) and confirms it.
    def hide(rig):
        if len(rig.steps) >= 3:rig.camera.hide_gripper = True
    rig, _, response, _ = run('tag-lost', hook=hide)
    failure = failed_safely(rig, response, 'gripper tag 2')
    lift = [v for n, f, v in rig.bus.log if n == 'right_arm_shoulder_lift' and f == 'Torque_Limit']
    assert lift[-11:] == [720, 640, 560, 480, 400, 320, 240, 160, 80, 0, 1000], lift
    assert rig.owner.state['last_release_mode'] == 'soft' and rig.owner.state['last_stop']['reason'] == 'Operator STOP'
    assert failure['cleanup']['stop']['release_confirmed'] is True and len(rig.steps) == 3
    checks.append('tag lost: STOP soft release (2 s ramp) confirmed')

    # 5. Obstacle 8 ticks short of a step. If the stalled servo reads stationary, the owner reports
    # endpoint_settled (inside its 57-tick tolerance) and the mover's 5-tick check refuses; if it keeps
    # Moving=1, the owner faults at its settle deadline. Either way: one STOP, confirmed release, no retry.
    for stationary, reason in ((True, 'misses calibration tolerance'), (False, 'failed to settle')):
        def obstacle(rig):
            rig.bus.blocked['right_arm_shoulder_pan'] = REST['right_arm_shoulder_pan']-40
            rig.bus.stall_reads_stationary = stationary
        rig, _, response, _ = run(f'obstacle-{stationary}', setup=obstacle)
        failed_safely(rig, response, reason)
        assert len(rig.steps) == 3, (stationary, rig.steps, response)
    checks.append('obstacle: endpoint_settled 8 ticks short or settle-deadline fault; mover stopped')

    # 6. Phone feed stale: the owner refuses enable; the mover sends STOP and no move.
    def stale(rig):rig.phone['fresh'] = False
    rig, _, response, _ = run('phone-stale', setup=stale)
    failed_safely(rig, response, 'phone feed')
    assert not rig.steps and not rig.moves()
    checks.append('stale phone feed: enable refused, no motion')

    # 7. Clocks: this Mac 0.9 s ahead or 0.2 s behind the robot. Freshness limits refuse before any write.
    for label, offset in (('clock-ahead', .9), ('clock-behind', -.2)):
        rig, robot, response, _ = run(label, chat_offset=offset)
        assert not response['ok'] and 'stale or host clocks differ' in response['result']['error'], response
        assert not any(n.startswith(('robot_set', 'robot_move', 'robot_stop')) for n, _, _ in rig.calls) and not rig.bus.log
        report = robot.call('robot_calibration_status', {})['result']
        assert report['status'] == 'BLOCKED' and any('host clocks differ' in b for b in report['blockers']), report
    checks.append('clock offsets +0.9 s / -0.2 s: refused before any write')

    # 8. Link tool (tools/measure_robot_link.py) recovers the offset and flags a clock that is behind.
    import measure_robot_link
    for offset, verdict in ((0., 'LINK_FITS_REGISTRATION'), (-.2, 'FIX_LINK_OR_CLOCKS_FIRST')):
        rig = Rig(tmp/f'link{offset}', tools, chat_offset_s=offset)
        report = measure_robot_link.summarize(measure_robot_link.measure(rig, 12, clock=rig.chat_clock, pause=0))
        measured = report['clock_offset_robot_minus_chat_s']['median']
        assert abs(measured+offset) <= rig.rtt/2+.02 and abs(report['round_trip_s']['median']-rig.rtt) < .05, report
        assert report['verdict'] == verdict and report['motor_writes'] == 0 and not rig.bus.log, report
    checks.append('link tool: offset within rtt/2, verdicts')

print({'tag_registration_contract': checks, 'detector': 'pupil_apriltags' if REAL_DETECTOR else 'corner lookup',
       'hand_eye_fit': HAND_EYE, 'hardware_access': False})
