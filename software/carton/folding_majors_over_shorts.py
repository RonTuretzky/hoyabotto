"""Close both majors over shorts retained by one open right claw (offline).

Executed continuation of the open-claw component: the right claw holds both
shorts near 90 degrees and the left hand is free. Every motion is
joint-driven physics with fresh RGB-D angles, the original collision,
penetration, tracking and carton-drift gates, and stops at the first refusal.
A held fold is not taping or hands-clear retention.

Order (each step was forced by an executed failure of the alternatives):
A. the left jaw tip presses on the far top edge, located from the far flap's
   own depth pixels, and drags it to a pinning angle. From the near side the
   5-DOF arm can neither pinch this edge nor reach behind it while the near
   flap is outward, and registration-placed contacts slid off the 3 mm edge;
B. the right claw lifts off both shorts and parks; the far major alone keeps
   them closed;
C. the left jaw releases the far major and parks. Held near 34 degrees its
   forearm lies in the near flap's sweep, and held near 60 degrees the far
   panel blocks the right claw's release. The released far springs back but
   still bounds the shorts' far corners;
D. the right jaw pushes the near outer face, 25 mm below its tip, to 90
   degrees (at the tip a few millimetres of registration error loaded the
   hinge axially);
E. with the near flap closed nothing blocks the left arm: it pushes the far
   outer face the same way to 90 degrees while the right jaw holds the near.
"""
import math

import numpy as np

from carton.folding_far_contact import FarContactGeometry, RobotVertexIK, _verify_target_angle
from carton.folding_hinge_vision import CARDBOARD_THICKNESS_M
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_progress import ContactProgressGuard
from carton.folding_sim import W, H


_PARK = dict(left=np.array([-.20, -.18, .30]), right=np.array([.20, -.18, .30]))
_FAR_EDGE_ALONG = (-.16, -.15, -.14, -.13, -.12)
# Contact positions along each major, preferred first. The right jaw keeps to
# the near flap's right side and the left jaw to the far flap's left side: a
# right wrist holding the near flap near the centre line lies in the closing
# far flap's sweep (it stalled the far at 60 degrees and twisted the carton).
_PUSH_ALONG = dict(long_near=(.12, .08, .04, .16), long_far=(-.12, -.16, -.08, -.04))
_PUSH_ARM = dict(long_near='right', long_far='left')
# Outer-face push radii, preferred first: 25 mm below the 140 mm tip, then up
# to the hook just behind the tip. From the near side a nearly upright far
# outer face is reachable only at the tip; lower contacts become reachable as
# it turns toward the robot.
_PUSH_RADII = dict(long_near=(.115,), long_far=(.115, .13, .14))
# Press of the jaw tip below the measured far top edge: start and maximum.
_PRESS_START_M, _PRESS_MAX_M = 0., .003
# Outward bias of the far top-edge contact from the measured midplane: start, maximum.
_EDGE_OUTWARD_START_M, _EDGE_OUTWARD_MAX_M = .002, .005
# Clear far-edge arm poses are rare (the upper arm passes close to the outward
# near flap), so contact searches try this many random IK seeds.
_IK_SEEDS = 40
# Push refusals that only mean "this contact no longer works": nothing was
# executed (pre-check refusals) or the flap stopped following. Physical stops
# are never retried.
_RECONTACTS = 3
_RECOVERABLE = ('stalled', 'proposal collides', 'path collides', 'pre-contact pose', 'IK exceeds 8 mm',
                'No clear')
_NEVER_RETRY = ('penetration', 'Forbidden', 'Loaded', 'Carton moved', 'Carton rotated', 'tracking',
                'Short closure')
# Independent physics stop band for the shorts during motion. A closing major
# can press a short slightly below flat into the empty carton; the final hold
# still requires 85-95 degrees.
_SHORTS_BAND = (80., 100.)
# If the friction drag slips after the far flap has started inward, a hook
# behind its top edge (reachable past about 4 degrees) takes over.
_HOOK_MIN_DEGREES = 4.
_HOOK_RADII = (.125, .13, .135, .14)


def _push_planner(sim, side, panel, allowed=()):
    """6 mm path check for a push command that ignores only the pushed panel.

    The pushed panel moves with the push, so testing the command against its
    current pose refuses every push deeper than 1 mm. It is excluded from this
    pre-check only; the runtime 1 mm robot/flap penetration stop still applies
    to it during the executed motion, and every other obstacle is checked.
    """
    planner = JointPathPlanner(sim, side, clearance=.006, allowed_flaps=allowed)
    if planner.model is sim.model:
        raise ValueError('Push pre-check requires a separate planning model copy')
    gid = planner.model.geom(panel).id
    planner.model.geom_contype[gid] = 0
    planner.model.geom_conaffinity[gid] = 0
    return planner


def _clear_pre(sim, ik, side, contact, seed, box, offsets_box):
    """First pre-contact pose, offset in box axes, that passes 6 mm transit clearance."""
    planner = JointPathPlanner(sim, side, clearance=.006)
    for offset in offsets_box:
        pre = contact + box[:3, :3] @ np.asarray(offset, dtype=float)
        try:
            q, _ = ik.solve(pre, seed)
        except ValueError:
            continue
        if planner.valid(q):
            return pre, q
    raise ValueError(f'No {side} pre-contact pose passes 6 mm transit clearance: {planner.last_collision}')


class _ContactAngleBridge:
    """Bounded encoder/CAD flap angle across a camera's edge-on band.

    A fixed camera sees each major nearly edge-on somewhere in its sweep
    (station: near about 20--25 degrees, far about 38--46). While a jaw is
    pushing the flap, the flap angle follows from the jaw's encoder FK, the
    CAD contact point and the fresh carton registration. It is used only when
    the visual angle is missing, only after it agreed with vision within 3
    degrees, for at most 12 consecutive commands, with the original 15 mm /
    8 degree carton-drift limits on every fresh registration; the next visual
    angle must agree within 4 degrees. It never replaces a visible angle.
    """

    def __init__(self, flap, guard, reading, *, max_bridged=12, agree=3., reacquire=4.):
        self.flap, self.guard, self.max_bridged = flap, guard, max_bridged
        self.agree, self.reacquire = agree, reacquire
        self.origin = np.asarray(reading['world_from_box'], dtype=float)
        self.last_seq = reading['seq']
        self.bridged, self.agreed, self.rows = 0, False, []

    def start(self, reading, kinematic):
        visual = reading['angles'][self.flap]['degrees']
        self.agreed = abs(visual - kinematic) <= self.agree
        self.rows.append(dict(seq=reading['seq'], visual_degrees=visual, kinematic_degrees=kinematic,
                              source='visual', agreed=self.agreed))

    def update(self, reading, command, kinematic, low, high):
        row = reading.get('angles', {}).get(self.flap)
        visual = None if row is None else row.get('degrees')
        if visual is not None and math.isfinite(visual):
            if self.bridged and abs(visual - kinematic) > self.reacquire:
                raise ValueError(f'Reacquired {self.flap} angle disagrees with contact kinematics')
            self.guard.check(reading, command)
            self.agreed, self.bridged, value, source = abs(visual - kinematic) <= self.agree, 0, visual, 'visual'
        else:
            if not self.agreed:
                raise ValueError(f'{self.flap} observation lost before contact kinematics agreed with vision')
            self.bridged += 1
            if self.bridged > self.max_bridged:
                raise ValueError(f'{self.flap} not visible for {self.max_bridged} contact commands')
            pose = np.asarray(reading.get('world_from_box'), dtype=float)
            seq = reading.get('seq')
            if (pose.shape != (4, 4) or not np.isfinite(pose).all() or not isinstance(seq, int)
                    or seq <= self.last_seq):
                raise ValueError('Fresh carton registration required to bridge a missing flap angle')
            cosine = (np.trace(self.origin[:3, :3].T @ pose[:3, :3]) - 1)/2
            if (np.linalg.norm(pose[:3, 3] - self.origin[:3, 3]) > .015
                    or math.degrees(math.acos(float(np.clip(cosine, -1, 1)))) > 8.):
                raise ValueError('Carton moved beyond the contact-fold bound while bridging')
            value, source = kinematic, 'contact_kinematics'
        self.last_seq = reading['seq']
        if not low <= value <= high:
            raise ValueError(f'{self.flap} angle {value:.1f} outside {low:g}--{high:g} degrees')
        self.rows.append(dict(seq=reading['seq'], command_degrees=command, visual_degrees=visual,
                              kinematic_degrees=kinematic, source=source, bridged=self.bridged))
        return value


def _sign(flap):
    return 1. if flap == 'long_near' else -1.


def _major_point(box, flap, theta, along, radius, outer):
    """Box-registered point on a major at ``radius`` and ``outer`` along its inward normal."""
    s, t = _sign(flap), math.radians(theta)
    inward = radius*math.sin(t) + outer*math.cos(t)
    up = radius*math.cos(t) - outer*math.sin(t)
    return box[:3, :3] @ np.array([along, -s*W/2 + s*inward, H + .0035 + up]) + box[:3, 3]


def _kinematic_angle(ik, sim, side, box, flap, outer):
    """Major angle implied by the jaw vertex on its panel at ``outer`` (inward +)."""
    point = np.linalg.inv(box) @ np.r_[ik.point(sim.data.qpos[sim.arm_indices[side][:5]]), 1.]
    inward = _sign(flap)*point[1] + W/2
    up = point[2] - H - .0035
    # (inward, up) is the contact rotated by theta from (outer, radius) along the panel.
    radius = math.sqrt(max(inward*inward + up*up - outer*outer, 1e-12))
    return math.degrees(math.atan2(inward, up) - math.atan2(outer, radius))


def _observed(reading, flap, low, high):
    row = reading.get('angles', {}).get(flap)
    degrees = None if row is None else row.get('degrees')
    if degrees is None or not math.isfinite(degrees) or not low <= degrees <= high:
        raise ValueError(f'Fresh {flap} angle within {low:g}--{high:g} degrees required, saw {degrees}')
    return float(degrees)


def close_majors_over_held_shorts(sim, controller, *, capture=False, far_pin_degrees=34.,
                                  near_target_degrees=88., far_target_degrees=88., max_commands=160):
    # Targets stop 2 degrees short of flat: a push that reaches 90 degrees
    # presses the shorts below flat into the empty carton (98.6 degrees seen).
    # 88 degrees is inside the original 85--95 degree visual closure band.
    if (not all(math.isfinite(v) for v in (far_pin_degrees, near_target_degrees, far_target_degrees))
            or not 25 <= far_pin_degrees <= 45 or not 30 <= near_target_degrees <= 90
            or not 30 <= far_target_degrees <= 90):
        raise ValueError('Far pin must be 25--45 and near/far targets 30--90 degrees')
    c = controller
    rng = np.random.default_rng(0)
    report = dict(simulation_only=True, hardware_commands=0, full_task_complete=False,
                  tape_applied=False, hands_clear=False, stage='register held shorts and free majors',
                  far_pin_degrees=far_pin_degrees, near_target_degrees=near_target_degrees,
                  far_target_degrees=far_target_degrees,
                  target_source='fresh RGB-D carton registration and flap angles plus declared CAD contact proposals',
                  checks=[], far_pin_commands=[], near_commands=[], far_commands=[])
    c.majors_over_shorts = report

    def gates(event, label):
        if event.get('step_error') or event['bad_penetration_mm'] > 1:
            raise ValueError(event.get('step_error') or f'Forbidden contact during {label}')
        if event.get('max_joint_tracking_error_radians', 0) > .08:
            raise ValueError(f'Joint tracking exceeds 0.08 rad during {label}')
        # Independent physics diagnostic: a stop condition, never a target.
        bad = {f: event['flap_angle_extrema_degrees'][f] for f in ('short_left', 'short_right')
               if not _SHORTS_BAND[0] <= event['flap_angle_extrema_degrees'][f][0]
               <= event['flap_angle_extrema_degrees'][f][1] <= _SHORTS_BAND[1]}
        if bad:
            raise ValueError(f'Short closure left {_SHORTS_BAND} during {label}: {bad}')

    def drive(side, q, point, ik, label, seconds=.25, allowed=(), pushed=None):
        planner = (_push_planner(sim, side, pushed, allowed) if pushed is not None
                   else JointPathPlanner(sim, side, clearance=.006, allowed_flaps=allowed))
        start = np.clip(sim.data.qpos[sim.arm_indices[side][:5]], planner.limits[:, 0], planner.limits[:, 1])
        if not planner.edge(start, q):
            raise ValueError(f'{label} path collides: {planner.last_collision}')
        event = sim.move({}, seconds, label, capture=capture, joint_targets={side: q})
        gates(event, label)
        if np.linalg.norm(ik.point(sim.data.qpos[sim.arm_indices[side][:5]]) - point) > .035:
            raise ValueError(f'{label}: CAD point tracking exceeds 35 mm')
        report['checks'].append(dict(time=float(sim.data.time), label=label,
            max_robot_flap_penetration_mm=event['max_robot_flap_penetration_mm'],
            independent_angles=sim.truth_angles(), motion=dict(sim.motion_stats)))

    def search(side, panel, targets):
        """Clear contact pose on a live-state copy, preferring margin beyond 6 mm."""
        copy = FarContactGeometry(sim.model, sim.data.qpos.copy(), side)
        for margin in (.010, .008, .006):
            copy.planner = JointPathPlanner(copy, side, clearance=margin, allowed_flaps=(panel,))
            for along, target in targets:
                for vertex in copy.vertices:
                    for _ in range(_IK_SEEDS):
                        found = copy.solve(target, vertex, rng.uniform(copy.limits[:, 0], copy.limits[:, 1]))
                        if found['clear']:
                            return dict(vertex=vertex, along=along, q=np.asarray(found['q']), margin_m=margin)
        return None

    def park(side, label):
        q, error = sim.ik(side, _PARK[side], None)
        if error > .008:
            raise ValueError(f'{side} park IK exceeds 8 mm')
        path = JointPathPlanner(sim, side, clearance=.006,
                                allowed_flaps=('short_left_cardboard', 'short_right_cardboard')).plan(q)
        execute_path(sim, side, path, label, capture=capture)
        gates(sim.events[-1], label)

    def lift_clear(side, direction, label):
        """Lift a hand in steps until it passes the 6 mm transit clearance."""
        tip = sim.data.site(side+'_tip').xpos.copy()
        transit = JointPathPlanner(sim, side, clearance=.006)
        for lift in (.015, .030, .045, .060):
            c.port.move_arms({side: tip + lift*direction}, .5, f'{label} {lift*1000:.0f} mm', None)
            gates(sim.events[-1], label)
            transit.data.qpos[:] = sim.data.qpos
            if transit.valid(sim.data.qpos[sim.arm_indices[side][:5]]):
                return lift
        raise ValueError(f'{label}: no pose with 6 mm transit clearance: {transit.last_collision}')

    def push_major(flap, target, commands, keep, radii=None, recontacts=_RECONTACTS):
        """Push a major's outer face (preferably 25 mm below its tip) to ``target`` degrees.

        A stalled stroke, or a next command whose every proposal is refused
        before execution, lifts off and searches a new contact from the current
        angle, excluding contacts already tried. Physical stops (penetration,
        forbidden or loaded non-jaw contact, carton motion, tracking) are never
        retried. The carton-drift origin stays the first registration.
        """
        side = _PUSH_ARM[flap]
        radii = radii or _PUSH_RADII[flap]
        origin = c.sense(f'Register {flap} before outer-face push')
        keep(origin)
        attempts = report.setdefault(flap+'_contact_attempts', [])
        tried, reading = set(), origin
        for attempt in range(recontacts + 1):
            try:
                return push_once(flap, target, commands, keep, radii, origin, reading, tried, side)
            except ValueError as exc:
                text = str(exc)
                if (attempt == recontacts or not any(k in text for k in _RECOVERABLE)
                        or any(k in text for k in _NEVER_RETRY)):
                    raise
                attempts.append(dict(attempt=attempt, reason=text, time=float(sim.data.time)))
                angle = math.radians(attempts_angle[0])
                outward = c.box[:3, :3] @ -np.array([0., _sign(flap)*math.cos(angle), -math.sin(angle)])
                lift_clear(side, outward + np.array([0., 0., .5]), f'Lift {side} claw off {flap} to recontact')
                reading = c.sense(f'Re-register {flap} before recontact')
                keep(reading)

    attempts_angle = [0.]

    def push_once(flap, target, commands, keep, radii, origin, reading, tried, side):
        theta0 = _observed(reading, flap, -25., 95.)
        attempts_angle[0] = theta0
        row = reading['angles'][flap]
        measured = dict(midplane_offset_mm=row.get('midplane_offset_mm') or 0.)
        report[flap+'_face_measurement'] = dict(measured, source=row.get('method'))
        outer = lambda clearance: measured['midplane_offset_mm']/1000 - CARDBOARD_THICKNESS_M/2 - clearance
        guard = ContactProgressGuard(flap, origin)
        choice = search(side, flap+'_cardboard',
                        [((along, radius), _major_point(c.box, flap, theta0, along, radius, outer(.002)))
                         for radius in radii for along in _PUSH_ALONG[flap] if (along, radius) not in tried])
        if choice is None:
            raise ValueError(f'No clear {side} outer-face contact on {flap}')
        choice['along'], radius = choice['along']
        tried.add((choice['along'], radius))
        report[flap+'_contact'] = dict({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in choice.items()},
                                       radius_m=radius)
        ik = RobotVertexIK(sim, side, choice['vertex'])
        contact = _major_point(c.box, flap, theta0, choice['along'], radius, outer(.002))
        t = math.radians(theta0)
        outward = -np.array([0., _sign(flap)*math.cos(t), -math.sin(t)])
        pre, q_pre = _clear_pre(sim, ik, side, contact, choice['q'], c.box,
                                [tuple(d*outward) for d in (.02, .015, .01, .025)])
        execute_path(sim, side, JointPathPlanner(sim, side, clearance=.006).plan(q_pre),
                     f'Reach outside {flap} with {side} claw', capture=capture)
        gates(sim.events[-1], f'{side} free transit to {flap}')
        previous = q_pre
        for u in np.linspace(.1, 1, 10):
            goal = (1-u)*pre + u*contact
            q, _ = ik.solve(goal, previous)
            drive(side, q, goal, ik, f'Approach {flap} outer face', .2, (flap+'_cardboard',))
            previous = q
        reading = c.sense(f'Observe {flap} at {side} contact')
        guard.check(reading, theta0)
        angle = _observed(reading, flap, -25., 95.)
        attempts_angle[0] = angle
        guard.begin_stroke(reading)
        bridge = _ContactAngleBridge(flap, guard, reading)
        bridge.start(reading, _kinematic_angle(ik, sim, side, c.box, flap, outer(.0005)))
        report.setdefault(flap+'_angle_sources', []).extend(bridge.rows)
        bridge.rows = report[flap+'_angle_sources']
        history = [angle]
        while angle < target - 1:
            if len(commands) >= max_commands:
                raise ValueError(f'{flap} outer-face push exceeded its command budget')
            theta = min(target, angle + 1.5)
            latest = reading['angles'].get(flap) or {}
            if latest.get('midplane_offset_mm') is not None:
                measured = dict(midplane_offset_mm=latest['midplane_offset_mm'])
            actual_q = sim.data.qpos[sim.arm_indices[side][:5]].copy()
            planner = _push_planner(sim, side, flap+'_cardboard')
            # Lowest reachable radius first; never move further toward the tip.
            for candidate in [r for r in radii if r <= radius]:
                goal = _major_point(c.box, flap, theta, choice['along'], candidate, outer(.0005))
                try:
                    q, _ = ik.solve(goal, actual_q)
                except ValueError:
                    continue
                if planner.edge(actual_q, q):
                    radius = candidate
                    break
            else:
                raise ValueError(f'Every {flap} outer-face push proposal collides: {planner.last_collision}')
            drive(side, q, goal, ik, f'Push {flap} outer face toward {theta:.1f} degrees at r={radius*1000:.0f} mm',
                  .25, pushed=flap+'_cardboard')
            reading = c.sense(f'Observe {flap} during outer-face push')
            box = np.asarray(reading['world_from_box'], dtype=float)
            angle = bridge.update(reading, theta, _kinematic_angle(ik, sim, side, box, flap, outer(.0005)),
                                  -25., target + 5)
            attempts_angle[0] = angle
            keep(reading)
            history.append(angle)
            commands.append(dict(commanded_degrees=theta, observed_degrees=angle, radius_m=radius,
                                 along_m=choice['along'], source=bridge.rows[-1]['source']))
            if len(history) > 12 and angle < target - 1 and angle - history[-13] < 2.:
                raise ValueError(f'{flap} outer-face push stalled: twelve commands produced less than 2 degrees')
        return reading

    # Register: shorts held by the right claw, far near upright, near outward.
    reading = c.sense('Register held shorts and free majors before closing majors')
    c.require_folded(reading, ['short_left', 'short_right'])
    far = _observed(reading, 'long_far', -3., 8.)
    _observed(reading, 'long_near', -25., -5.)
    far_guard = ContactProgressGuard('long_far', reading)
    report['initial_visual_angles'] = reading['angles']

    # A. Press-and-drag the far top edge to the pinning angle.
    report['stage'] = 'left press-and-drag of far top edge'
    row = reading['angles']['long_far']
    if row.get('free_edge_radius_mm') is None or row.get('midplane_offset_mm') is None:
        raise ValueError('Fresh measured far free edge required for the top-edge contact')
    edge = dict(free_edge_radius_mm=row['free_edge_radius_mm'], midplane_offset_mm=row['midplane_offset_mm'])
    outward = _EDGE_OUTWARD_START_M

    def edge_point(theta, along, below, measured=None):
        # ``outward`` biases the tip toward the outer side of the 3 mm edge: on
        # the inner corner a press pushes the flap outward, behind it the tip
        # hooks the outer face, which drags it inward as intended.
        measured = measured or edge
        return _major_point(c.box, 'long_far', theta, along, measured['free_edge_radius_mm']/1000 - below,
                            measured['midplane_offset_mm']/1000 - outward)

    c.port.set_grippers({'left': -.17}, .4, 'Close left claw before far top-edge contact')
    choice = search('left', 'long_far_cardboard',
                    [(along, edge_point(far, along, -.0015)) for along in _FAR_EDGE_ALONG])
    if choice is None:
        raise ValueError('No clear left top-edge contact on the measured far edge')
    report['far_edge_contact'] = dict(vertex=choice['vertex'], along_m=choice['along'], margin_m=choice['margin_m'],
                                      measured_edge=dict(edge))
    ik = RobotVertexIK(sim, 'left', choice['vertex'])
    above = edge_point(far, choice['along'], -.0015)
    pre, q_pre = _clear_pre(sim, ik, 'left', above, choice['q'], c.box,
                            [(0., dy, dz) for dz in (.020, .025, .030, .015) for dy in (0., -.010, .010, -.020)])
    execute_path(sim, 'left', JointPathPlanner(sim, 'left', clearance=.006).plan(q_pre),
                 'Reach above far top edge with left claw', capture=capture)
    gates(sim.events[-1], 'left free transit')
    previous = q_pre
    for u in np.linspace(.1, 1, 10):
        goal = (1-u)*pre + u*above
        q, _ = ik.solve(goal, previous)
        drive('left', q, goal, ik, 'Lower left claw above far top edge', .2, ('long_far_cardboard',))
        previous = q
    reading = c.sense('Observe far edge under left claw')
    far_guard.check(reading, far)
    far = _observed(reading, 'long_far', -3., 12.)
    far_guard.begin_stroke(reading)
    far_bridge = _ContactAngleBridge('long_far', far_guard, reading)
    far_bridge.start(reading, _kinematic_angle(ik, sim, 'left', c.box, 'long_far', edge['midplane_offset_mm']/1000))
    report['far_pin_angle_sources'] = far_bridge.rows
    history, press = [far], _PRESS_START_M
    while far < far_pin_degrees - 1:
        if len(report['far_pin_commands']) >= max_commands:
            raise ValueError('Far top-edge drag exceeded its command budget')
        # Seed from the actual encoder pose; sub-millimetre margins make the
        # IK branch matter. Try a shorter lead or lighter press before refusing.
        actual_q = sim.data.qpos[sim.arm_indices['left'][:5]].copy()
        planner = _push_planner(sim, 'left', 'long_far_cardboard')
        for lead, lighter in ((1., 0.), (.5, 0.), (1., .0005), (.5, .0005)):
            theta, depth = min(far_pin_degrees, far + lead), max(0., press - lighter)
            goal = edge_point(theta, choice['along'], depth)
            q, _ = ik.solve(goal, actual_q)
            if planner.edge(actual_q, q):
                break
        else:
            raise ValueError(f'Every far top-edge drag proposal collides: {planner.last_collision}')
        drive('left', q, goal, ik, f'Drag far top edge toward {theta:.1f} degrees (press {depth*1000:.1f} mm)',
              .25, pushed='long_far_cardboard')
        reading = c.sense('Observe far angle during top-edge drag')
        box = np.asarray(reading['world_from_box'], dtype=float)
        before = far
        far = far_bridge.update(reading, theta, _kinematic_angle(ik, sim, 'left', box, 'long_far',
                                                               edge['midplane_offset_mm']/1000),
                                -5., far_pin_degrees + 5)
        latest = reading['angles'].get('long_far') or {}
        if latest.get('free_edge_radius_mm') is not None and latest.get('midplane_offset_mm') is not None:
            edge = dict(free_edge_radius_mm=latest['free_edge_radius_mm'],
                        midplane_offset_mm=latest['midplane_offset_mm'])
        history.append(far)
        report['far_pin_commands'].append(dict(commanded_degrees=theta, observed_degrees=far, press_m=depth,
                                               outward_m=outward, source=far_bridge.rows[-1]['source'],
                                               edge_measurement=dict(edge)))
        if far < before - .2:
            # The flap moved outward: the tip is on the inner corner. Move the
            # contact outward and restart the press instead of pressing harder.
            if outward >= _EDGE_OUTWARD_MAX_M:
                if far >= _HOOK_MIN_DEGREES:
                    break
                raise ValueError('Far top-edge contact kept pushing the flap outward')
            outward, press = min(_EDGE_OUTWARD_MAX_M, outward + .001), _PRESS_START_M
        elif far - before < .3:
            if press >= _PRESS_MAX_M:
                if far >= _HOOK_MIN_DEGREES:
                    break
                raise ValueError('Far top-edge drag did not follow at the maximum declared press')
            press = min(_PRESS_MAX_M, press + .0005)
        if len(history) > 12 and far - history[-13] < 2.:
            if far >= _HOOK_MIN_DEGREES:
                break
            raise ValueError('Far top-edge drag stalled: twelve commands produced less than 2 degrees')
    report['far_drag_end_degrees'] = far
    if far < far_pin_degrees - 1:
        # The friction drag slipped after starting the flap inward: lift off
        # the edge, then hook behind it and pull to the pinning angle.
        report['stage'] = 'left hook behind far top edge to pinning angle'
        t = math.radians(far)
        radial = c.box[:3, :3] @ np.array([0., -math.sin(t), math.cos(t)])
        report['left_edge_lift_m'] = lift_clear('left', radial, 'Lift left claw off far top edge')
        push_major('long_far', far_pin_degrees, report['far_pin_commands'], lambda reading: None, radii=_HOOK_RADII)
    c.port.move_arms({}, 1., 'Hold far major over shorts', None)
    gates(sim.events[-1], 'far pin hold')
    reading = c.sense('Verify far pin before releasing short support')
    report['far_pin_visual_degrees'] = _verify_target_angle(reading, 'long_far', far_pin_degrees)
    far_guard = ContactProgressGuard('long_far', reading)

    # B. Right claw leaves the shorts; the far major must keep them closed.
    report['stage'] = 'right claw releases shorts under held far major'
    report['right_release_lift_m'] = lift_clear('right', np.array([0., 0., 1.]), 'Lift right claw off both shorts')
    park('right', 'Park right hand under held far major')
    for _ in range(4):
        c.port.move_arms({}, .5, 'Observe shorts held by far major only', None)
        gates(sim.events[-1], 'short retention under far major')
        reading = c.sense('Verify shorts and far major after right release')
        far_guard.check(reading, far_pin_degrees)
        _verify_target_angle(reading, 'long_far', far_pin_degrees)
    report['shorts_retained_by_far_major'] = True

    # C. Left jaw leaves the far major; its forearm would otherwise lie in the
    # near flap's sweep. The released far still bounds the shorts' corners.
    report['stage'] = 'left releases far major'
    t = math.radians(far_pin_degrees)
    radial = c.box[:3, :3] @ np.array([0., -math.sin(t), math.cos(t)])
    report['left_release_lift_m'] = lift_clear('left', radial, 'Lift left claw off far top edge')
    park('left', 'Park left hand after far pin')
    for _ in range(4):
        c.port.move_arms({}, .5, 'Observe released far major and shorts', None)
        gates(sim.events[-1], 'released far major')
    reading = c.sense('Verify shorts and released far major')
    c.require_folded(reading, ['short_left', 'short_right'])
    report['released_far_visual_degrees'] = _observed(reading, 'long_far', -5., far_pin_degrees + 5)

    # D. Right jaw closes the near major; the far major is free.
    report['stage'] = 'right outer-face push of near major'
    push_major('long_near', near_target_degrees, report['near_commands'], lambda reading: None)
    c.port.move_arms({}, .5, 'Hold near major closed', None)
    gates(sim.events[-1], 'near hold')
    reading = c.sense('Verify held near major')
    report['near_hold_visual_degrees'] = _verify_target_angle(reading, 'long_near', near_target_degrees)

    # E. Left jaw closes the far major while the right jaw holds the near.
    report['stage'] = 'left outer-face push of far major with near held'
    push_major('long_far', far_target_degrees, report['far_commands'],
               lambda reading: _verify_target_angle(reading, 'long_near', near_target_degrees))
    c.port.move_arms({}, 2., 'Hold both majors closed over shorts', None)
    event = sim.events[-1]
    gates(event, 'final two-major hold')
    reading = c.sense('Verify near and far closure')
    _verify_target_angle(reading, 'long_near', near_target_degrees)
    _verify_target_angle(reading, 'long_far', far_target_degrees)
    shorts = {f: event['flap_angle_extrema_degrees'][f] for f in ('short_left', 'short_right')}
    if any(not 85 <= low <= high <= 95 for low, high in shorts.values()):
        raise ValueError(f'Shorts not within 85--95 degrees during the final hold: {shorts}')
    report.update(stage='all four flaps closed and held by both jaws', visual_angles=reading['angles'],
                  independent_final_angles=sim.truth_angles(), final_hold_short_extrema_degrees=shorts,
                  motion=dict(sim.motion_stats), four_flaps_closed_and_held=True, both_hands_holding=True)
    return report
