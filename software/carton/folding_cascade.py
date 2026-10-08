"""Offline attempt to transfer one short-flap hold to a major flap.

The right hand keeps its existing short-flap contact while the left releases
the rim pinch and presses the near major. Any incidental panel-to-panel fold
comes from contact physics. No flap is prescribed or latched. This is a
partial experiment, not a completed folding policy or a hardware adapter.
"""
import math

import numpy as np

from carton.folding_diagonal import contact_point
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_progress import ContactProgressGuard


def press_near_over_short(sim, controller, *, capture=False, from_minor=False,
                          along=-.13, radius=.105, tilt=-.65,
                          pre_out=.055, pre_up=.040, release_right=False,
                          right_park_joints=None, from_open_claw=False,
                          release_right_at_degrees=None, majors_first=False,
                          target_degrees=90.):
    if (not all(math.isfinite(v) for v in (along, radius, tilt, pre_out, pre_up))
            or not -.18 <= along <= .18 or not .02 <= radius <= .14
            or not -3 <= tilt <= 3 or not 0 <= pre_out <= .10
            or not 0 <= pre_up <= .10):
        raise ValueError('Invalid near-major contact geometry')
    if release_right and not from_minor:
        raise ValueError('Right-hand withdrawal is only defined after the minor folds')
    if release_right_at_degrees is not None and (not from_open_claw or
            not math.isfinite(release_right_at_degrees) or
            not -10 <= release_right_at_degrees <= 60 or release_right):
        raise ValueError('A visual withdrawal angle from -10 to 60 degrees requires an open-claw hold')
    if majors_first and (from_minor or from_open_claw or release_right or release_right_at_degrees is not None):
        raise ValueError('Major-first mode starts from parked hands without a short-flap hold')
    if (not math.isfinite(target_degrees) or not 40 <= target_degrees <= 90
            or (target_degrees != 90 and not majors_first)):
        raise ValueError('Partial near hold requires major-first mode and target 40..90 degrees')
    c = controller
    port = c.port
    c.sense('Register carton before transferring support to near major')
    if from_minor and not from_open_claw:
        actual=sim.data.site('left_tip').xpos.copy()
        port.move_arms({'left':actual+[0,0,.065]},.5,
                       'Lift left hand clear of reopening short flap','down')
    if release_right:
        if not from_minor:
            raise ValueError('Right-hand withdrawal is only defined after the minor folds')
        if right_park_joints is not None:
            # Encoder pose recorded at the validated initial park; execute a
            # checked joint path, never restore the robot or free-tool state.
            q=np.asarray(right_park_joints,dtype=float)
            path=JointPathPlanner(sim,'right',clearance=.006,
                allowed_flaps=('short_right_cardboard',)).plan(q)
        else:
            actual=sim.actual_control_position('right').copy()
            port.move_arms({'right':actual+[0,0,.145]},1.,
                           'Lift right hand clear before near-major sweep','down')
            q,error=sim.ik('right',np.array([.20,-.18,.30]),None)
            if error>.008:raise ValueError('Right park IK exceeds 8 mm')
            path=JointPathPlanner(sim,'right',clearance=.006).plan(q)
        execute_path(sim,'right',path,'Park right hand outside near-major sweep',capture=capture)
    if majors_first:
        port.set_grippers({'left': -.17}, .5, 'Close left claw at park before major-first approach')
    if not from_minor and not from_open_claw and not majors_first:
        port.set_grippers({'left': .6}, .5, 'Release left near-flap pinch')
        actual = sim.data.site('left_tip').xpos.copy()
        direction = sim.data.body('left_gripper_link').xmat.reshape(3, 3)[:, 2].copy()
        port.move_arms(
            {'left': actual + c.box[:3, :3] @ np.array([0, -.04, .025])}, .8,
            'Withdraw left fingers before near-flap press',
            {'direction': direction.tolist()},
        )
        port.set_grippers({'left': -.17}, .5, 'Close left claw clear of the panel')
    reading = c.sense('Locate near major with short flaps open' if majors_first else
                      'Locate near major while right hand retains right short')
    measured = reading['angles'].get('long_near')
    if measured is None:
        raise ValueError('Fresh near-flap observation required before press')
    start = math.radians(measured['degrees'])
    if not -math.pi / 2 < start < math.pi / 2:
        raise ValueError('Near flap outside the proposed press approach')
    checks = []
    guard = ContactProgressGuard('long_near', reading)
    # Keep diagnostics available even if the attempt raises before returning.
    if not hasattr(c,'contact_progress_checks'):c.contact_progress_checks=[]
    c.contact_progress_checks.append({'stage':'near major transfer','checks':guard.checks})
    right_released = release_right or majors_first
    def maybe_withdraw_right(observed):
        nonlocal right_released
        if (release_right_at_degrees is None or right_released or
                observed['angles']['long_near']['degrees'] < release_right_at_degrees):
            return observed, False
        tip = sim.actual_control_position('right').copy()
        direction = sim.data.body('right_gripper_link').xmat.reshape(3, 3)[:, 2].copy()
        port.move_arms({'right': tip + [0, 0, .14]}, 1.,
                       'Withdraw open right claw at declared observed near-flap angle',
                       {'direction': direction.tolist()})
        q, error = sim.ik('right', np.array([.20, -.18, .30]), None)
        if error > .008:
            raise ValueError('Right park IK after support handoff exceeds 8 mm')
        path = JointPathPlanner(sim, 'right', clearance=.006,
                                allowed_flaps=('long_near_cardboard',)).plan(q)
        execute_path(sim, 'right', path, 'Park right claw during near-flap hold', capture=capture)
        right_released = True
        observed = c.sense('Observe short-flap state after right withdrawal; retention is unproven')
        guard.check(observed, observed['angles']['long_near']['degrees'])
        return observed, True

    point, orientation = contact_point(start, 1, -1, along, radius, tilt, .010)
    outward = np.array([0, -math.cos(start), math.sin(start)])
    outside = point + pre_out * outward + [0, 0, pre_up]
    world = c.box[:3, :3] @ outside + c.box[:3, 3]
    world_orientation = {'direction': (c.box[:3, :3] @ orientation['direction']).tolist()}
    q, error = sim.ik('left', world, world_orientation)
    if error > .008:
        raise ValueError(f'Near-major press approach IK misses {error * 1000:.2f} mm')
    planner = JointPathPlanner(sim, 'left', clearance=.006,
        allowed_flaps=('short_left_cardboard','long_near_cardboard') if from_minor or from_open_claw else ())
    path = planner.plan(q)
    execute_path(sim, 'left', path, 'Reach outside near major', capture=capture)
    observed = c.sense('Check carton after near-major transit')
    guard.check(observed, math.degrees(start))
    observed, withdrew = maybe_withdraw_right(observed)
    if not withdrew:
        for u in np.linspace(.1, 1, 10):
            c.move({'left': (1-u)*outside + u*point}, .15,
                   ('Approach near-major face with short flaps open' if majors_first else
                    'Approach near-major face with right short held'), orientation)
            observed = c.sense('Check carton during near-major contact approach')
            guard.check(observed, math.degrees(start))
            observed, withdrew = maybe_withdraw_right(observed)
            if withdrew:
                break
    # The contact approach itself rotates the flap. Regenerate the sweep from
    # that fresh observation rather than reversing towards a stale start angle.
    start = math.radians(observed['angles']['long_near']['degrees'])
    if not -math.pi / 2 < start < math.pi / 2:
        raise ValueError('Near flap outside the proposed press sweep')
    guard.begin_stroke(observed)
    for theta in np.linspace(start, math.radians(target_degrees), 51):
        point, orientation = contact_point(theta, 1, -1, along, radius, tilt, .010)
        c.move({'left': point}, .3,
               (f'Press near major first {math.degrees(theta):.1f}' if majors_first else
                f'Press near major over right short {math.degrees(theta):.1f}'), orientation)
        # Independent simulation guard, not a claim of hardware sensing.
        angles = sim.truth_angles()
        check = {'time': float(sim.data.time), 'angles': angles,
                 'motion': dict(sim.motion_stats)}
        checks.append(check)
        if not right_released and angles['short_right'] < 80:
            raise ValueError('Right short-flap support lost during near-major press')
        observed = c.sense('Observe panel contact and carton movement during near-major press')
        guard.check(observed, math.degrees(theta))
        maybe_withdraw_right(observed)
    port.move_arms({}, 2., 'Hold near major' if majors_first else 'Hold near major and right short', None)
    reading = c.sense('Verify near major remains held' if majors_first else
                      'Verify near major and right short remain held')
    if target_degrees == 90:
        c.require_folded(reading, ['long_near'] if majors_first else
                         (['long_near', 'short_left', 'short_right'] if right_released
                          else ['long_near', 'short_right']))
    else:
        observed = reading['angles'].get('long_near')
        if observed is None or abs(observed['degrees']-target_degrees) > 5:
            raise ValueError('Near clearance angle was not held within five degrees')
    return {'angles': sim.truth_angles(), 'motion': dict(sim.motion_stats),
            'visual_angles': reading['angles'], 'checks': checks,
            'contact_progress_checks': guard.checks,
            'right_released_after_visual_handoff': right_released and from_open_claw,
            'held_only': True, 'target_degrees': target_degrees,
            'near_major_closure_verified': target_degrees == 90, 'full_task_complete': False}
