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


def press_near_over_short(sim, controller, *, capture=False):
    c = controller
    port = c.port
    c.sense('Register carton before transferring near-flap pinch to press')
    port.set_grippers({'left': .6}, .5, 'Release left near-flap pinch')
    actual = sim.data.site('left_tip').xpos.copy()
    direction = sim.data.body('left_gripper_link').xmat.reshape(3, 3)[:, 2].copy()
    port.move_arms(
        {'left': actual + c.box[:3, :3] @ np.array([0, -.04, .025])}, .8,
        'Withdraw left fingers before near-flap press',
        {'direction': direction.tolist()},
    )
    port.set_grippers({'left': -.17}, .5, 'Close left claw clear of the panel')
    reading = c.sense('Locate near major while right hand retains right short')
    measured = reading['angles'].get('long_near')
    if measured is None:
        raise ValueError('Fresh near-flap observation required before press')
    start = math.radians(measured['degrees'])
    if not -math.pi / 2 < start < math.pi / 2:
        raise ValueError('Near flap outside the proposed press approach')
    checks = []
    for index, theta in enumerate(np.linspace(start, math.pi / 2, 51)):
        point, orientation = contact_point(theta, 1, -1, -.13, .105, -.65, .010)
        if index == 0:
            outward = np.array([0, -math.cos(theta), math.sin(theta)])
            outside = point + .055 * outward + [0, 0, .040]
            world = c.box[:3, :3] @ outside + c.box[:3, 3]
            world_orientation = {'direction': (c.box[:3, :3] @ orientation['direction']).tolist()}
            q, error = sim.ik('left', world, world_orientation)
            if error > .008:
                raise ValueError(f'Near-major press approach IK misses {error * 1000:.2f} mm')
            planner = JointPathPlanner(sim, 'left', clearance=.006)
            path = planner.plan(q)
            execute_path(sim, 'left', path, 'Reach outside near major', capture=capture)
            for u in np.linspace(.1, 1, 10):
                c.move({'left': (1-u)*outside + u*point}, .15,
                       'Approach near-major face with right short held', orientation)
        c.move({'left': point}, .3,
               f'Press near major over right short {math.degrees(theta):.1f}', orientation)
        # Independent simulation guard, not a claim of hardware sensing.
        angles = sim.truth_angles()
        check = {'time': float(sim.data.time), 'angles': angles,
                 'motion': dict(sim.motion_stats)}
        checks.append(check)
        if angles['short_right'] < 80:
            raise ValueError('Right short-flap support lost during near-major press')
        if index % 5 == 0:
            c.sense('Observe panel contact and carton movement during near-major press')
    port.move_arms({}, 2., 'Hold near major and right short', None)
    reading = c.sense('Verify near major and right short remain held')
    c.require_folded(reading, ['long_near', 'short_right'])
    return {'angles': sim.truth_angles(), 'motion': dict(sim.motion_stats),
            'visual_angles': reading['angles'], 'checks': checks,
            'held_only': True, 'full_task_complete': False}
