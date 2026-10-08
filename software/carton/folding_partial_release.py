"""Opt-in joint-driven release of the observed near40/far35 partial holds.

This offline component tests passive retention with both arms parked. It does
not close the carton or supply a hardware obstacle model. Object targets and
hold checks use fresh rendered observations; robot withdrawal uses encoder FK.
"""
import math

import numpy as np

from carton.folding_far_contact import _verify_target_angle
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_progress import ContactProgressGuard


def _require_passive_far_stage(controller):
    prior = getattr(controller, 'far_edge_attempt', None)
    expected = dict(
        stage='partial far angle passively retained for five seconds',
        passive_retention_only=True, right_hand_parked=True,
        near_still_actively_held=True, physical_far_target_verified=True,
        target_far_degrees=35., expected_near_degrees=40.,
        full_task_complete=False,
    )
    if not isinstance(prior, dict) or any(prior.get(k) != v for k, v in expected.items()):
        raise ValueError('Verified passive far35 release with near40 still held required')
    checks = prior.get('visual_progress_checks')
    if (not isinstance(checks, list) or not checks
            or any(row.get('stop_reason') for row in checks)
            or not isinstance(checks[-1].get('seq'), int)
            or isinstance(checks[-1]['seq'], bool) or checks[-1]['seq'] < 1):
        raise ValueError('Successful fresh visual checks from the prior far release required')
    return checks[-1]['seq']


class _PartialMajorHold:
    """Keep both declared bands, freshness, and original carton drift limits."""
    targets = {'long_near': 40., 'long_far': 35.}

    def __init__(self, reading, *, after_seq):
        self.fault = None
        self.guards = {flap: ContactProgressGuard(flap, reading) for flap in self.targets}
        if reading['seq'] <= after_seq:
            raise ValueError('A new visual observation after the far release is required')
        for flap, target in self.targets.items():
            _verify_target_angle(reading, flap, target)

    def check(self, reading):
        if self.fault:
            raise ValueError('Partial hold fault remains latched: ' + self.fault)
        try:
            for flap, target in self.targets.items():
                self.guards[flap].check(reading, target)
                _verify_target_angle(reading, flap, target)
        except (ValueError, TypeError) as exc:
            self.fault = str(exc)
            raise


def release_near_after_passive_far(sim, controller, *, capture=False):
    """Release left contact, park, and verify five seconds of both partial holds.

    Requires the successful far35 passive-release stage with near40 actively
    held. This is intentionally one-shot: an incomplete or failed attempt must
    not be re-entered to clear its visual/drift faults. No gripper, flap, carton,
    joint-state, or physical-parameter resets are performed.
    """
    c = controller
    if getattr(c, 'partial_major_release', None) is not None:
        raise ValueError('Partial major release already attempted; its outcome cannot be reset')
    prior_seq = _require_passive_far_stage(c)
    report = dict(
        simulation_only=True, hardware_commands=0, joint_driven_physics=True,
        stage='register partial holds before left release', fault=None,
        expected_near_degrees=40., expected_far_degrees=35.,
        passive_retention_only=True, both_hands_parked=False,
        both_majors_passively_retained=False, all_flaps_closed=False,
        full_task_complete=False, lift_m=.020, hold_seconds=5., checks=[],
        target_source='fresh RGB-D partial angles and robot encoder FK withdrawal',
    )
    c.partial_major_release = report

    def require_parked(side):
        target = np.array([-.20 if side == 'left' else .20, -.18, .30])
        actual = np.asarray(sim.actual_control_position(side), dtype=float)
        if (actual.shape != (3,) or not np.isfinite(actual).all()
                or np.linalg.norm(actual - target) > .035):
            raise ValueError(f'{side.capitalize()} parked-hand tracking exceeds 35 mm')

    try:
        reading = c.sense('Verify fresh near40 and passive far35 before left release')
        hold = _PartialMajorHold(reading, after_seq=prior_seq)
        report['visual_checks'] = {flap: guard.checks for flap, guard in hold.guards.items()}
        report['initial_visual_angles'] = reading['angles']
        require_parked('right')

        def observe(label, *, left_parked=False):
            current = c.sense(label)
            hold.check(current)
            require_parked('right')
            if left_parked:
                require_parked('left')
            report['checks'].append(dict(time=float(sim.data.time), seq=current['seq'],
                                         visual_angles=current['angles']))
            return current

        actual = sim.actual_control_position('left').copy()
        direction = sim.data.body('left_gripper_link').xmat.reshape(3, 3)[:, 2].copy()
        goal = actual + np.array([0., 0., .020])
        q, error = sim.ik('left', goal, {'direction': direction.tolist()})
        if not math.isfinite(error) or error > .008:
            raise ValueError('Partial near release IK exceeds 8 mm')
        report['withdrawal'] = dict(start_world=actual.tolist(), target_world=goal.tolist(),
                                    direction_world=direction.tolist(), ik_error_m=error)
        report['stage'] = 'physically lift left claw clear of partial near hold'
        planner = JointPathPlanner(sim, 'left', clearance=.006,
                                   allowed_flaps=('long_near_cardboard',))
        execute_path(sim, 'left', planner.plan(q),
                     'Physically lift left claw from partial near flap', capture=capture)
        if np.linalg.norm(sim.actual_control_position('left') - goal) > .035:
            raise ValueError('Partial near release Cartesian tracking exceeds 35 mm')
        observe('Verify both partial major angles after physical left release')

        report['stage'] = 'physically park left hand after partial near release'
        q, error = sim.ik('left', np.array([-.20, -.18, .30]), None)
        if not math.isfinite(error) or error > .008:
            raise ValueError('Partial near released-hand park IK exceeds 8 mm')
        path = JointPathPlanner(sim, 'left', clearance=.006).plan(q)
        execute_path(sim, 'left', path, 'Park left hand after partial near release', capture=capture)
        observe('Verify partial angles with both hands newly parked', left_parked=True)
        report['both_hands_parked'] = True
        report['stage'] = 'observe both partial majors without hand support'
        for _ in range(10):
            c.port.move_arms({}, .5, 'Observe original passive retention of both partial majors', None)
            reading = observe('Verify fresh near40 and far35 with both hands parked', left_parked=True)
        report.update(
            stage='both partial major angles passively retained for five seconds',
            both_majors_passively_retained=True, visual_angles=reading['angles'],
            # Independent evaluation only; never used to generate commands or
            # substitute a missing rendered observation.
            independent_final_angles=sim.truth_angles(), motion=dict(sim.motion_stats),
        )
        return report
    except Exception as exc:
        report['fault'] = str(exc)
        raise
