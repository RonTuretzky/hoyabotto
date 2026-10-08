"""Offline first checks for bare-gripper, two-arm carton closing.

Reuses the measured carton and shoulder/reach approximation. No camera, policy
inference, motor transport or dynamics are instantiated. Arc points are a
feasibility screen, not inverse kinematics or executable trajectories.
"""
from __future__ import annotations

from dataclasses import asdict
import math

import numpy as np

from .geometry import Box, Stance

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
JOINTS = tuple(f'{arm}_arm_{joint}' for arm in ('left', 'right') for joint in ARM_JOINTS)
UNITS = tuple(unit for _ in ('left', 'right') for unit in ('model_degrees',)*5+('open_fraction',))
PHASES = (
    ('short_left', 'left', 'right braces the box without obstructing the left flap'),
    ('short_right', 'right', 'left holds the first short flap down'),
    ('long_far', 'right', 'left retains the short flaps while the far flap covers them'),
    ('long_near', 'left', 'right holds the far flap down while the near flap closes'),
)


def fold_reach_screen(box: Box, stance: Stance, samples: int = 31):
    """Screen vertical-to-horizontal flap contacts at half flap length.

    The saved shoulder pose, fingertip length, upright starting flaps and contact
    locations are assumptions. Bracing contacts and arm/arm swept volumes still
    need a full dual-arm contact simulation. Neither arm carries a paddle.
    """
    values = list(asdict(box).values()) + list(asdict(stance).values())
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Geometry must be finite')
    if min(box.length, box.width, box.height, box.flap, stance.spacing, stance.tool) <= 0:
        raise ValueError('Positive box dimensions, arm spacing and fingertip length required')
    if stance.setback < 0 or stance.margin < 0 or stance.reach(False) <= 0 or samples < 3:
        raise ValueError('Invalid stance, reach margin or sample count')
    rows = []
    for name, arm, supporting_role in PHASES:
        shoulder = stance.shoulder(arm, box)
        contact_x = min(stance.spacing/2, box.length/2-.01)
        points = []
        for angle in np.linspace(0, math.pi/2, samples):
            inward, z = box.flap/2*math.sin(angle), box.flap/2*math.cos(angle)
            if name == 'short_left':p = (-box.length/2+inward, 0., z)
            elif name == 'short_right':p = (box.length/2-inward, 0., z)
            elif name == 'long_far':p = (contact_x, box.width/2-inward, z)
            else:p = (-contact_x, -box.width/2+inward, z)
            points.append(p)
        distances = [math.dist(shoulder, p) for p in points]
        worst = int(np.argmax(distances))
        rows.append({'phase':name, 'folding_arm':arm, 'supporting_role':supporting_role,
                     'contact_arc_box_frame_m':points, 'worst_contact_m':points[worst],
                     'maximum_shoulder_distance_m':distances[worst],
                     'reach_margin_m':stance.reach(False)-distances[worst],
                     'within_spherical_reach':max(distances)<=stance.reach(False)})
    return {'status':'OFFLINE_REACH_SCREEN', 'scope':'close_existing_carton_top_flaps',
            'box':asdict(box), 'stance':asdict(stance), 'uses_paddle':False,
            'usable_bare_gripper_reach_m':stance.reach(False), 'phases':rows,
            'all_folding_contacts_within_spherical_reach':all(r['within_spherical_reach'] for r in rows),
            'motor_writes':0, 'collision_checked':False, 'contact_simulated':False,
            'learned_policy_evaluated':False, 'motion_ready':False,
            'success_definition':'All four flaps closed under coordinated hold; release retention is a separate test. No tape in this experiment.',
            'remaining':['Validate both arm-base transforms, fingertip offsets and camera views',
                         'Check bracing contacts and both arms together with URDF IK and collision geometry',
                         'Model flap springback/contact and test held-out box poses',
                         'Evaluate a genuinely 12-action learned policy; geometry alone is not policy success']}


def validate_action_chunk(actions, *, joint_order, units, max_steps=100):
    """Reject single-arm/YAM outputs instead of padding, truncating or mirroring.

    This is a data boundary for a future adapted model, never a live command.
    Model joint zeros/signs still need explicit encoder conversion elsewhere.
    """
    if tuple(joint_order) != JOINTS or tuple(units) != UNITS:
        raise ValueError('Need explicit left-six/right-six order, model degrees and gripper open fractions')
    values = np.asarray(actions, dtype=float)
    if values.ndim != 2 or values.shape[1] != 12 or not 1 <= len(values) <= max_steps or not np.isfinite(values).all():
        raise ValueError('Need a finite (time, 12) joint-action chunk; never reshape 6-D or 14-D actions')
    if np.any(values[:,[5,11]] < 0) or np.any(values[:,[5,11]] > 1):
        raise ValueError('Both gripper channels must explicitly use 0 closed to 1 open')
    return {'status':'BIMANUAL_DATA_SHAPE_VALID', 'joint_order':list(JOINTS), 'units':list(UNITS),
            'actions':values.tolist(), 'motor_writes':0, 'motion_ready':False,
            'note':'Dimensions and units checked only; no policy transfer, joint-limit, collision or contact validation.'}


def molmo_checkpoint_contract(stats, norm_tag):
    """Inspect the actual released normalizer, not max_action_dim padding size."""
    meta = stats['metadata_by_tag'][norm_tag]
    n_state, n_action = len(meta['state_stats']['q01']), len(meta['action_stats']['q01'])
    return {'norm_tag':norm_tag, 'state_dim':n_state, 'action_dim':n_action,
            'camera_keys':meta.get('camera_keys', []), 'control_mode':meta.get('control_mode'),
            'dimension_matches_dual_so101':n_state==n_action==12,
            'direct_deployment_verified':False,
            'reason':'12 channels still require verified order, units, cameras, normalization and task adaptation'
                     if n_state==n_action==12 else 'Robot embodiment differs; do not duplicate, truncate or mirror output channels'}
