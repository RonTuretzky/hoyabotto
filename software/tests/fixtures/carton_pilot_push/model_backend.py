"""Frozen pre-change decision schema; no live imports or startup.
Historical wording is intentional.
"""

def _nullable(kind, **extra):
    return {'type': [kind, 'null'], **extra}

DECISION_ACTIONS = ['look', 'reach', 'reach_delta', 'fold', 'move', 'sense', 'halt', 'finish']

DECISION_SCHEMA = {
    'type': 'object', 'additionalProperties': False,
    'required': ['action', 'question', 'arm', 'forward_cm', 'left_cm', 'up_cm', 'along_jaws_cm', 'pitch_deg', 'toward', 'radius_cm', 'degrees', 'steps', 'hold_s', 'pitch_end_deg', 'path', 'hinge_cm', 'to_deg', 'calls', 'what', 'reason', 'answer'],
    'properties': {
        'action': {'type': 'string', 'enum': DECISION_ACTIONS,
                   'description': 'look: ask the eyes one question about the cameras; reach: put a claw tip at a point in the robot frame; '
                                  'reach_delta: move a claw tip by an offset from where it is; fold: turn a pinched flap about its hinge on an arc toward the box centre; move: 1-4 robot tool calls run in order; '
                                  'sense: read state/scene/motion; halt: stop and hold; finish: final message to the user.'},
        'question': _nullable('string', description='look only: under 400 characters, naming the cameras and one concrete question.'),
        'arm': {'type': ['string', 'null'], 'enum': ['left', 'right', None], 'description': 'reach, reach_delta and fold only.'},
        'forward_cm': _nullable('number', description='reach: target forward of the base; reach_delta: change in cm.'),
        'left_cm': _nullable('number', description='reach: target left of the base (negative = right); reach_delta: change in cm.'),
        'up_cm': _nullable('number', description='reach: target height above the FLOOR; reach_delta: change in cm.'),
        'along_jaws_cm': _nullable('number', description='reach_delta only: cm along the direction the jaws point (negative = back).'),
        'pitch_deg': _nullable('number', description='reach/reach_delta: 0 = claw level, negative = tip down; null keeps the default.'),
        'toward': {'type': ['string', 'null'], 'enum': ['forward', 'back', 'left', 'right', None], 'description': 'fold only: direction of the box centre seen from the flap.'},
        'radius_cm': _nullable('number', description='fold only: pinch height minus the hinge (box top) height, 4 to 15 cm.'),
        'degrees': _nullable('number', description='fold only: how far to turn the flap, 15 to 120 (null = 90).'),
        'steps': _nullable('integer', description='fold only: number of reach steps, 1 to 6 (null = 3).'),
        'hold_s': _nullable('number', description='fold only: seconds to hold the flap at the end of the arc with the gripper closed, 0 to 30 (null = 0).'),
        'pitch_end_deg': _nullable('number', description='fold only: pitch at the last step; the pitch eases linearly to it from the current one (null keeps the pitch).'),
        'path': {'type': ['boolean', 'null'], 'description': 'fold only: true sends the arc as one continuous path (null = false: separate reaches).'},
        'hinge_cm': {'type': ['array', 'null'], 'items': {'type': 'number'},
                     'description': 'fold only: [position along the toward axis (left_cm for toward left/right, forward_cm for forward/back), up_cm] of the hinge line in model cm; the arc then runs about it from where the claw is to to_deg (radius_cm unused).'},
        'to_deg': _nullable('number', description='fold with hinge_cm only: the absolute end angle from vertical, 15 to 130 (null = 110).'),
        'calls': {'type': ['array', 'null'], 'description': 'move only: 1-4 tool calls run in order, stopping at the first refusal.',
                  'items': {'type': 'object', 'additionalProperties': False, 'required': ['tool', 'args'],
                            'properties': {'tool': {'type': 'string', 'description': 'a move tool name from the list'},
                                           'args': {'type': 'object', 'description': "that tool's arguments"}}}},
        'what': {'type': ['array', 'null'], 'items': {'type': 'string', 'enum': ['state', 'motion', 'scene', 'heights', 'tags']}, 'description': 'sense only (tags: fresh carton tag IDs and per-image pixels; heights: a height map from the head depth camera with a verdict per box side).'},
        'reason': _nullable('string', description='halt only: why the arm must stop and hold.'),
        'answer': _nullable('string', description='finish only: the final message to the user.')}}
