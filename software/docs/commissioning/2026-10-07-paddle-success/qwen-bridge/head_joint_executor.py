"""Head moves (pan head_motor_1, tilt head_motor_2) for the pickup-profile owner started with --head.

The head carries the OAK camera (2026-10-09: OAK-D Lite in a slot cradle on the stock tilt link; the old USB head
camera is gone). It is driven by the same ramp, following-error (96 ticks), contact-halt, settle and watchdog guards
as the arm joints (this is PaddleJointExecutor with the head names), plus tighter head-only rules checked before
anything moves:

- only head motors, and only through the owner op 'head_move' (robot_move_head); never part of an arm's six-joint rule;
- at most 200 ticks per joint per move (about 18 degrees);
- duration_s at least 1 s per 100 ticks of the longest travel (the ramp then runs at most 40 ticks per 0.4 s);
- targets inside the saved calibration range minus the 40-tick margin (PaddleJointExecutor.MARGIN);
- every named joint travels at least 3 ticks (callers drop joints already within 2 ticks).
"""
import math
from paddle_joint_executor import PaddleJointExecutor,MARGIN
HEAD_MOTORS=('head_motor_1','head_motor_2')
HEAD_MAX_TICKS=200          # per joint per move
HEAD_TICKS_PER_S=100        # duration_s >= longest travel / this
HEAD_TORQUE_LIMIT=500       # enable torque limit and load fault level for head motors (arm joints: 800)
HEAD_LIMITS={'max_ticks_per_move':HEAD_MAX_TICKS,'min_duration_s_per_100_ticks':100/HEAD_TICKS_PER_S,'margin_ticks':MARGIN,
             'max_duration_s':25,'load_limit':HEAD_TORQUE_LIMIT,'tool':'robot_move_head'}


def check_head_request(c,current):
    """Head-only rules on top of the executor's own checks; raises ValueError before anything moves."""
    if c.get('waypoints') is not None:raise ValueError('Head moves take positions, not waypoints')
    positions=c.get('positions');duration=c.get('duration_s')
    if not isinstance(positions,dict) or not positions or any(n not in HEAD_MOTORS for n in positions):
        raise ValueError('Head move targets head_motor_1 (pan) and/or head_motor_2 (tilt) only')
    if any(type(t) is not int for t in positions.values()):raise ValueError('Integer head-motor target required')
    travel={n:abs(t-current[n]) for n,t in positions.items()}
    for n,d in travel.items():
        if d>HEAD_MAX_TICKS:raise ValueError(f'Head move: {n} travels {d} ticks; at most {HEAD_MAX_TICKS} per move')
    if type(duration) not in (int,float) or not math.isfinite(duration) or not 0<duration<=25:raise ValueError('Head move duration_s must be finite in (0,25]')
    need=max(travel.values())/HEAD_TICKS_PER_S
    if duration+1e-9<need:raise ValueError(f'Head move duration_s must be at least 1 s per 100 ticks: {max(travel.values())} ticks needs {need:.2f} s, got {duration}')
    return travel


class HeadJointExecutor(PaddleJointExecutor):
    PREFIXES=('head_motor_',)
    def start(self,c,current,session_started,held_goals=None):
        check_head_request(c,current)
        return super().start(c,current,session_started,held_goals)
