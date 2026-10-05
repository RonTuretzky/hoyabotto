"""Bounded position-mode capture of the observed high-side elbow sag."""
import math


def recovery_target(position, lower, upper):
    if not all(type(v) is int for v in (position,lower,upper)):
        raise ValueError('Integer encoder values required')
    if not 0<=lower<upper<=4095 or not 0<=position<=4095:
        raise ValueError('Invalid or wrapping encoder range')
    if position<=upper:
        raise ValueError('Recovery only supports a high-side out-of-range start')
    # Keep the same total travel cap as the first successful recovery. If the
    # released arm has settled slightly farther, use a closer interior goal,
    # never less than 24 ticks inside the saved boundary.
    target=max(upper-48,position-math.floor(12*4096/360))
    if target<lower or target>upper-24 or position-target>math.floor(12*4096/360):
        raise ValueError('Recovery exceeds twelve-degree envelope')
    return target
