"""Fresh visual stop conditions for an offline contact-fold attempt.

These bounds stop a failed push; they neither constrain the free carton nor
constitute a successful fold. Observations must come from the perception port.
"""
from dataclasses import dataclass, field
import math

import numpy as np


def _observation(reading, flap):
    if not isinstance(reading, dict):
        raise ValueError('Fresh visual observation required')
    pose = np.asarray(reading.get('world_from_box'), dtype=float)
    row = reading.get('angles', {}).get(flap)
    seq = reading.get('seq')
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-5)
            or not isinstance(seq, int) or isinstance(seq, bool) or seq < 1
            or not isinstance(row, dict)
            or not math.isfinite(row.get('degrees', float('nan')))):
        raise ValueError('Fresh finite carton pose and flap angle required')
    return pose.copy(), float(row['degrees']), seq


@dataclass
class ContactProgressGuard:
    flap: str
    initial_reading: dict
    max_translation_m: float = .015
    max_rotation_degrees: float = 8.
    command_window_degrees: float = 12.
    min_progress_degrees: float = 2.
    checks: list = field(default_factory=list, init=False)

    def __post_init__(self):
        values = (self.max_translation_m, self.max_rotation_degrees,
                  self.command_window_degrees, self.min_progress_degrees)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Positive finite contact-progress bounds required')
        self.origin, angle, self.last_seq = _observation(self.initial_reading, self.flap)
        self.history = [(angle, angle)]

    def check(self, reading, command_degrees):
        if not math.isfinite(command_degrees):
            raise ValueError('Finite commanded fold angle required')
        pose, angle, seq = _observation(reading, self.flap)
        if seq <= self.last_seq:
            raise ValueError('Contact progress requires a new visual observation')
        self.last_seq = seq
        translation = float(np.linalg.norm(pose[:3, 3] - self.origin[:3, 3]))
        cosine = (np.trace(self.origin[:3, :3].T @ pose[:3, :3]) - 1) / 2
        rotation = math.degrees(math.acos(float(np.clip(cosine, -1, 1))))
        reason = None
        if translation > self.max_translation_m:
            reason = f'Carton moved more than {self.max_translation_m*1000:g} mm during contact fold'
        elif rotation > self.max_rotation_degrees:
            reason = 'Carton rotated beyond the contact-fold bound'
        earlier = [entry for entry in self.history
                   if command_degrees - entry[0] >= self.command_window_degrees]
        progress = None
        if earlier:
            progress = angle - earlier[-1][1]
            if reason is None and progress < self.min_progress_degrees:
                reason = (f'Fold stalled: commanded advance produced less than '
                          f'{self.min_progress_degrees:g} degrees of progress')
        self.history.append((float(command_degrees), angle))
        row = dict(seq=seq, command_degrees=float(command_degrees), flap_degrees=angle,
                   translation_mm=translation * 1000, rotation_degrees=rotation,
                   observed_progress_degrees=progress, stop_reason=reason)
        self.checks.append(row)
        if reason:
            raise ValueError(reason)
        return row
