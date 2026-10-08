"""Privileged near-angle diagnostic, never a vision or hardware success test.

Use only to separate contact-path failures from the independently investigated
RGB-D occlusion failure. The rest of the prefix uses the normal pixel port;
no object/robot state, physical parameter, control limit or collision gate is
changed. A successful mechanical probe cannot establish end-to-end readiness.
"""
from copy import deepcopy

from tools.simulate_bimanual_folding import PixelPort


class PrivilegedNearAngleProbe(PixelPort):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.probing_near = False

    def observe(self, label):
        if label == 'Register carton before transferring support to near major':
            self.probing_near = True
        reading = super().observe(label)
        if self.probing_near:
            reading['measured_near_before_diagnostic_override'] = deepcopy(
                reading['angles'].get('long_near'))
            reading['angles']['long_near'] = dict(
                degrees=self.sim.truth_angles()['long_near'],
                method='PRIVILEGED_SIMULATOR_ANGLE_DIAGNOSTIC_ONLY')
            reading['privileged_mechanics_probe'] = True
        return reading
