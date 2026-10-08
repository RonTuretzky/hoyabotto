"""Experimental contact sequence for a diagonal carton and rear-mounted arms.

This is an offline, geometry-specific strategy, not a hardware motion adapter.
The demonstrated station has a 30-degree carton yaw, 150 mm base-to-table-edge
gap and base origins 60 mm above the table. Carton pose and flap observations
come through the same RGB-D port as the original controller.
"""
from __future__ import annotations

import math
import numpy as np

from carton.folding_controller import FoldingController, L, W, H


def contact_point(theta, axis, sign, along, radius, tilt, clearance):
    """A point on the outside of the rotating panel, in carton coordinates."""
    point = np.zeros(3)
    point[axis] = sign * ((L if axis == 0 else W) / 2
                          - radius * math.sin(theta) + clearance * math.cos(theta))
    point[1 - axis] = along
    point[2] = H + (.0035 if axis == 1 else 0) + radius * math.cos(theta) + clearance * math.sin(theta)
    direction = np.zeros(3)
    direction[axis] = -sign * tilt * (1 - theta / (math.pi / 2))
    direction[2] = 1
    direction /= np.linalg.norm(direction)
    return point, {'direction': direction.tolist()}


class DiagonalFoldingController(FoldingController):
    flap_assignments = {'left': ('short_left', 'long_far'),
                        'right': ('short_right', 'long_near')}

    def contact(self,side,theta,axis,sign,along,radius,tilt,clearance):
        return contact_point(theta,axis,sign,along,radius,tilt,clearance)

    def run(self):
        self.sense('Register diagonal carton from RGB-D')
        for i, theta in enumerate(np.linspace(0, math.pi / 2, 25)):
            points, orientations = {}, {}
            for side, sign, along, radius, tilt in (
                    ('left', -1, -.10, .095, 0.), ('right', 1, -.115, .14, 1.5)):
                if side == 'right':
                    # Begin at the edge, then slide onto the panel as it closes.
                    radius = .14 - .04 * float(np.clip((theta - math.pi / 4) / (math.pi / 4), 0, 1))
                points[side], orientations[side] = self.contact(side,theta, 0, sign, along, radius, tilt, .010)
            if i == 0:
                self.move({s: p + [0, 0, .020] for s, p in points.items()},
                          1., 'Approach short-flap edges', orientations)
            self.move(points, .22, 'Fold diagonal short flaps: ' + str(round(math.degrees(theta))), orientations)
            if i % 4 == 0:
                self.sense('Observe diagonal short flaps')

        self.port.set_grippers({'left': .35, 'right': .35}, .4, 'Release any fingertip pinch before lifting')
        self.move({s: p + [0, 0, .055] for s, p in points.items()}, .6, 'Lift clear of short flaps', orientations)
        # This retreat is explicitly in the calibrated world/base workspace.
        self.port.move_arms({'left': [-.24, -.10, .29], 'right': [.24, -.10, .29]},
                            1., 'Clear camera view of short flaps', 'down')
        self.port.set_grippers({'left': -.17, 'right': -.17}, .4, 'Close hands clear of carton')
        self.require_folded(self.sense('Verify diagonal short flaps'), ['short_left', 'short_right'])

        # Keep the near panel down while the opposite hand reaches the far corner.
        for flap, side, sign, along, tilt in (
                ('long_near', 'right', -1, 0., -.65), ('long_far', 'left', 1, -.175, 3.)):
            measured = self.sense('Locate ' + flap)['angles'].get(flap)
            if measured is None:
                raise ValueError('Visible flap required: ' + flap)
            start = math.radians(np.clip(measured['degrees'] - 5, -30, 0))
            for i, theta in enumerate(np.linspace(start, math.pi / 2, 25)):
                radius = (.14 - .025 * float(np.clip((theta - math.pi / 3) / (math.pi / 6), 0, 1))
                          if sign == 1 else .095)
                point, orientation = self.contact(side,theta, 1, sign, along, radius, tilt, .012)
                if i == 0:
                    if sign == -1:
                        # A vertical descent through a leaning panel pushes it
                        # outward. Clear its top, go outside, then descend.
                        normal = np.array([0, -math.cos(theta), math.sin(theta)])
                        outside = point + .035 * normal
                        self.move({side: point + [0, 0, .11]}, 1., 'Clear the raised near-flap edge', orientation)
                        self.move({side: outside + [0, 0, .12]}, .6, 'Go outside the near-flap edge', orientation)
                        self.move({side: outside}, .8, 'Lower outside the near-flap face', orientation)
                    else:
                        self.move({side: point + [0, 0, .020]}, .7, 'Approach edge of ' + flap, orientation)
                self.move({side: point}, .25, 'Fold ' + flap + ': ' + str(round(math.degrees(theta))), orientation)
                if i % 4 == 0:
                    self.sense('Observe ' + flap)
            self.require_folded(self.sense('Verify ' + flap), [flap])

        self.move({'left': [-.175, .0215, H + .0105], 'right': [0., -.0215, H + .0105]},
                  .8, 'Press and retain both long flaps')
        self.sense('Check paired contact hold')
        self.port.move_arms({}, 2., 'Verify two-second closure', None)
        self.require_folded(self.sense('Final visible long-flap closure'), ['long_far', 'long_near'])
        return {'visual_sequence_passed': True, 'visual_flap_evidence': self.visual_evidence, 'trace': self.trace}
