"""Explicit, uncalibrated station geometry for offline folding experiments.

Tabletop is z=0; the nominal carton is centred at x=y=0. Positive y is away
from the robot. Arm-base origins are the imported SO101 base_link origins,
not shoulder joints or the front edge of the cart.
"""
from dataclasses import asdict, dataclass
import math

from carton.geometry import Box


@dataclass(frozen=True)
class FoldingStation:
    base_height: float
    base_to_table_edge: float
    box_from_table_edge: float
    base_spacing: float = .30
    reference_layout: bool = False

    def __post_init__(self):
        values = (self.base_height, self.base_to_table_edge,
                  self.box_from_table_edge, self.base_spacing)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Station dimensions must be finite metres')
        if self.box_from_table_edge < 0 or self.base_spacing <= 0:
            raise ValueError('Carton must be on the table and base spacing positive')
        if self.base_to_table_edge < 0 and not self.reference_layout:
            raise ValueError('Base origin must be behind the table edge; negative gap is reserved for historical reference')

    @classmethod
    def historical_reference(cls):
        # Old table edge y=-.43; base y=-.1815, i.e. 248.5 mm OVER the table.
        return cls(.26, -.2485, .2885, reference_layout=True)

    @property
    def setback(self):
        return self.base_to_table_edge + self.box_from_table_edge

    @property
    def table_edge_y(self):
        return -Box().width / 2 - self.box_from_table_edge

    @property
    def base_y(self):
        return self.table_edge_y - self.base_to_table_edge

    @property
    def table_tag_position(self):
        # Reference reproduces old pixels. Other layouts put the assumed
        # surveyed anchor beside the carton, actually on the shifted tabletop.
        return [0., -.35, .0013] if self.reference_layout else [-.31, .08, .0013]

    def report(self):
        return {**asdict(self), 'base_setback_from_near_rim_m': self.setback,
                'table_edge_y_m': self.table_edge_y, 'base_y_m': self.base_y,
                'table_tag_position_m': self.table_tag_position,
                'physical_registration_verified': False,
                'scope': 'historical favorable layout, not photographed station' if self.reference_layout
                         else 'explicit hypothetical dimensions; not measured from photos'}
