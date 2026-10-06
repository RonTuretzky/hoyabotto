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
    table_marker_xy: tuple[float, float] | None = None
    backup_table_marker_xy: tuple[float, float] | None = None

    def __post_init__(self):
        values = (self.base_height, self.base_to_table_edge,
                  self.box_from_table_edge, self.base_spacing)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Station dimensions must be finite metres')
        if self.box_from_table_edge < 0 or self.base_spacing <= 0:
            raise ValueError('Carton must be on the table and base spacing positive')
        if self.base_to_table_edge < 0 and not self.reference_layout:
            raise ValueError('Base origin must be behind the table edge; negative gap is reserved for historical reference')
        for marker in (self.table_marker_xy,self.backup_table_marker_xy):
            if marker is None:continue
            if len(marker)!=2 or not all(math.isfinite(v) for v in marker):
                raise ValueError('Table marker requires two finite coordinates')
            x,y=marker
            # 60 mm black square plus the full 7.5 mm white border.
            if abs(x)>.55-.0375 or not self.table_edge_y+.0375<=y<=self.table_edge_y+1.1-.0375:
                raise ValueError('The complete table marker must fit on the tabletop')

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
        if self.table_marker_xy is not None:return [*self.table_marker_xy,.0013]
        return [0., -.35, .0013] if self.reference_layout else [-.45, .65, .0013]

    def report(self):
        return {**asdict(self), 'base_setback_from_near_rim_m': self.setback,
                'table_edge_y_m': self.table_edge_y, 'base_y_m': self.base_y,
                'table_tag_position_m': self.table_tag_position,
                'physical_registration_verified': False,
                'scope': 'historical favorable layout, not photographed station' if self.reference_layout
                         else 'explicit hypothetical dimensions; not measured from photos'}

    def carton_footprint(self, offset=(0., 0.), yaw=0.):
        """Check all initial bottom corners, including a rotated box's inset."""
        if len(offset)!=2 or not all(math.isfinite(v) for v in (*offset,yaw)):
            raise ValueError('Finite carton translation and yaw required')
        b=Box();c,s=math.cos(yaw),math.sin(yaw)
        corners=[(offset[0]+c*x-s*y,offset[1]+s*x+c*y)
                 for x in (-b.length/2,b.length/2) for y in (-b.width/2,b.width/2)]
        margins=[min(x+.55,.55-x,y-self.table_edge_y,self.table_edge_y+1.1-y) for x,y in corners]
        return {'corners_xy_m':corners,'minimum_table_edge_clearance_m':min(margins),
                'fully_on_table':min(margins)>=-1e-9}
