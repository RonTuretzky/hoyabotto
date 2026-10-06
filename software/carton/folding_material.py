"""Explicit, unmeasured material assumptions for free-carton sensitivity tests."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class CartonMaterial:
    cardboard_mass_kg: float = .272
    contents_mass_kg: float = 0.
    contents_top_m: float = .102
    table_friction: float = .35
    hinge_stiffness: float = .018
    hinge_friction: float = .004
    hinge_damping: float = .008
    hinge_rest_degrees: float = 0.
    flap_stiffness: tuple[float, float, float, float] | None = None

    def __post_init__(self):
        nonnegative=(self.contents_mass_kg,self.table_friction,self.hinge_stiffness,
                     self.hinge_friction,self.hinge_damping)
        if not all(math.isfinite(v) and v>=0 for v in nonnegative):
            raise ValueError('Mass, friction, stiffness and damping must be finite and nonnegative')
        if not math.isfinite(self.cardboard_mass_kg) or self.cardboard_mass_kg<=0:
            raise ValueError('Cardboard mass must be positive and finite')
        if not math.isfinite(self.contents_top_m) or not .004<self.contents_top_m<.108:
            raise ValueError('Contents support must lie between bottom and carton rim')
        if not math.isfinite(self.hinge_rest_degrees) or not -60<=self.hinge_rest_degrees<=60:
            raise ValueError('Declared crease rest angle must lie between -60 and 60 degrees from upright')
        if self.flap_stiffness is not None and (len(self.flap_stiffness)!=4 or
                not all(math.isfinite(v) and v>=0 for v in self.flap_stiffness)):
            raise ValueError('Supply four finite nonnegative crease stiffnesses')

    @property
    def stiffnesses(self):
        return self.flap_stiffness or (self.hinge_stiffness,)*4

    def report(self):
        return {**asdict(self),'parameters_measured':False,'carton_constraint':'six-DOF free joint, no weld, clamp or guide',
                'total_mass_kg':self.cardboard_mass_kg+self.contents_mass_kg,
                'spring_moments_at_90_degrees_Nm':[
                    k*math.radians(90-self.hinge_rest_degrees) for k in self.stiffnesses],
                'model':'Rigid panels; elastic opening torque k*(angle-rest), Coulomb crease friction and viscous damping. No plastic crease training or panel bending.',
                'contents_model':'Absent' if self.contents_mass_kg==0 else 'Rigid contents volume attached to box; load shifting is not modeled'}
