"""Passive, segmented tape for offline contact experiments.

This is an uncalibrated material model, not a tape-placement controller.
The backing is approximately inextensible, with finite bending/torsional
stiffness. Adhesion uses MuJoCo's native finite-tension contact law, not a
weld, a flap-angle constraint, or an actuator. The strip is never pinned.

The ordinary 2 ms folding timestep is unsuitable for this thin strip.
The dated coupon audit compares discrete/Newton with exact constraint-inertia
diagonals at 25 and 10 microseconds. That component agreement is configuration
specific; it does not certify tape placement, carton retention or material
strength. See docs/carton-tape-convergence-2026-10-07.md.
"""
from dataclasses import asdict, dataclass
import math
import re
import xml.etree.ElementTree as E


@dataclass(frozen=True)
class TapeSpec:
    length_m: float = .080
    width_m: float = .024
    backing_thickness_m: float = .000100
    adhesive_thickness_m: float = .000020
    density_kg_m3: float = 750.
    backing_young_pa: float = 200e6
    poisson: float = .3
    segments: int = 16
    angular_damping_Nm_s: float = 2e-7
    adhesion_per_contact_N: float = .020
    adhesion_range_m: float = .000020
    surface_friction: float = .5

    def __post_init__(self):
        positive=('length_m','width_m','backing_thickness_m',
                  'adhesive_thickness_m','density_kg_m3','backing_young_pa')
        for name in positive:
            value=getattr(self,name)
            if not math.isfinite(value) or value<=0:raise ValueError('Positive finite '+name+' required')
        for name in ('angular_damping_Nm_s','adhesion_per_contact_N','adhesion_range_m','surface_friction'):
            value=getattr(self,name)
            if not math.isfinite(value) or value<0:raise ValueError('Nonnegative finite '+name+' required')
        if not math.isfinite(self.poisson) or not 0<=self.poisson<.5:
            raise ValueError('Poisson ratio must be in [0, 0.5)')
        if isinstance(self.segments,bool) or not isinstance(self.segments,int) or not 2<=self.segments<=64:
            raise ValueError('Use between 2 and 64 tape segments')

    @property
    def thickness_m(self):return self.backing_thickness_m+self.adhesive_thickness_m

    @property
    def mass_kg(self):return self.length_m*self.width_m*self.thickness_m*self.density_kg_m3

    @property
    def segment_length_m(self):return self.length_m/self.segments

    @property
    def bend_stiffness_Nm(self):
        return self.backing_young_pa*self.width_m*self.backing_thickness_m**3/(12*self.segment_length_m)

    @property
    def twist_stiffness_Nm(self):
        shear=self.backing_young_pa/(2*(1+self.poisson))
        return shear*self.width_m*self.backing_thickness_m**3/(3*self.segment_length_m)

    def report(self):
        return {**asdict(self),'parameters_measured':False,'mass_kg':self.mass_kg,
                'bend_stiffness_Nm_per_rad':self.bend_stiffness_Nm,
                'twist_stiffness_Nm_per_rad':self.twist_stiffness_Nm,
                'adhesion_model':'Native passive finite-tension contact, per contact point; not measured masking-tape peel strength.',
                'validated_for_folding':False,'peel_timing_converged':False,
                'peel_timing_scope':'No integrator/timestep is specified by TapeSpec. Use the dated component convergence audit; generic material parameters are not a retention certificate.',
                'approximations':'Inextensible rectangular segments with bending and twist; no in-plane bending or adhesive aging. Finite adhesive range and thin-layer one-sided contact require validation before robot use.',
                'pinned':False,'actuated':False}


def add_tape(root, spec, *, start_m, name='tape', flipped=False):
    """Add a free strip with its adhesive layer initially facing downward.

    start_m is the midpoint of the short edge at the start of the strip.
    Native passive adhesion requires a MuJoCo build supporting geom.adhesion.
    The caller selects and verifies an integrator for the thin stiff backing.
    """
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',name):raise ValueError('Invalid tape name')
    if len(start_m)!=3 or not all(math.isfinite(v) for v in start_m):
        raise ValueError('Finite three-dimensional tape start required')
    world=root.find('worldbody')
    if world is None:raise ValueError('Scene requires a worldbody')
    if root.find(f".//body[@name='{name}_0']") is not None:raise ValueError('Tape name already exists')
    dl=spec.segment_length_m;parent=world
    for index in range(spec.segments):
        attrs={'name':f'{name}_{index}','pos':' '.join(map(str,start_m if index==0 else (dl,0,0)))}
        if index==0 and flipped:attrs['quat']='0 1 0 0'
        body=E.SubElement(parent,'body',**attrs)
        if index==0:E.SubElement(body,'freejoint',name=name+'_free')
        else:
            for axis,vector,stiffness in (('twist','1 0 0',spec.twist_stiffness_Nm),
                                          ('bend','0 1 0',spec.bend_stiffness_Nm)):
                E.SubElement(body,'joint',name=f'{name}_{axis}_{index}',type='hinge',
                             axis=vector,limited='false',stiffness=str(stiffness),
                             damping=str(spec.angular_damping_Nm_s))
        # The backing has zero adhesion. Soft contact can penetrate a thin
        # backing; callers must reject excessive penetration and covered-face
        # tensile contacts, as the coupon diagnostic does. Priority preserves
        # these thin-layer contact settings when touching softer cardboard.
        for face,thickness,z,adhesion,colour in (
            ('backing',spec.backing_thickness_m,spec.adhesive_thickness_m/2,0.,'.92 .83 .50 1'),
            ('adhesive',spec.adhesive_thickness_m,-spec.backing_thickness_m/2,
             spec.adhesion_per_contact_N,'.72 .65 .35 1')):
            E.SubElement(body,'geom',name=f'{name}_{face}_{index}',type='box',
                         pos=f'{dl/2} 0 {z}',size=f'{dl/2} {spec.width_m/2} {thickness/2}',
                         mass=str(spec.mass_kg/spec.segments*thickness/spec.thickness_m),
                         adhesion=str(adhesion),friction=f'{spec.surface_friction} .0001 .00001',
                         gap=str(spec.adhesion_range_m if face=='adhesive' and adhesion else 0),
                         priority='1',condim='3',solref='.0004 1',
                         solimp='.9999 .9999 .00002',rgba=colour)
        parent=body
    return name+'_0'
