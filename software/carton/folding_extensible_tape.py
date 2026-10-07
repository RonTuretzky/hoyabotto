"""Isolated finite-axial-compliance tape experiment; never a folding policy.

The legacy 80 mm coupon remains unchanged. Each original rigid tape segment
retains its geometry/material. Passive axial slides approximate distributed
backing extension; their series compliance equals L/(E*A). Adhesive axial
stiffness, plasticity, material calibration, and robot application are absent.
"""
from dataclasses import dataclass, field
import math
import xml.etree.ElementTree as ET

from carton.folding_tape import TapeSpec, add_tape


@dataclass(frozen=True)
class ExtensibleTapeSpec:
    tape: TapeSpec = field(default_factory=lambda: TapeSpec(length_m=.180, segments=36))
    # No new physical loss is fitted to make a benchmark pass. Discrete
    # integration has numerical damping, whose timestep sensitivity is audited.
    axial_damping_Ns_m: float = 0.

    def __post_init__(self):
        if not isinstance(self.tape, TapeSpec):
            raise ValueError('An explicit original TapeSpec is required')
        if (type(self.axial_damping_Ns_m) not in (int, float)
                or not math.isfinite(self.axial_damping_Ns_m)
                or self.axial_damping_Ns_m < 0):
            raise ValueError('Finite nonnegative axial damping required')

    @property
    def axial_rigidity_N(self):
        return self.tape.backing_young_pa * self.tape.width_m * self.tape.backing_thickness_m

    @property
    def axial_cell_length_m(self):
        # Lump the end half-cells into the n-1 elastic interfaces. Using the
        # geometric 5 mm spacing as every spring's elastic gauge would omit
        # one cell and make total compliance 35/36 of the declared 180 mm strip.
        return self.tape.length_m / (self.tape.segments-1)

    @property
    def axial_joint_stiffness_N_m(self):
        return self.axial_rigidity_N / self.axial_cell_length_m

    def extension_m(self, tension_N):
        if type(tension_N) not in (int, float) or not math.isfinite(tension_N):
            raise ValueError('Finite axial test load required')
        return tension_N * self.tape.length_m / self.axial_rigidity_N

    def report(self):
        return dict(tape=self.tape.report(), axial_rigidity_N=self.axial_rigidity_N,
                    axial_cell_length_m=self.axial_cell_length_m,
                    axial_joint_stiffness_N_m=self.axial_joint_stiffness_N_m,
                    axial_damping_Ns_m=self.axial_damping_Ns_m,
                    material_measured=False, validated_for_folding=False,
                    approximation='Rigid 5 mm segments separated by passive elastic axial interfaces; lumped end-half-cell compliance; no continuous geometry stretching, adhesive axial stiffness, plasticity or calibrated strength.')


def add_extensible_tape(root, spec, *, start_m, name='tape', flipped=False):
    """Add the original free strip plus passive axial springs, without bonding."""
    if not isinstance(spec, ExtensibleTapeSpec):
        raise ValueError('ExtensibleTapeSpec required')
    body = add_tape(root, spec.tape, start_m=start_m, name=name, flipped=flipped)
    for index in range(1, spec.tape.segments):
        segment = root.find(f".//body[@name='{name}_{index}']")
        segment.insert(0, ET.Element('joint', name=f'{name}_axial_{index}', type='slide',
                                    axis='1 0 0', limited='false',
                                    stiffness=str(spec.axial_joint_stiffness_N_m),
                                    damping=str(spec.axial_damping_Ns_m)))
    return body
