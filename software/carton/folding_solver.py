"""Declared numerical settings; these do not change friction or motor limits.

The friction preset follows MuJoCo's slow-slip guidance. It is a numerical
sensitivity experiment, not evidence that the physical gripper holds a tool.
https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FoldingSolver:
    timestep: float = .002
    impratio: float = 1.
    tolerance: float = 1e-8
    noslip_iterations: int = 0

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.timestep, self.impratio, self.tolerance)):
            raise ValueError('Positive finite solver parameters required')
        if (type(self.noslip_iterations) is not int
                or not 0 <= self.noslip_iterations <= 3):
            raise ValueError('NoSlip diagnostic iteration count must be 0 through 3')

    @classmethod
    def friction(cls, timestep=.002):
        return cls(timestep=timestep, impratio=10., tolerance=1e-10,
                   noslip_iterations=3)

    def xml_attributes(self):
        return dict(timestep=str(self.timestep), integrator='implicitfast',
                    cone='elliptic', solver='Newton', iterations='80',
                    impratio=str(self.impratio), tolerance=str(self.tolerance),
                    noslip_iterations=str(self.noslip_iterations))


def solver_report(model):
    """Read back the actual engine settings, including diagnostic overrides."""
    import mujoco
    o = model.opt
    return dict(timestep=float(o.timestep), impratio=float(o.impratio),
                tolerance=float(o.tolerance), iterations=int(o.iterations),
                noslip_iterations=int(o.noslip_iterations),
                noslip_tolerance=float(o.noslip_tolerance),
                solver=mujoco.mjtSolver(o.solver).name,
                cone=mujoco.mjtCone(o.cone).name,
                integrator=mujoco.mjtIntegrator(o.integrator).name)
