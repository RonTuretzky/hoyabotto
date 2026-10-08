"""Carton closing: a parked XLeRobot folds the four flaps of a filled carton and tapes the seam.

Separate task from the farm; shares its devices, skill runner, safety clamps, evidence store,
viewer and LLM teaching. Measurements and plan: docs/carton.md.
"""

from farm.mujoco_exit import install as _install_mujoco_exit

_install_mujoco_exit()
