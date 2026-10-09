# Arm spacing from the XLeRobot 0.4.0 kit CAD

The shoulder-pan axes are **273.000 mm apart with both arm plates in the STEP export's orientation**.
This comes from the actual 0.4.0 arm-base parts and matching SO101 mounting holes, not a tape measurement,
simulator spacing, or a dimension inferred from a camera image. The adjustable plate orientation matters. On 10 October the owner independently checked the assembled robot manually and reported that 273 mm seems right; this corroborates the CAD value approximately, without establishing a metrology tolerance.

## Sources and reproducibility

- [XLeRobot 0.4.0 armbase STEP, pinned b017b5e](https://github.com/Vector-Wangel/XLeRobot/blob/b017b5e6354bd9f61f4247a920c72622ca0aade0/hardware/step/XLeRobot_040/XLeRobot040_armbase.step)
- [SO101 base STEP, pinned 5f6d2b8](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Base_SO101.step)
- [0.4.0 assembly guide at the same XLeRobot revision](https://github.com/Vector-Wangel/XLeRobot/blob/b017b5e6354bd9f61f4247a920c72622ca0aade0/docs/en/source/hardware/getting_started/assemble_2wheel.md), which describes changeable arm installation directions.

Download those two STEP files, install CadQuery 2.8 in an isolated environment, then run:

```sh
python software/tools/measure_xlerobot040_arm_spacing.py \
  --armbase /path/to/XLeRobot040_armbase.step \
  --so101-base /path/to/Base_SO101.step \
  --out /path/to/arm-spacing.json
```

The [checked extraction](evidence/xlerobot040-arm-spacing.json) records file hashes, cylinder centres,
mounting holes, residuals and the result. The script reads CAD only; it never connects to the robot.

## Derivation

The arm-base STEP contains 14 solids. Top plates 10 and 12 (zero-based) have their rotation centres at
CAD XY `(21.3267, 152.3761)` and `(21.3267, -120.6239)` mm: 273 mm apart. Lower-seat geometry differs
by 0.1 mm in one component; the upper plates that accept the arms give the stated 273.0 mm.

Each upper plate has four SO101 mating holes: two at X=52.8223 and Y=centreY±31.75, and two at
X=-16.9527 and Y=centreY±27.7765 mm. The SO101 base has the matching pairs at local X=±31.75,
Z=-7.5 and local X=±27.7765, Z=62.275 mm. A rigid fit with local Y upward gives:

```
CAD X = 45.3223 - SO101 local Z
CAD Y = plate centre Y - SO101 local X
```

The base's pan-bearing cylinder has local X=0, Z=45.2 mm. Thus each pan axis lies at CAD X≈0.1223,
Y=its plate centre Y. The axes share X and differ in Y by 273 mm. All four holes agree with the rigid
fit to below 0.001 mm; this is CAD consistency, not a claim of physical assembly tolerance.

## Why this is not a universal installed spacing

Each pan axis is offset about 21.2044 mm from the corresponding rotary-plate centre. Turning a plate
therefore shifts its pan axis as well as its arm's forward direction. In the export, the XY offset is
`(-21.2044, 0)` mm. At an additional plate angle θ it is
`(-21.2044*cos θ, -21.2044*sin θ)`. Use each plate's actual orientation before computing installed axes.
The mounting ring offers discrete positions; the CAD result alone does not establish which positions
the owner assembled.

Previously used values have different origins: 220 mm was a training-model assumption, 310.4 mm was
in the upstream twin/reach model, and about 286 mm was camera-inferred. None overrides this kit CAD
measurement. Following the owner's approximate physical confirmation, the repository's pilot model now uses
273 mm through a shared assembly definition in `farm/kinematics/xlerobot_geometry.py`.
The twin applies an explicit 18.7 mm inward translation to each arm subtree of the default
vendored MJCF at load time; the upstream asset stays unchanged. Custom MJCF files keep their
own geometry. The reach solver and carton right-base-to-midpoint translation use the same
half-spacing (136.5 mm), so their frames stay aligned. The camera-to-right-arm registration
itself remains unchanged because it is in the arm-local frame.

The matching modules were installed in the local pilot on 10 October during an idle,
all-motors-released window. A chat-only restart and source-hash/import checks confirmed
activation. The hardware owner was not restarted. See `software/STATUS.md` for the record.

This is a spacing correction, not validation of arm orientation, encoder zero/sign, height,
absolute camera registration or contact accuracy. Old midpoint-frame coordinates must be
re-sensed after activation. Historical benchmark scores retain their original geometry.
