# Two-hand carton folding: simulation handoff

The bare-gripper geometric controller now closes the measured box in MuJoCo
using rendered AprilTags and aligned depth. This is a simulation baseline for
the MolmoAct2 work, not a physical execution prompt or a learned-policy result.

Read `software/docs/carton-bimanual-rgbd-simulation.md` and its evidence JSON.
Five operating cases passed, six failure controls failed as intended, and the
strong-springback case remains unsupported. All four flaps must be within 5°
of horizontal continuously for two seconds; the two grippers must contact
their assigned flaps and the folded shorts must have content support.

Code is under `software/carton/folding_{sim,vision,controller}.py`.
`software/tools/simulate_bimanual_folding.py` runs one case;
`software/tools/evaluate_bimanual_folding.py` runs the reproducible matrix.
Dependencies are pinned in `software/requirements-carton-folding-sim.txt`.
Use fresh output directories. The tools have no physical motor/camera backend.

On the development Mac the verified replay is:
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/verified-matrix-v2/nominal/folding.gif`
and the full results are in that matrix directory. The source arm/collision
assets are at `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot`.
These absolute paths are local; do not assume they exist on the robot Mac.

Material and station assumptions are part of the result: 26 cm arm-base height
above the table, 4 cm base setback, 30 cm base spacing, filled-box contents
support, measured carton dimensions, and the specified passive hinge model.
Extra tags are ID 4 on the left housing, ID 10 on the box wall and IDs 11–14 on
flap outside faces. Existing table ID 1 and right housing ID 2 are reused.
The paddle and its ID 3 are unused.

For Molmo, reuse this scene, sensing interface and independent evaluator to
collect coordinated examples and compare a genuine twelve-channel policy.
Do not label this as Molmo inference, drop two channels from a YAM policy,
remove the second arm from evaluation or copy these waypoints to live motors.
Physical use still requires the actual station geometry, contents/crease
measurements, both arm registrations, fingertip offsets and live control gates.
