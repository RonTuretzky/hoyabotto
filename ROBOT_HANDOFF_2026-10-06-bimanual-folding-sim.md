# Two-hand carton folding: simulation handoff

**Correction after reviewing actual photos:** this scene is not registered to
the real station. The old tabletop extended 248.5 mm behind the simulated
arm-base line, placing bases over the tabletop. The photo shows the robot cart
behind the table, and the latest saved view has a paddle but no folding carton.
Read `software/docs/carton-folding-station-audit.md` first. Larger separations
reject the current far-flap trajectory; physical folding remains unvalidated.
Do not use the five historical passes to declare the actual station reachable.

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
above the table, 4 cm base setback **from the carton rim**, 30 cm base spacing, filled-box contents
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

Station dimensions are now explicit. To replay historical success, both tools
require `--reference-layout`. A new station requires arm-base height, base-line
to table-edge distance and carton near-wall inset from that edge. The base
origin is the imported `base_link`, not the shoulder joint or cart front edge.
`software/tools/audit_folding_reach.py` checks ten hypothetical layouts using
the same controller contact points; its sparse IK checks are not collision,
perception, or contact validation. No physical robot commands were sent.
