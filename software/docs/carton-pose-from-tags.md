# Carton pose from the AprilTags (right-arm registration)

`farm/perception/carton_pose.py` turns one fresh OAK `robot_get_tags` observation plus the installed
camera-to-arm registration (`pilot/.private/tag-registration.json`, validated 2026-10-10, residuals
0.8-1.5 mm on the recorded held-out samples) into the carton's position in the frame the pilot's `reach` action takes. Read-only: it
calls `robot_get_state` (head ticks) and `robot_get_tags` (OAK, carton IDs) and nothing else.
Pilot tool: `robot_get_carton_pose`; chat: `{"action":"sense","what":["carton"]}`.

Activated in the local pilot on 10 October with the matching 273 mm geometry, push policy
and eyes/supervisor guidance. The tool catalog and HTTP route were verified after an idle
chat-only restart. The first route check refused stale/future-dated/missing head capture
timing; it did not yield usable carton coordinates. This is a tested refusal, not a new
successful live geometry measurement. See `software/STATUS.md` for deployment evidence.

## Frames

| frame | definition | units |
|---|---|---|
| camera | `CAM_A_optical`: +x right, +y down, +z forward | mm (tag solver) |
| arm base | SO-101 URDF `base_link` of the RIGHT arm: +x forward, +y the arm's left, +z up; `base_from_camera` from the registration | mm |
| model | pilot reach frame (`farm/sim/xlerobot_twin.py` FRAME): origin on the floor below the midpoint of the two shoulder-pan axes; +forward, +left, +up | cm |

model = base + (-38.8, -136.5, +777.4) mm. The URDF puts the pan axis at base x 0.0388 and the
shoulder-lift axis 0.1166 m above base z = 0; the twin puts the right pan axis at forward 0 / left
-0.1365 and the lift axis 0.894 m above the floor (`farm/kinematics/so101_reach.py`). Rotation is the
identity: base +x is taken as the robot's forward, i.e. the pan midpoint tick points the arm straight
ahead, the same `feetech_degrees_v1` candidate the reach solver and the registration use. Because
the registration and the twin share the same URDF base frame, a target taken from this tool and
sent to `reach` is consistent with the twin's FK even if the modelled shoulder height differs from
the real floor height; the floor-referenced numbers are only as good as that 0.894 m. Shared coordinates do not establish physical accuracy: mount orientation, joint mapping and camera fit still matter. The 273 mm kit CAD spacing is orientation-dependent; after the owner approximately corroborated it on the assembled robot, the twin, reach solver and this translation were updated together through `farm/kinematics/xlerobot_geometry.py`. The arm-local camera registration is unchanged.

## Assumptions (stated in every result)

- Carton tag sizes are the print plan's: 45 mm black squares for the wall/floor tags
  (10, 26, 27, 21, 28, 22, 24, 25) and 35 mm for the flap tags (11, 12, 13, 14). The owner printed
  the sheet at 100 %; nothing was ruler-measured. Live check: tag spacings 26-27 238 mm (plan 240),
  10-27 120 mm (plan 120), consistent with the nominal print scale. This is a consistency check, not independent scale calibration.
- Wall tags sit at half wall height (54 mm below the rim); 26/27 are 120 mm either side of 10;
  flap tags 90 mm above the hinge and 70 mm toward the far side along it.
- The near wall (10/26/27) is vertical: the three tag centres are collinear, so the plane is the
  vertical plane through the fitted horizontal line of the centres (tag normals tilt 6 deg from
  horizontal live; the resulting rim shift would be 5 mm, ignored).
- Carton 379 x 283 x 108 mm, 140 mm flaps, cardboard thickness ignored. This carton is placed with
  the print plan's "left" panels on the robot's RIGHT (`mirrored=True`): 21/28 and flap 11 are on
  the robot's right, 22 and flap 12 on its left. Roles are reported both ways.
- The registration is valid only with the head at `head_ticks_reference` (2085/2623, 3 ticks). The
  tool refuses otherwise. It requires fresh stationary head telemetry bracketing the OAK capture, at most 1 tick of change within that bracket, finite capture age at most 1 second, unchanged registration file, and matching camera-intrinsics and tag-geometry hashes. Old OAK captures get at most four read-only attempts.
- This is coarse carton geometry. It does not revalidate current motor calibration, kinematic model, commandable ranges or the gripper-consistency check that `robot_get_registered_tags` runs. A registration file marked validated is historical fit evidence, not full current robot validation. Do not use this output as an executable contact target; revalidate registration and physical clearance before motion.

## What the tool returns

Each detected carton tag centre in both frames; the fitted carton: near face (forward at the
centre, yaw from square), rim height, right wall top line = hinge of the right short flap (near
corner, far corner, midpoint, direction), left wall top line, far face; a scale check over the
near-wall tag pairs; `rim_check` against the owner's `workspace.json` rim; the right short flap's
lean when tag 11 is visible (tag position relative to the hinge) and the near long flap's state
(standing/hanging, lean) when tag 14 is visible. Two near-wall tags are enough; with one it gives
tag centres only. Tag 26 is near the distorted lower-left image corner and is sometimes rejected
by the 1.5 px reprojection gate; the fit still works from 10 and 27.

## Geometry file and the registration fingerprint

Metric tag poses need the sizes in `pilot/.private/apriltag-geometry.json`. The registration
binding fingerprints that file (`tag_geometry_sha256`), so the carton sizes live in a separate
`carton_tags` section that `TagGeometry` measures like `tags` but leaves out of the fingerprint
(`NON_MEASUREMENT_SECTIONS`). `install_carton_tag_sizes()` adds the section, backs the file up as
`apriltag-geometry.before-carton-<time>.json`, and refuses if the fingerprint would change.
Installed live on 2026-10-10: fingerprint `90e7817a...` before and after; `robot_get_registered_tags`
stays usable. Tests: `tests/test_carton_pose.py` (synthetic carton at several yaws and leans,
refusals, fingerprint neutrality, read-only wrapper dispatch).

## Live check, 2026-10-10 (`.context/carton-pose-20261010/`)

Historical samples in the prior 310.4 mm midpoint frame: head 2085/2623, robot idle, all motors released, five reads over two runs, no motion. Do not reuse these coordinates as current targets; the 273 mm update shifts the right-base translation 18.7 mm toward robot-left:

| quantity | tags | known | difference |
|---|---|---|---|
| near face forward (centre) | 26.0-26.4 cm | "about 20" (9 Oct note also says ~27) | +6 cm vs 20 |
| rim height | 83.1-83.5 cm | 81 (desk 70 + 10.8) | +2.1..+2.5 cm |
| right wall top (left_cm) | -18.4..-18.5 cm | -18 | -0.4..-0.5 cm |
| yaw from square | +0.9..+3.3 deg | | |
| right short flap hinge midpoint | 40.9-41.3 fwd, -17.7..-18.2 left, 83.1-83.5 up | | |

Repeatability within a run was about 0.1 cm; including/excluding noisy tag 26 changed the result by
about 0.4 cm and 2 degrees of yaw. These samples do not establish absolute accuracy. The rim was
2.1–2.5 cm above the owner's nominal 81 cm; tag mounting height and model frame offsets are possible
causes, not proven explanations. Do not subtract that discrepancy as a correction or assume it
cancels all physical error. Tag 14 was interpreted as standing with an outward lean near 12 degrees.

This is historical observation under the earlier read protocol. The new stationary-capture wrapper
has separate offline coverage. A later read at head 2094/2621 is outside the 2085/2623 reference
(9 ticks in pan, limit 3) and must refuse metric carton geometry. No head return was commanded.

## Installing in the pilot

```sh
cd software
# 1. carton sizes (fingerprint-neutral; already done live on 2026-10-10, idempotent)
PYTHONPATH=. python -c "from farm.perception.carton_pose import install_carton_tag_sizes as i; print(i('/path/to/pilot/.private/apriltag-geometry.json'))"
# 2. pilot patch (backs up chat_server.py and model_backend.py under .private/carton-pose-backups/)
python tools/install_carton_pilot_pose.py --pilot /path/to/pilot
# 3. current push policy, then compatible eyes/supervisor guidance
python tools/install_carton_pilot_push_policy.py --pilot /path/to/pilot
python tools/install_carton_pilot_eyes.py --pilot /path/to/pilot --reconcile-supervisor
# 4. install matching farm modules (including xlerobot_geometry) from this revision
# 5. restart the chat while idle with all motors released (installers never restart it)
```

Do not use `.context/carton-pose-20261010/live_check.py` as a read-only diagnostic: that old script
also installs tag sizes and overwrites evidence. For a new read use the installed
`robot_get_carton_pose` tool or `GET /api/carton-pose`, save a new timestamped snapshot, and inspect
`ok`, capture metadata and assumptions. The source installers do not activate a running process;
restart only while the pilot is idle and all motors are torque-off.

## Prompt guidance

Sense `carton` for the near face, rim, hinges and visible flap lean. For the right short flap,
`top_edge_midpoint_if_flat` already includes the signed outward direction, carton yaw and the cosine
height change. Do not use the old `hinge_left + 14*sin(lean)` lateral shortcut or constant top height.
The returned top edge is approach geometry, not an executable pinch/contact target. Missing flap
tag means unknown lean and top edge; it does not imply vertical or folded.

The owner now permits deliberate pushing with the pads or claw body as well as pinch-and-carry
folding. Choose the contact method explicitly; tag geometry does not verify either contact mode.
Use fresh wrist/overview observations to establish the actual contact surface and wrist-camera /
arm-link clearance, then observe the flap and carton after disengagement. Mat resistance is
owner-reported and not a measured slip limit. Preserve the owner's load/contact/travel checks.

Keep `up_cm` in the same model frame for planning; `rim_check` is diagnostic only. If the head has
moved, do not reuse old coordinates or automatically move it during contact. Re-establish the
registered pose in a clear setup or re-register. Tag disappearance, a stalled close, or an edge
beside the pads does not establish a pinch or a completed fold.
