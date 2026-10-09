# Paid refit qualification

The October 10 v11 job was canceled because all older scenes used 220 mm arm
spacing. The official CAD has **310.4 mm** between pan axes, but on October 10
the owner measured **273 mm** and explicitly confirmed the reference points
were the shoulder-pan rotation axes. Use **0.273 m** for this physical station.
Passing results at either 220 mm or 310.4 mm do not qualify this measured refit.
Existing models remain available for separate experiments.

This workflow is simulation only. It does not authorize robot commands.

## Station file

The local v12 launcher reads `station-measured-20261010.json` beside itself.
Populate it only from owner measurements and the server thread's evidence:

| Key | Required evidence or value |
| --- | --- |
| `measured` | `true`, only after the measurements arrive |
| `measurement_source` | Path or description of the actual measurement record |
| `frame` | `forward_so101_base_link` |
| `base_spacing_m` | `0.273`, owner-confirmed physical pan-axis separation |
| `base_height_above_desk_m` | `0.05`, approximate flat mounting plane height reported by owner |
| `base_to_table_edge_m` | `0.2488353`: 100 mm pan-to-cart edge + 110 mm cart/table gap + 38.8353 mm frame conversion |
| `desk_height_m` | `0.7` |
| `carton_inset_m` | `0.0`, interpreting owner placement "on the edge of the table" |
| `placement_mode` | `owner_will_match_target`: owner will place the carton at the specified target |
| `physical_placement_verified` | `false` until fresh physical validation |
| `verify_placement_before_robot_execution` | `true`; mandatory before any robot run |

In the forward-facing SO101 model, the pan axis is 38.8353 mm forward and
62.4 mm above the base origin. Do not interchange mounting-plane height,
pan-axis height, and the rotated CAD base-body origins. The 500 × 480 mm desk
is fixed in the recorder; carton inset is an explicit `--carton-inset` argument.
The owner wrote "100m" in answer to the millimetre pan-to-cart-edge question;
this is interpreted as 100 mm. Keep this provenance with the station.

The previous +29.1 mm mounting height and 180 mm base-origin setback are
**diagnostic assumptions**, not measurements. Tag-based rim height currently
has a roughly 20 mm systematic discrepancy from tape and cannot replace them.

## Camera and teacher

`config/front-camera-registered-20261010.json` retains the measured
`base_from_camera` transform. The recorder resolves it against the current
scene's `right_base_link` on every run. Its `pos`/`xyaxes` values are a preview
of one scene, not fixed world coordinates for every station.

Validate the registration's frame convention, source hash, head reference pose,
and rendered policy view against the target scene. Show it before training. The
owner agreed to match carton placement later: centre it between the arms, with
its nearest bottom wall flush with the nearest desk edge (210 mm from pan axes).
Current tag detection returned no usable metric tags; physical alignment is
unverified and is a gate before robot execution, not simulation-only training.
No registration is rewritten to fit the target.

The launcher also requires `qualified-teacher-v12.json`, with a `flags` array
containing the exact locally qualified grasp/approach options. Supported flags:

```
--along --radius --axis-sign --pinch-normal-tilt-degrees --pre-height
--clearance --prepare-near-degrees --full-grip-orientation
--left-press-along --near-release-lift --start-jitter-m --teacher-position
```

Geometry, cameras, speed caps and timing cannot be overridden by this file.
Teacher tuning must preserve reach, collision, grasp, tracking and contact
audits. Record unsuccessful seeds as well as successful ones. Teacher success
and replay success do not establish learned-policy or physical success.

## Qualification report

`robot-conditions-verification-v12.json` uses schema 3. It must contain:

- `schema: 3`, `scope: "simulation_training_only"`, `physical_execution_ready: false`,
  `verify_placement_before_robot_execution: true`.
- `station_sha256`: `tools.refit_launch_preflight.digest(station)` of the complete
  measured station object.
- `pipeline_args`: the exact ordered argument list used by the launcher,
  excluding the dataset/model repository arguments.
- `source_files`: the complete `files` mapping from the portable bundle's
  `manifest.json`, including Python, camera configuration and simulation assets.
- A true field for every check below, plus `evidence[check]` containing `path`
  and `sha256`. Evidence paths resolve relative to the launcher's directory.
- `projected_private_storage_bytes`: a positive projection for all new private
  dataset, source, demonstration and checkpoint artifacts, with headroom.

Checks in `tools.refit_launch_preflight.CHECKS`:

| Check | Required result |
| --- | --- |
| `target_scene_camera_contract_passed` | Target scene coordinates, camera frame/projection and registration provenance verified; physical alignment still pending |
| `tests_passed` | Relevant code and physics contract tests |
| `capped_teacher_folds` | Reproducible folds and holds across varied starts on measured geometry, with all contact audits |
| `replay_under_robot_limits_passed` | Audited replays at 4 Hz, speed cap and step clamps |
| `dataset_conversion_passed` | Readable three-camera data at 4 Hz, delayed images, correct next-tick actions and disjoint holdout seeds |
| `local_training_smoke_passed` | Small local optimizer run, checkpoint load and inference at the expected camera/joint shapes |
| `frozen_bundle_smoke_passed` | Portable source/assets executed after relocation; use this bundle for qualification |

Hash the actual evidence files. Do not manufacture a passing report from the
October 9/early October 10 boolean-only report or from a startup-only smoke test.
Changing station dimensions, teacher flags, camera registration or any bundled
source/assets invalidates the report and requires requalification. A successful
dry run performs local checks only; it uploads nothing and allocates no hardware.

## Final storage and submission checks

Immediately before submitting, create `storage-preflight-v12.json` from a fresh
account-wide private-storage observation. Fields are `source`, `checked_at`
(Unix seconds) and `available_bytes`. The launcher requires it to be at most
15 minutes old and to cover the report's projection. No automatic deletion is
authorized by this workflow.

The launcher refuses an active prior job, an existing run with the same label,
an uncertain earlier submission, or unreconciled model results. Source is
uploaded by immutable revision and archive checksum. Eight H200s and the
five-hour provider timeout follow the existing user authorization; the internal
watchdog is 295 minutes. This is a maximum runtime, not an ETA.

After submission, inspect the returned job directly and keep monitoring
read-only and infrequent. Status uploads inside the job are best-effort and
throttled; monitoring failure must not kill optimizer training. No automatic
paid retry. Successful training still requires held-out policy evaluation before
any separately authorized physical test.
