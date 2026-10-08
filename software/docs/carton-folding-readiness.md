# Read-only folding tools in the Gemma wrapper

`farm/perception/gemma_calibration.py:CalibrationRobot` now optionally adds
`robot_folding_status` and `robot_folding_proposal`. The existing pilot composition
is `CalibrationRobot(TagRobot(Robot(args.config)))`; its authenticated robot
client, tag reader, registered-tag verifier and calibration tools remain the
integration path. This change does not install a new pilot, restart a server,
contact a robot, modify camera configuration or provide an execution endpoint.
Existing calibration and single-arm trajectory v1 behavior is unchanged.

The optional switch is a local `folding-readiness.json` beside the existing
`tag-calibration.json`. No file is installed automatically. With no manifest,
the catalog is unchanged and calls to the new names refuse rather than forward.
Catalog enumeration and the new service constructor do not read robot state or
cameras. Calling status or proposal later deliberately performs read-only owner
and registered-tag reads; these must run in the existing authorized pilot.

## Evidence boundaries

Every outcome has `motion_ready:false`, `execution_available:false` and
`motor_writes:0`. There is no argument that can promote these values. These tools
make blocked preparation inspectable, not ready for deployment.

Status separates these domains:

- **Simulation:** verifies the byte hash of a locally selected result file and
  labels its status, stage, error, angles and full-task claim as reported. It does not rerun the simulation, evaluate a
  policy, interpret a `success` boolean as evidence or establish physical success.
- **Kinematics:** loads existing `CalibratedArm` for each arm. That checks pinned
  model assets, saved motor calibration, configured joint zeros/signs, tool and
  station transforms and workspace bounds. Successful loading proves configuration
  consistency; it does not prove those values were independently measured.
- **Calibration:** reuses `read_registered_tags` separately for each arm. That
  verifier checks independent fit/validation residuals, camera stream and
  intrinsics, fixed mount, model and encoder mapping, stationary captures, table
  anchor and current observed tag/FK agreement. Right binds tag 2; left binds tag 4.
  The folding service also compares all six saved ranges per arm to owner state.
  Saved ranges are not actual homing-offset/limit-register readbacks. Physical
  register, jaw contact, tool-offset and zero/sign measurement verification remain
  explicitly false here.
- **Owner:** reports the fresh stationary telemetry and advertised paired protocol
  separately from local `validate_profile` v2 schema checks. Matching profile and
  binding hashes are required for proposal availability. Claimed `motion_ready`,
  `calibrated`, `COMMISSIONED` or similar booleans do not certify owner deployment
  or commissioning. The library cannot prove a watchdog or interrupt a blocked
  synchronous serial callback.
- **Policy:** no complete physically validated adaptive four-flap policy is
  installed by this wrapper. A simulation artifact cannot remove this blocker.

The constant blockers include no execution endpoint, independently unverified
paired commissioning/watchdog/I/O timeout, unverified current physical motor
register calibration, missing independent exact-trajectory collision certificate
and missing complete physical policy. Missing, stale, mismatched or malformed
artifacts add concrete per-domain blockers. STOP is never cleared or retried.

## Local manifest

All file references have exactly `path` and `sha256`; hashes are SHA256 of the
exact file bytes. Relative manifest paths resolve beside the manifest. Relative
`calibration_file` and `model_directory` in each kinematics artifact resolve
beside that kinematics artifact. No model tool argument accepts file paths,
calibration values, profile limits or execution switches.

```json
{
  "schema": 1,
  "arms": {
    "left": {
      "tag_config": {"path": "left-tag-config.json", "sha256": "<byte SHA256>"},
      "registration": {"path": "left-registration.json", "sha256": "<byte SHA256>"},
      "kinematics": {"path": "left-kinematics.json", "sha256": "<byte SHA256>"}
    },
    "right": {
      "tag_config": {"path": "right-tag-config.json", "sha256": "<byte SHA256>"},
      "registration": {"path": "right-registration.json", "sha256": "<byte SHA256>"},
      "kinematics": {"path": "right-kinematics.json", "sha256": "<byte SHA256>"}
    }
  },
  "simulation": {"path": "simulation-result.json", "sha256": "<byte SHA256>"},
  "profile": {"path": "paired-profile.json", "sha256": "<byte SHA256>"},
  "bindings": {"path": "paired-bindings.json", "sha256": "<byte SHA256>"}
}
```

This is a schema example with placeholders, not a commissioning configuration.
The actual two-arm calibration/registration files are not supplied by this change.
The per-arm profile bindings `calibration_sha256`, `registration_sha256` and
`kinematics_sha256` must equal those artifact byte hashes. Registered-tag results
also contain their existing canonical-JSON `registration_sha256`; that is a
separate verifier identity and must not be confused with a file-byte hash.

The owner must advertise `bimanual_trajectory_protocol`,
`bimanual_trajectory_version`, canonical `profile_sha256` and canonical
`bindings_sha256` matching the existing paired owner component. This change does
not add those advertisements or wire the component into an actual owner. Current
owners lacking them remain blocked. The containing commissioning process still
has to independently check the record referred to by the profile, each physical
calibration and camera/station identity, rather than copying a client claim.

## Read-only proposal contract

`robot_folding_status({})` returns domain evidence and blockers.
`robot_folding_proposal` accepts only:

```json
{
  "targets": {
    "left": [[[1,0,0,0.2],[0,1,0,0],[0,0,1,0.2],[0,0,0,1]]],
    "right": [[[1,0,0,0.2],[0,1,0,0],[0,0,1,0.2],[0,0,0,1]]]
  },
  "segment_seconds": 0.5
}
```

The numeric poses above illustrate syntax only; they are not verified targets.
Targets are equal-length lists of 1–20 explicit station-frame tool transforms in
metres. Orientation is always constrained. Existing `CalibratedArm.plan` solves
each arm from fresh measured encoders and validates rounded encoder residuals.
Both jaw encoder targets retain their measured starting positions. No grasp,
contact, retention strategy or flap policy is inferred from these tool poses.

The service requires stationary, fresh both-arm telemetry (age at most 0.2 s,
skew at most 0.1 s), no pending commands, stable owner identity/counters/ranges,
matching pinned configurations, passing existing registered-tag reads and matching
v2 profile/camera bindings. It brackets evidence and IK with fresh owner snapshots
and refuses the whole pair on an asymmetric IK or corridor failure. Existing
`JointTrajectory` validates all twelve joint corridors and retimes the whole
paired path against configured velocity/acceleration limits and the 30-second
supervision horizon. These checks remain mathematical preparation.

The returned proposal has protocol `carton_bimanual_trajectory`, schema `2`, but
`op:proposal_only`; it lacks a command id, owner session, scene identity and
independent collision certificate. It is deliberately invalid as a
`BimanualTrajectoryOwner.start` command. `proposal_sha256` is only an identity,
never a collision-check result. A later independently commissioned execution
integration must obtain the actual observed scene and validate the exact profile,
bindings, scene and waypoint identity before creating a separately authorized
command. This wrapper will still return `motion_ready:false` until explicitly
extended with a reviewed evidence verifier and sole-owner integration.

## Offline verification

`tests/test_folding_readiness.py` uses only fake owner telemetry, a fake registered
reader and fake IK. It covers claim booleans, prepared evidence, missing calibration,
changed hashes, wrong arm/tag, STOP, stale telemetry, changed camera identity or
stream, owner/profile mismatch, an owner change during capture, asymmetric IK and
corridor failures, zero-write catalog/constructor behavior and a read allowlist.
The existing registered-tag and kinematics suites cover their own mathematical
and synthetic-camera validation. No test constitutes physical calibration,
physical folding, deployed owner operation or successful motor release.
