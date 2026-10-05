# Session reconciliation onto main

2026-10-05 (Asia/Tokyo). The human user explicitly requested reconciling the code used during this session, saved calibration and handoff documents onto `main`. This supersedes the earlier source-publication approval blocker recorded in the historical handoff.

## Included

- Used carton visual controller and its kinematics/perception dependencies, guarded sole-owner scripts and tests.
- Current explicit head-plus-OAK preparation, reader validation, continuous profile binding and metric grasp observation.
- Analytical model/encoder/normalized-unit conversion with measured-reference validation; corrected analytical FK/IK conventions.
- Existing main OAK list/capture/preview behavior, plus the session's immutable aligned RGB/depth streaming support.
- Saved motor calibration and README from `codex/robot-calibration-snapshot`.
- `ROBOT_HANDOFF_2026-10-05.md`, the last progress report, reference guide/templates and concise numerical/camera evidence.
- Exact session camera/viewer/phone sources and the old motor owner/isolated claw utility under `software/docs/session-archive-2026-10-05/`. These are historical snapshots with deployment caveats, not installed entry points or additional active owners.

Main's existing planter/R3.2/R3.3 work is preserved. Unused experimental `owner_trajectory`, `trajectory_binding`, `trajectory_transport` adapters and their tests are excluded. Runtime credentials, certificate/private keys, authorization configs, recordings, raw motor logs, model downloads, binaries and environments are excluded.

## Validation

The combined checkout's full suite produced 504 passing tests, eight optional skips and four dataset failures caused by the sandbox denying writes to the default Hugging Face cache. Rerunning exactly those four with a workspace cache passed all four. Thus all 508 executed tests passed across the full run and targeted rerun; no failed assertion remained. Python compilation and whitespace validation also passed. These are software checks and do not establish a physical pickup or fold.

## Physical/deployment state remains unchanged

The paddle has not been securely grasped or lifted and the carton has not been folded. The latest left-claw opening moved about 4.5 degrees but settled short of its requested endpoint and then released. All capture/viewer processes and recurring reporting were stopped at handoff. This reconciliation sent no motor commands and did not change live calibration.

The packaged head/OAK owner is source-tested but had not replaced the older work-directory owner during physical operation. Do not interpret publication as that deployment having happened. Physical model zero/sign reference, camera-to-arm/station registration, reliable visible/depth grasp features and a validated approach/lift path remain uncommissioned. The blank reference templates deliberately fail validation.

The earlier handoff/progress files are retained as dated records; statements that source publication was pending describe their original time, not the current reconciliation.
