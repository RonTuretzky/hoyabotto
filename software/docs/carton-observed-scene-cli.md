# Reproducible offline observed-scene diagnostic

`tools/diagnose_folding_observed_scene.py` replays saved states **only to render
sensor images**, runs the normal PixelPort RGB-D estimator, constructs the
observation-derived collision scene, and checks both arms' captured
configurations. It does not advance dynamics, command an arm, execute or plan
a trajectory, score folding, or certify a physical station.

From `software`:

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_folding_observed_scene.py \
  --run /path/to/saved/offline/run \
  --frames 0,527,529 \
  --out /path/to/new/evidence-directory \
  --uncertainty-profile synthetic-envelope-v1
```

The input folder must contain `scene.xml` and `folding-frames.json`. Frames may
also be space-separated. They must be unique, nonnegative and ascending; their
capture times must strictly increase. Output must be a new directory. Existing
evidence is never overwritten. `--seed` defaults to zero and is recorded.

## Supported setup and refusal

The initial version accepts one explicit hypothetical station profile:
60 mm arm-base height, 150 mm base-to-table edge spacing, 10 mm carton-to-edge
nominal spacing, 300 mm base spacing, the declared world-mounted `station`
camera, both table anchors, and the proposed carton floor markers 24 and 25.
It checks a geometric setup fingerprint before rendering. That fingerprint
covers 30 static cart/table obstacles, camera declaration, robot bases and
housing tags, marker patterns/mounts, carton-local geometry and hinge axes.

This allowlist is **not physical calibration**. It was derived from the
archived local-station scene used by the earlier component experiment. The
free carton's initial world pose and joint material/rest parameters are
excluded from the setup fingerprint. Carton world pose and flap angles used
by the planner must still come from pixels. An unknown object, changed marker
mount, different carton, loaded contents volume, changed static layout or
camera setup produces `setup_refused`; the CLI does not infer a replacement
calibration or quietly omit the obstacle.

The supported fingerprint is
`2cbe4b69b2c7c883d77fd43abc64d4e95c7123441d640045187f6ec64bbe87e8`.
Supporting another setup requires a separately reviewed explicit declaration
and regression evidence. Do not merely copy a new input's hash into the
allowlist to suppress a refusal.

The source scene's robot-only template is exported as offline setup. Its two
robot roots and external assets are validated and hashed. Scene objects,
actuators, keyframes and sensor state do not enter that template. A separate
robot-only FK model checks the visible housing markers using only named robot
encoders. PixelPort receives a camera-intrinsics/clock interface, never the
renderer `MjData` or an object-truth method.

## Named synthetic uncertainty profile

`--uncertainty-profile` is mandatory. The only current profile is
`synthetic-envelope-v1`:

| Quantity | Declared bound or setting |
| --- | --- |
| Robot base translation / rotation | 0.5 mm / 0.1° |
| Robot collision geometry | 0.2 mm |
| Static station points | 0.5 mm |
| Each carton dimension | 0.5 mm |
| Carton registration translation / rotation | 1 mm / 0.1° before any larger observed marker disagreement |
| Each flap angle | 0.5° |
| Each named robot encoder | 0.001 rad |
| Error-bound horizon | 0.1 s of declared simulation time |
| Render size / camera | 1280×720 / `station` |
| Added depth noise / dropout | 0.8 mm standard deviation / 25% |

All numbers are explicit simulation assumptions. No authoritative measured
physical station profile is known. Small tag, plane or anchor fit residuals
do not establish transform accuracy and do not reduce these bounds. The
adapter, declarations and results retain `offline:` identities and
`physical_calibration_verified=False`.

The scene wrapper preserves the existing nominal transit clearance with
`max(0.006 m, computed robot uncertainty clearance) + extra clearance`. This
does not claim an additional 6 mm physical gap after worst-case error. The
current synthetic profile produces about 10.544 mm required robot clearance,
so the new nominal floor does not change these component results.

## Report semantics

Each requested frame runs two explicitly recorded policies:

1. `all_required`: all four fresh identified flap estimates are required.
2. `majors_required_optional_sweeps`: both majors are required; any missing or
   unsupported short flap becomes a full conservative hinge sweep.

Outcomes distinguish `sensor_refused`, `scene_refused`,
`configuration_refused`, and `configuration_clear`. A clear configuration
only means the captured nominal joint positions passed both geometric checks
with the declared margins. It does not establish a motion path, safe contact,
closed carton, physical measurement, or hardware readiness. No `fold_success`
claim is made. Exit code zero means the offline component diagnostic completed;
its cases may all refuse. Unsupported setup or invalid invocation exits two.

The normal PixelPort observer retains its sequence, calibration and priors
across requested frames. Sparse replay is not a replacement for the full
sensor stream. A failed sensor registration resets the observer before the
next requested frame so an initial housing-tag failure cannot be bypassed.
Fresh registration may therefore refuse a later isolated frame.

`result.json` records:

- Requested frames, both policies, PixelPort readings, same-frame depth/tag
  quality, pixel-derived camera poses, exactly twelve named robot encoder
  captures, adapted packets, scene metadata and collision/refusal details.
- The full named uncertainty profile and explicit station/carton declarations,
  with profile and combined-assumption SHA-256 hashes.
- Executed local Python source snapshots and per-file/manifest hashes; archived
  run sources are separately copied and hashed when present.
- Exact input XML/frame-file copies and hashes, external render/robot asset
  file hashes, asset-manifest hash, robot-template hash and setup fingerprints.
- Python, MuJoCo, NumPy and OpenCV versions, plus checks for source or asset
  changes during execution.

Saved `frame-*-rgb.npy` and `frame-*-depth.npy` files are the actual arrays
exposed to PixelPort, including its added depth noise/dropout, with file
hashes, shapes and dtypes. They contain no segmentation or contact IDs. Asset
bytes are frozen for rendering; original assets remain external files and are
not redistributed. The recorded hash manifest must still match them for a
later reproduction. Missing asset identities are explicitly null when setup
is refused before asset loading.

The macOS renderer may warn that `ARB_clip_control` is unavailable and depth
precision is limited. The component is not a depth-sensor accuracy validation.

## Verification

```sh
PYTHONPATH=. .venv/bin/python -m pytest \
  tests/test_diagnose_folding_observed_scene.py \
  tests/test_folding_observed_adapter.py \
  tests/test_folding_observed_scene.py tests/test_folding_paths.py -q
```

175 relevant tests pass. Tests cover unsupported setup, frame ordering,
preservation of existing output, source/asset hashing, marker-registry
restoration, separation of render state from robot FK/encoder captures,
hidden-object-state invariance, and distinct configuration/refusal statuses.

The actual CLI replay of frames 0, 527 and 529 is saved at
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/observed-scene-cli-2026-10-07-02/`.
The source run is
`claws-retention/parallel-near-center-release-01/trial-000/run` under the same
`bimanual-fold-sim` output root. Frame 0 refuses the unidentified right short
flap under the all-required policy; the swept policy builds but refuses the
captured configuration's uncertainty clearance. Frames 527 and 529 refuse
left-wrist encoder intervals beyond the declared joint limit. No angle is
filled, encoder clipped, collision gate weakened, or error bound reduced.
