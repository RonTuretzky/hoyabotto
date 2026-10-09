# Automatic fold-policy refit

`tools/refit_fold_policy.py` replaces setup step 7 ("send the measurements; the policy is refitted") with one
command. It takes the measured camera and station files and produces a policy trained for them:

1. re-render the recorded demonstrations with the measured cameras;
2. rebuild the LeRobot dataset;
3. retrain ACT on Hugging Face Jobs with the recipe that scored 63/63;
4. evaluate the checkpoints in simulation;
5. hand back the checkpoint path for the chat config.

**Simulation and cloud training only. Nothing in it talks to the robot.**

By default it is a dry run: it prints the plan, every command, and the estimated time and cost, and it writes
nothing. It uploads data or starts paid cloud work only with `--launch`.

## What the owner does

Nothing beyond the measurements themselves and one approval:

1. Produce the measurement files with the measuring tools (setup steps 1–6):
   - the head camera pose: `tools/camera_pose_from_tag.py`;
   - the two wrist lenses: `tools/calibrate_camera_checkerboard.py`;
   - the station JSON.
2. List them in a manifest (below).
3. Read the dry-run plan. If it looks right, approve the spend by running the same command with `--launch`
   (about $2.4; the job timeout caps it at $5).

The agent or the tool handles everything else. That includes the "Needs a human" list at the end of
`report.md`, which covers the runtime checks that remain before any robot run.

## Commands

Run from `software/` with the project venv:

```sh
PYTHONPATH=. python tools/refit_fold_policy.py measurements/manifest.json              # dry run: plan, commands, time, cost
PYTHONPATH=. python tools/refit_fold_policy.py measurements/manifest.json --run-local  # local stages only, no network
PYTHONPATH=. python tools/refit_fold_policy.py measurements/manifest.json --launch     # everything, including the cloud
```

Useful options:

| option | effect |
|---|---|
| `--json` | print the plan as JSON (statuses, commands, minutes, dollars per stage) |
| `--allow-rerecord` | allow new demonstrations when the measured station differs from the recorded one (see below) |
| `--checkpoints DIR` | evaluate existing checkpoints (`DIR/checkpoints/<step>/pretrained_model`) instead of training; works with `--run-local` |
| `--eval-steps 15000 20000 25000` | checkpoints to evaluate (default). Pass multiples of 5000 only. `last` is never used |
| `--min-success .95` | install gate on the held-out success rate |
| `--pilot <pilot>` | if the gate passes, also set `checkpoint` in `<pilot>/.private/fold-policy.json` (a backup is kept) |
| `--run-dir`, `--output-root` | where the run lives; default `<output-root>/fold-refit/<name>` |
| `--hub-user` | Hub namespace for the private dataset and model repos (default `RonTuretzky`) |
| `--allow-unmeasured` | accept `"measured": false`, for check runs on the model values |

## Manifest

Schema `xlerobot-fold-refit-manifest/1`. Paths are relative to the manifest file. Only `schema` and `name` are
required.

```json
{"schema": "xlerobot-fold-refit-manifest/1",
 "name": "measured-01",
 "measured": true,
 "station": "station.json",
 "head_camera": "head-pose.json",
 "wrist_lenses": {"left": "left_wrist-640x480.json", "right": "right_wrist-640x480.json"},
 "appearance": {"arm_rgba": [0.05, 0.05, 0.05, 1]},
 "demos": {"train_batches": ["/…/fold-demos/batch-220-01"], "eval_batches": ["/…/fold-demos/batch-220-02"]}}
```

| key | what it accepts |
|---|---|
| `name` | 1–40 characters from `a-z`, `0-9` and `-`. Used in the run directory and the Hub repo names |
| `measured` | must be `true` unless `--allow-unmeasured` is given |
| `base` | the measurement file that fills any gap. Default: `profiles/fold-station-xlerobot-220.json`, model-derived |
| `station` | a station-measurement file (its `station` block is used), or a bare object with `base_height_above_table_m`, `base_line_to_table_edge_m` and `base_spacing_m` |
| `head_camera` | a camera entry in the `arm_base` frame: `position_m`, `rotation_cv` or `look_at_m`, and `intrinsics` or `fovy_deg`. The full output of `tools/camera_pose_from_tag.py` also works (its `camera_entry` is used) |
| `wrist_lenses` | the outputs of `tools/calibrate_camera_checkerboard.py`. The lens position and orientation come from the robot model, because the mount fixes them; only the field of view comes from the calibration |
| `appearance` | optional, inline or a path: arm and table colour, table size, hidden markers (`carton/folding_station_measured.py`) |
| `demos` | demo batches. Training uses `train_batches`; held-out starts come from both. The default is the proven split: train on batch-220-01, evaluate on the held-out starts of both batches |

Validation collects every problem before refusing. It refuses:

- a missing file or an unknown key;
- an improper rotation, or a head pose not in `arm_base`;
- a principal point outside the image;
- a lens reprojection error above 2 px, or fewer than 10 calibration photos;
- a station file marked `"measured": false`;
- a carton that is not 10 mm from the table edge.

It warns, but continues, on:

- parts that fell back to model values;
- a reprojection error above 1 px;
- significant lens distortion (the simulation renders a pinhole camera);
- a non-4:3 image, which the policy sees as a centred crop that the robot runtime must reproduce;
- a head pose that is implausible next to the model's.

## Re-render or re-record

The scripted demonstrator registers through its own `station` camera and never through the policy cameras.
Changing the cameras is therefore exact by re-rendering. That takes about 3 minutes for 640 trials.

A measured station that differs from the recorded one by more than 2 mm can't be fixed by re-rendering, because
it moves the arms relative to the carton. This applies to the arm-base height above the table, the setback from
the base line to the table edge, and the base spacing. The validate stage detects the difference and prints it.
The record stage then stops with:

> the measured station differs from the recorded one (re-rendering cannot match it); re-run with
> --allow-rerecord to record new demonstrations

With `--allow-rerecord`, the tool first records a 16-episode pilot at the measured station with
`tools/record_measured_fold_demos.py`. It continues only if the pilot reaches 80% success: the controller was
tuned at a 120 mm base height. It then re-records every batch with the same seeds and randomisation ranges, from
each batch's `batch.json`. That adds about 40–70 minutes at 5 workers.

## Stages, time and cost

These are the estimates printed by the dry run for the default split. Training uses 287 episodes from
batch-220-01, and evaluation uses 62 held-out starts from batch-220-01 and batch-220-02.

| stage | where | what | estimate |
|---|---|---|---|
| validate | local | compose `measurement.json`, check it, compare the station with the recorded scenes | seconds |
| record | local | only when the station differs, with `--allow-rerecord` | 40–70 min |
| restage | local | `tools/restage_fold_scenes.py`, one mirror batch per source batch (demo files linked, new `scene.xml`) | ~3 min |
| dataset | local | `tools/fold_demos_to_lerobot.py --cameras front=front left_wrist=left_wrist right_wrist=right_wrist`, 240×320, seed % 10 held out | ~45 min |
| holdout | local | the dataset's held-out starts plus those of the evaluation-only batches | ~1 min |
| push | cloud | private Hub dataset `<user>/carton_fold_refit_<name>_<hash>`, about 9 GB | ~8 min |
| train | cloud | `lerobot-train --job.target=a100-large`: ACT chunk_size=100, n_action_steps=100, batch 32, lr 3e-5, 25k steps, a checkpoint every 5k pushed to the Hub, timeout 2 h | ~57 min, **~$2.4** (cap $5) |
| wait | cloud | poll `huggingface_hub.inspect_job` until the job ends | (inside train) |
| download | cloud | the evaluated checkpoints only, then drop `"dtype": null` (the cloud trainer writes it; lerobot 0.6.1 rejects it) | ~2 min |
| eval | local | `tools/eval_fold_policy.py --temporal-ensemble .01` on every held-out start for checkpoints 15k/20k/25k, shards of 8, 4 in parallel on MPS | ~85 min |
| pick | local | rank by successes, then carton slide (max, then median), then flap penetration, then the earlier step. Never "last" | seconds |

The total is about 3.3 hours of wall time and about $2.4 of A100 time. The 8 October runs of the same recipe
took 47 minutes of training, about $2.4 per run.

The private model repo is named `<user>/act_carton_fold_refit_<name>_<hash>`. The Hugging Face token is read
only at launch, through `huggingface_hub`'s standard login (`hf auth login`). The tool never prints it.

## Outputs

These are written to the run directory, by default `…/output/fold-refit/<name>/`:

| file | contents |
|---|---|
| `measurement.json` | the composed station-measurement file every tool uses |
| `validate.json` | provenance, warnings, mode, recorded station, and the policy crop of each camera |
| `demos-restaged/<batch>/` | the re-rendered demonstrations |
| `dataset/` | the LeRobot dataset |
| `holdout.json` | the held-out starts |
| `train/checkpoints/<step>/pretrained_model` | the downloaded checkpoints |
| `eval/<step>/shard-*/` | evaluation results and a video per checkpoint |
| `eval/<step>/summary.json` | the score of each checkpoint |
| `report.json`, `report.md` | the score table, the chosen checkpoint, the gate result, and "Needs a human" |
| `fold-policy-checkpoint.json` | `{"checkpoint": …, "model_sha256": …}`, the value for `checkpoint` in `<pilot>/.private/fold-policy.json`. It is `null` when the gate fails |
| `stages/<stage>.json` | the input hash and outputs of each finished stage. A submitted job id is stored in `stages/train.json` |
| `logs/` | one log per command |

## Resuming

Every stage records the hash of its inputs. Those inputs are:

- the manifest and every file it names;
- a fingerprint of the demo batches;
- the source of the tool that the stage runs;
- the stage's parameters;
- the hash of the stage before it.

On re-run, a stage whose outputs exist and whose inputs are unchanged is skipped. A changed stage is rebuilt. The
tool removes old outputs only inside the run directory. A symlinked output is unlinked; its target is left alone.

Changing one camera file reruns everything from restage on. Changing only `--height`/`--width` reruns the dataset
and the stages after it.

A submitted job is never submitted twice: re-running polls the recorded job id. If the job ended early, the
checkpoints that reached the Hub are evaluated. Evaluation shards that already finished are not rerun.

## After the refit

The refit doesn't change the robot or the chat by default. To put the new checkpoint into the chat config:

- pass `--pilot <pilot>`, which sets `checkpoint` when the gate passes, or
- copy the `checkpoint` value from `fold-policy-checkpoint.json` into the config yourself.

The chat vouches for the model's sha256, so a new chat dry run is required before any fold run.

Several things in `report.md` under "Needs a human" are outside the simulation:

- **Head pose.** At run time, the head must have the same tilt and pan it had when the head camera was measured.
- **Wrist crop.** If a wrist image is not 4:3, the robot runtime must feed the policy the same centred crop.
  `carton/fold_policy_runner.py` does not crop today.
- **Wrist distortion.** If the wrist lens distortion is significant, undistort the image before the policy.
- **First robot run.** It still follows the setup slides: one short flap first, with STOP in hand.

## Tests

`tests/test_refit_fold_policy.py` makes no network calls; a guard fails any test that opens an internet socket.
It covers:

- schema validation and its refusals;
- the dry-run plan for a manifest whose "measurements" are model-derived profile values;
- detecting a station change and gating the re-recording on the pilot;
- stage skipping, staleness and rebuilding;
- evaluating and ranking existing checkpoints, the install gate, and the `dtype` fix.

A smoke test re-renders three recorded trials at 48×64 into a one-episode dataset, with two held-out starts, in
about 25 s. It is skipped when the recorded demos are absent.
