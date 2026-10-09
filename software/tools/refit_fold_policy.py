"""Refit the carton fold policy to measured cameras/station in one command. Simulation and cloud training only;
nothing here talks to the robot.

Replaces setup step 7 ("send the measurements; the policy is refitted"). From one measurements manifest
(`xlerobot-fold-refit-manifest/1`, below) it:

  validate  compose one station-measurement file (carton/folding_station_measured.py schema) from the head
            camera pose, wrist lens calibrations and station JSON (model-derived values fill any gap), check
            it, and compare the station with the recorded demonstrations;
  record    only when the measured station (base height, setback, spacing) differs from the recorded one,
            which re-rendering cannot fix: a 16-episode pilot at the measured station gates a full
            re-recording (tools/record_measured_fold_demos.py). Needs --allow-rerecord;
  restage   re-render the demonstrations with the measured cameras (tools/restage_fold_scenes.py);
  dataset   LeRobot dataset of the training batches (tools/fold_demos_to_lerobot.py, head + both wrists);
  holdout   held-out starts: the dataset's own plus those of the evaluation-only batches;
  push      [--launch] the dataset to a PRIVATE Hub dataset repo;
  train     [--launch] the proven recipe on Hugging Face Jobs: ACT chunk_size=100 n_action_steps=100,
            batch 32, lr 3e-5, 25k steps, checkpoints every 5k, a100-large (docs/carton-fold-policy.md);
  wait      [--launch] poll the job until it ends;
  download  [--launch] the evaluated checkpoints; drop the cloud trainer's `dtype: null` (lerobot 0.6.1);
  eval      closed loop on every held-out start with temporal ensembling 0.01 (tools/eval_fold_policy.py),
            each chosen checkpoint, sharded and run in parallel on this Mac;
  pick      choose the checkpoint by score (never "last"), write report.json / report.md and
            fold-policy-checkpoint.json (the `checkpoint` value for <pilot>/.private/fold-policy.json).

Modes:
  (default)     dry run: print the plan, every command, and estimated time and cost. Writes nothing.
  --run-local   run the local stages (validate .. holdout; eval + pick too with --checkpoints DIR). No network.
  --launch      everything, including the cloud stages (uploads the dataset, spends about $2.5 on an A100).

Resumable: every stage records the hash of its inputs in <run>/stages/<stage>.json and is skipped when its
outputs exist and the hash is unchanged; a stale stage's outputs (inside the run directory only) are
removed and rebuilt. A submitted job is never submitted twice: re-running resumes polling it.

    PYTHONPATH=. python tools/refit_fold_policy.py measurements.json              # plan only
    PYTHONPATH=. python tools/refit_fold_policy.py measurements.json --run-local  # local stages
    PYTHONPATH=. python tools/refit_fold_policy.py measurements.json --launch     # the whole refit

Manifest (paths relative to the manifest file; every key but `schema` and `name` optional):

    {"schema": "xlerobot-fold-refit-manifest/1",
     "name": "measured-01",
     "measured": true,
     "base": "profiles/fold-station-xlerobot-220.json",
     "station": "station.json",
     "head_camera": "head-pose.json",
     "wrist_lenses": {"left": "left_wrist-640x480.json", "right": "right_wrist-640x480.json"},
     "appearance": {"arm_rgba": [.05, .05, .05, 1]},
     "demos": {"train_batches": [".../fold-demos/batch-220-01"], "eval_batches": [".../fold-demos/batch-220-02"]}}

- `station`: a station-measurement file (its `station` block is used) or a bare object with
  base_height_above_table_m, base_line_to_table_edge_m, base_spacing_m.
- `head_camera`: a camera entry (position_m, rotation_cv or look_at_m, intrinsics or fovy_deg; arm_base
  frame), or the output of tools/camera_pose_from_tag.py (its `camera_entry`).
- `wrist_lenses`: tools/calibrate_camera_checkerboard.py outputs; the lens sits where the robot model puts it
  (the mount fixes it), only the field of view comes from the calibration.
- Anything not given comes from `base` (default: the model-derived profile) and is reported as such.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

SOFTWARE = Path(__file__).resolve().parents[1]
if str(SOFTWARE) not in sys.path:
    sys.path.insert(0, str(SOFTWARE))

from carton import folding_station_measured as fsm  # noqa: E402

MANIFEST_SCHEMA = 'xlerobot-fold-refit-manifest/1'
OUTPUT_ROOT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output')
DEFAULT_BASE = SOFTWARE / 'profiles/fold-station-xlerobot-220.json'
DEFAULT_TRAIN_BATCHES = ('fold-demos/batch-220-01',)
DEFAULT_EVAL_BATCHES = ('fold-demos/batch-220-02',)
DEFAULT_HUB_USER = 'RonTuretzky'
POLICY_CAMERA_ARGS = ('front=front', 'left_wrist=left_wrist', 'right_wrist=right_wrist')
NAME_RE = re.compile(r'^[a-z0-9][a-z0-9-]{0,39}$')
TASK = 'both-shorts'

# The recipe that gave 63/63 unseen starts (docs/carton-fold-policy.md, "Longer action chunks fix the stall").
RECIPE = {'policy': 'act', 'chunk_size': 100, 'n_action_steps': 100, 'batch_size': 32, 'optimizer_lr': 3e-5,
          'steps': 25000, 'save_freq': 5000, 'num_workers': 10, 'target': 'a100-large', 'timeout': '2h'}
DEFAULT_EVAL_STEPS = (15000, 20000, 25000)

# Estimates, from the 8 October runs on this Mac and on HF Jobs.
A100_LARGE_USD_PER_HOUR = 2.50      # HF Jobs a100-large; check `hf jobs hardware`
TRAIN_S_PER_STEP = .113             # batch 32, chunk 100: 25k steps in 47 min
JOB_OVERHEAD_MIN = 10               # image pull, dataset download, checkpoint uploads
RESTAGE_S_PER_TRIAL = .2            # 320 trials in about a minute
CONVERT_S_PER_EPISODE_8_WORKERS = 9.5   # 287 episodes in 45 min at 8 workers, 3 cameras, 240x320
RECORD_S_PER_TRIAL_1_WORKER = 80    # ~16 s wall per trial at 5 workers
DATASET_MB_PER_EPISODE = 32         # 287 episodes = 9.2 GB (images in parquet)
UPLOAD_MB_PER_S = 20
CHECKPOINT_MB = 200
EVAL_S_PER_EPISODE = 105            # temporal ensembling, MPS, 2 torch threads, 4-8 evaluations in parallel
PILOT_EPISODES, PILOT_MIN_SUCCESS, PILOT_SEED0 = 16, .8, 9000


class RefitError(SystemExit):
    pass


# ---------------------------------------------------------------- hashing

def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def digest(*parts):
    return sha256_bytes(json.dumps(parts, sort_keys=True, default=str).encode())


def trials(batch):
    return sorted(t for t in Path(batch).glob('trial-*') if (t / 'demo.json').exists()
                  and (t / 'run/scene.xml').exists())


def batch_fingerprint(batch):
    """Cheap fingerprint of a demo batch: name, size and mtime of every trial's demo and scene files."""
    rows = []
    for t in trials(batch):
        for name in ('demo.npz', 'demo.json', 'run/scene.xml'):
            st = (t / name).stat()
            rows.append((t.name, name, st.st_size, st.st_mtime_ns))
    return digest(str(Path(batch).resolve()), rows)


def tool_hash(*relpaths):
    return digest({p: sha256_file(SOFTWARE / p) for p in relpaths})


# ---------------------------------------------------------------- manifest

def _read_json(path, label, errors):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        errors.append(f'{label}: file not found: {path}')
    except (OSError, ValueError) as exc:
        errors.append(f'{label}: cannot read {path}: {exc}')
    return None


def _resolve(manifest_dir, value):
    p = Path(value).expanduser()
    if p.is_absolute():
        return p
    for root in (manifest_dir, SOFTWARE):
        if (root / p).exists():
            return (root / p).resolve()
    return (manifest_dir / p).resolve()


def _check_intrinsics(k, label, errors, warnings):
    keys = ('fx', 'fy', 'cx', 'cy', 'width', 'height')
    missing = [key for key in keys if not isinstance(k.get(key), (int, float))]
    if missing:
        errors.append(f'{label}: intrinsics need numbers {missing}')
        return False
    if not all(math.isfinite(float(k[key])) and float(k[key]) > 0 for key in keys):
        errors.append(f'{label}: intrinsics must be positive finite numbers')
        return False
    if not (0 < k['cx'] < k['width'] and 0 < k['cy'] < k['height']):
        errors.append(f'{label}: principal point outside the image')
        return False
    dist = [abs(float(v)) for v in (k.get('distortion') or [])]
    if dist and max(dist[:2] + dist[4:5]) > .05:
        warnings.append(f'{label}: lens distortion is significant (k = {[round(v, 3) for v in dist[:5]]}); the '
                        'simulation renders a pinhole camera, so robot images must be undistorted before the policy')
    return True


def _lens_entry(base_spec, lens, label, errors, warnings):
    if not isinstance(lens, dict):
        errors.append(f'{label}: lens calibration must be a JSON object')
        return None
    if not _check_intrinsics(lens, label, errors, warnings):
        return None
    rms = lens.get('rms_reprojection_px')
    if rms is not None:
        if float(rms) > 2.:
            errors.append(f'{label}: reprojection error {float(rms):.2f} px is above 2 px; retake the checkerboard photos')
        elif float(rms) > 1.:
            warnings.append(f'{label}: reprojection error {float(rms):.2f} px is above 1 px')
    if lens.get('photos_used') is not None and int(lens['photos_used']) < 10:
        errors.append(f'{label}: only {lens["photos_used"]} calibration photos; take 15-20')
    spec = {key: copy.deepcopy(base_spec[key]) for key in ('frame', 'position_m', 'rotation_cv') if key in base_spec}
    spec['intrinsics'] = {key: lens[key] for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')}
    return spec


def _camera_entry(data, label, errors):
    if not isinstance(data, dict):
        errors.append(f'{label}: camera pose must be a JSON object')
        return None
    entry = data.get('camera_entry', data)
    if data.get('camera') not in (None, 'front'):
        errors.append(f'{label}: file is for camera {data["camera"]!r}, expected the head camera (front)')
    if entry.get('frame', 'arm_base') != 'arm_base':
        errors.append(f'{label}: the head camera pose must be in the arm_base frame')
    keep = ('position_m', 'rotation_cv', 'look_at_m', 'up_hint', 'roll_deg', 'fovy_deg', 'intrinsics',
            'crop_to_fovy_deg', 'sources', 'head_tilt_deg', 'head_pan_deg')
    return {k: copy.deepcopy(entry[k]) for k in keep if k in entry}


def _station_block(data, label, errors):
    if not isinstance(data, dict):
        errors.append(f'{label}: station must be a JSON object')
        return None
    if data.get('schema') == fsm.SCHEMA or 'station' in data:
        if data.get('measured') is False and not data.get('model_derived'):
            errors.append(f'{label}: file says "measured": false')
        data = data.get('station')
        if not isinstance(data, dict):
            errors.append(f'{label}: no station block')
            return None
    return copy.deepcopy(data)


def recorded_station(batch):
    """Station geometry of the first recorded trial of a batch (scene.recorded.xml when restaged in place)."""
    import xml.etree.ElementTree as ET
    ts = trials(batch)
    if not ts:
        return None
    scene = ts[0] / 'run/scene.recorded.xml'
    scene = scene if scene.exists() else ts[0] / 'run/scene.xml'
    return fsm.scene_station(ET.parse(scene).getroot())


@dataclass
class Validation:
    manifest_path: Path
    manifest: dict = field(default_factory=dict)
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    measurement: dict | None = None
    provenance: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)        # label -> resolved file path
    train_batches: list = field(default_factory=list)
    eval_batches: list = field(default_factory=list)
    recorded: dict | None = None
    station_mismatch: dict = field(default_factory=dict)
    crops: dict = field(default_factory=dict)

    @property
    def name(self):
        return self.manifest.get('name')

    @property
    def mode(self):
        return 'rerecord' if self.station_mismatch else 'restage'

    def inputs_hash(self):
        files = {label: sha256_file(p) for label, p in sorted(self.sources.items()) if Path(p).is_file()}
        return digest(self.manifest, files, [batch_fingerprint(b) for b in self.train_batches + self.eval_batches],
                      tool_hash('carton/folding_station_measured.py'))


def validate(manifest_path, *, allow_unmeasured=False, output_root=OUTPUT_ROOT):
    """Read the manifest and every file it names, compose the measurement and check it. Never raises on bad
    input: problems are collected in `errors` (refusals) and `warnings` (reported, not blocking)."""
    manifest_path = Path(manifest_path).resolve()
    v = Validation(manifest_path)
    m = _read_json(manifest_path, 'manifest', v.errors)
    if m is None:
        return v
    v.manifest = m
    if m.get('schema') != MANIFEST_SCHEMA:
        v.errors.append(f'manifest: schema must be {MANIFEST_SCHEMA!r}')
        return v
    unknown = set(m) - {'schema', 'name', 'measured', 'base', 'station', 'head_camera', 'wrist_lenses',
                        'appearance', 'demos', 'note', 'notes'}
    if unknown:
        v.errors.append(f'manifest: unknown keys {sorted(unknown)}')
    if not isinstance(m.get('name'), str) or not NAME_RE.match(m['name']):
        v.errors.append('manifest: name must be 1-40 characters of a-z, 0-9 and -, starting with a letter or digit')
    if m.get('measured') is not True and not allow_unmeasured:
        v.errors.append('manifest: "measured" is not true; pass --allow-unmeasured for a check run on model values')
    here = manifest_path.parent

    base_path = _resolve(here, m.get('base') or DEFAULT_BASE)
    v.sources['base'] = base_path
    base = _read_json(base_path, 'base', v.errors)
    if base is None:
        return v
    try:
        base = fsm.load_measurement(base)
    except ValueError as exc:
        v.errors.append(f'base: {exc}')
        return v
    measurement = {'schema': fsm.SCHEMA, 'measured': m.get('measured') is True, 'model_derived': False,
                   'note': f'Composed by tools/refit_fold_policy.py from {manifest_path.name}.',
                   'station': copy.deepcopy(base.get('station')), 'cameras': copy.deepcopy(base.get('cameras') or {})}
    measurement['cameras'].pop('top', None)   # the overhead camera is not a robot camera
    model_parts = []

    if m.get('station') is not None:
        p = _resolve(here, m['station'])
        v.sources['station'] = p
        data = _read_json(p, 'station', v.errors)
        station = _station_block(data, 'station', v.errors) if data is not None else None
        if station is not None:
            missing = [k for k in fsm._STATION_KEYS if station.get(k) is None]
            if missing:
                v.errors.append(f'station: missing {missing}')
            merged = {**(measurement['station'] or {}), **station}
            merged.pop('sources', None)
            merged['sources'] = f'measured: {p.name}'
            measurement['station'] = merged
        v.provenance['station'] = f'measured ({p.name})'
    else:
        model_parts.append('station')
        v.provenance['station'] = f'model ({base_path.name})'

    if m.get('head_camera') is not None:
        p = _resolve(here, m['head_camera'])
        v.sources['head_camera'] = p
        data = _read_json(p, 'head_camera', v.errors)
        entry = _camera_entry(data, 'head_camera', v.errors) if data is not None else None
        if entry is not None:
            if entry.get('intrinsics') is not None:
                _check_intrinsics(entry['intrinsics'], 'head_camera', v.errors, v.warnings)
            entry['sources'] = f'measured: {p.name}' + (f'; {entry["sources"]}' if entry.get('sources') else '')
            measurement['cameras']['front'] = entry
        v.provenance['front'] = f'measured ({p.name})'
    else:
        model_parts.append('head camera (front)')
        v.provenance['front'] = f'model ({base_path.name})'

    lenses = m.get('wrist_lenses') or {}
    if set(lenses) - {'left', 'right'}:
        v.errors.append('wrist_lenses: keys must be left and/or right')
    for side in ('left', 'right'):
        key = f'{side}_wrist'
        if side not in lenses:
            model_parts.append(f'{side} wrist lens')
            v.provenance[key] = f'model ({base_path.name}; field of view assumed)'
            continue
        p = _resolve(here, lenses[side])
        v.sources[f'{side}_wrist_lens'] = p
        base_spec = (base.get('cameras') or {}).get(key)
        if base_spec is None:
            v.errors.append(f'base: no {key} camera to mount the {side} lens on')
            continue
        lens = _read_json(p, f'{side} wrist lens', v.errors)
        spec = _lens_entry(base_spec, lens, f'{side} wrist lens', v.errors, v.warnings) if lens is not None else None
        if spec is not None:
            spec['sources'] = f'mount: robot model; lens: measured {p.name}'
            measurement['cameras'][key] = spec
        v.provenance[key] = f'lens measured ({p.name}), mount from the robot model'

    if m.get('appearance') is not None:
        app = m['appearance']
        if isinstance(app, str):
            p = _resolve(here, app)
            v.sources['appearance'] = p
            app = _read_json(p, 'appearance', v.errors)
        measurement['appearance'] = app

    if model_parts:
        measurement['model_derived'] = True
        if m.get('measured') is True:
            v.warnings.append('not measured, model values used: ' + ', '.join(model_parts))
    measurement['refit_provenance'] = v.provenance
    try:
        measurement = fsm.load_measurement(measurement)
    except (ValueError, TypeError) as exc:
        v.errors.append(f'measurement: {exc}')
        measurement = None
    v.measurement = measurement

    if measurement is not None:
        for key, spec in measurement['cameras'].items():
            if spec.get('intrinsics') is not None:
                crop = fsm.policy_crop(spec['intrinsics'], spec.get('crop_to_fovy_deg'))
                v.crops[key] = {k: (round(x, 2) if isinstance(x, float) else x) for k, x in crop.items()}
                v.crops[key]['crop_xyxy'] = [round(x, 1) for x in crop['crop_xyxy']]
                if not crop['fits']:
                    v.errors.append(f'{key}: camera is narrower than crop_to_fovy_deg')
                if crop['pixel_aspect_error'] > .02:
                    v.warnings.append(f'{key}: fx and fy differ by {100 * crop["pixel_aspect_error"]:.1f}%')
                w, h = spec['intrinsics']['width'], spec['intrinsics']['height']
                x0, y0, x1, y1 = crop['crop_xyxy']
                if x0 > 1 or y0 > 1 or x1 < w - 1 or y1 < h - 1:
                    v.warnings.append(f'{key}: the policy sees a centred 4:3 crop of the {w}x{h} image, '
                                      f'x {x0:.0f}..{x1:.0f}, y {y0:.0f}..{y1:.0f} (vertical field '
                                      f'{crop["fovy_deg"]:.1f} deg); the robot runtime must crop the same way')
        front = measurement['cameras'].get('front')
        if front is not None and 'head_camera' in v.sources:
            position, right, up, _ = fsm.camera_pose(front, 'front')
            forward = fsm.rotation_cv_from_axes(right, up)[:, 2]
            pitch = math.degrees(math.asin(max(-1., min(1., -forward[2]))))
            if not 20 <= pitch <= 85:
                v.warnings.append(f'head camera pitch {pitch:.0f} deg below horizontal; the model head at 58 deg '
                                  'tilt looks about 32 deg down; check the pose and the tag layout')
            model_front = (base.get('cameras') or {}).get('front')
            if model_front is not None:
                gap = float(((position - fsm.camera_pose(model_front, 'front')[0]) ** 2).sum() ** .5)
                if gap > .05:
                    v.warnings.append(f'head camera is {gap * 100:.0f} cm from the model head camera; check '
                                      'the head tilt and the tag positions')

    demos = m.get('demos') or {}
    root = Path(output_root)
    v.train_batches = [_resolve(here, b) if Path(b).is_absolute() or (here / b).exists() else root / b
                       for b in demos.get('train_batches', DEFAULT_TRAIN_BATCHES)]
    v.eval_batches = [_resolve(here, b) if Path(b).is_absolute() or (here / b).exists() else root / b
                      for b in demos.get('eval_batches', DEFAULT_EVAL_BATCHES)]
    names = [b.name for b in v.train_batches + v.eval_batches]
    if len(set(names)) != len(names):
        v.errors.append(f'demos: batch directory names must be unique ({names})')
    if not v.train_batches:
        v.errors.append('demos: no training batches')
    stations = {}
    for b in v.train_batches + v.eval_batches:
        if not trials(b):
            v.errors.append(f'demos: no recorded trials in {b}')
            continue
        stations[str(b)] = recorded_station(b)
    if stations:
        first = next(iter(stations.values()))
        for b, s in stations.items():
            if fsm.station_mismatch(first, s):
                v.errors.append(f'demos: {b} was recorded at a different station than the other batches')
        v.recorded = first
        if measurement is not None and measurement.get('station') is not None:
            v.station_mismatch = fsm.station_mismatch(first, measurement['station'])
        if v.station_mismatch:
            for b in v.train_batches + v.eval_batches:
                if not (Path(b) / 'batch.json').exists():
                    v.errors.append(f'demos: {b} has no batch.json; re-recording needs its randomisation ranges')
    return v


# ---------------------------------------------------------------- stages

@dataclass
class Cmd:
    argv: list
    env: dict = field(default_factory=dict)
    in_process: bool = False   # shown for the record; the tool does this itself
    cwd: Path | None = None    # default: software/

    def show(self):
        if self.in_process:
            return '(in-process) ' + ' '.join(str(a) for a in self.argv)
        env = ' '.join(f'{k}={shlex.quote(str(v))}' for k, v in self.env.items())
        cd = f'cd {shlex.quote(str(self.cwd))} && ' if self.cwd else ''
        return cd + (env + ' ' if env else '') + shlex.join([str(a) for a in self.argv])


@dataclass
class Stage:
    name: str
    inputs: str
    outputs: list
    commands: list
    minutes: float
    cloud: bool = False
    usd: float = 0.
    note: str = ''
    skip: str | None = None     # reason the stage does not apply
    blocked: str | None = None  # reason it cannot run in this invocation
    run: object = None          # callable(ctx) for in-process stages
    status: str = ''


class Refit:
    def __init__(self, validation, args):
        self.v = validation
        self.args = args
        self.python = str(args.python)
        name = validation.name or 'invalid'
        self.run_dir = Path(args.run_dir or Path(args.output_root) / 'fold-refit' / name).resolve()
        self.state_dir = self.run_dir / 'stages'
        self.hub_user = args.hub_user
        self.stages = []

    # -- paths
    def p(self, *parts):
        return self.run_dir.joinpath(*parts)

    def env(self):
        return {'PYTHONPATH': str(SOFTWARE)}

    def tool(self, name, *argv):
        return Cmd([self.python, str(SOFTWARE / 'tools' / name), *map(str, argv)], self.env())

    # -- state
    def state(self, name):
        path = self.state_dir / f'{name}.json'
        return json.loads(path.read_text()) if path.exists() else None

    def save_state(self, stage, **extra):
        self.state_dir.mkdir(parents=True, exist_ok=True)
        data = {'stage': stage.name, 'inputs': stage.inputs, 'outputs': [str(o) for o in stage.outputs],
                'finished': time.strftime('%Y-%m-%dT%H:%M:%S'), **extra}
        tmp = self.state_dir / f'.{stage.name}.json.tmp'
        tmp.write_text(json.dumps(data, indent=1))
        tmp.replace(self.state_dir / f'{stage.name}.json')

    def is_done(self, stage):
        s = self.state(stage.name)
        return bool(s and s.get('inputs') == stage.inputs and s.get('finished')
                    and all(Path(o).exists() for o in stage.outputs))

    def clear_outputs(self, stage):
        for o in stage.outputs:
            o = Path(os.path.abspath(o))   # not resolve(): a symlink output is removed, never its target
            if self.run_dir not in o.parents:
                raise RefitError(f'Refusing to remove {o}: outside the run directory {self.run_dir}')
            if o.is_symlink() or o.is_file():
                o.unlink()
            elif o.is_dir():
                shutil.rmtree(o)

    # -- plan
    def plan(self):
        v, a = self.v, self.args
        S = []
        m_path = self.p('measurement.json')
        h_validate = v.inputs_hash()
        S.append(Stage('validate', h_validate, [m_path, self.p('validate.json')],
                       [Cmd(['compose and check', v.manifest_path, '->', m_path], in_process=True)], .1,
                       run=self._run_validate,
                       note=f'mode: {v.mode}' + (f'; station differs: {v.station_mismatch}' if v.station_mismatch else '')))

        # record: only when the station cannot be matched by re-rendering
        sources = list(v.train_batches) + list(v.eval_batches)
        h_record = digest(h_validate, tool_hash('tools/record_measured_fold_demos.py', 'tools/record_fold_demos.py'),
                          PILOT_EPISODES, PILOT_MIN_SUCCESS)
        rec_cmds, rec_outputs, rec_min = [], [], 0.
        if v.mode == 'rerecord':
            ref = self._batch_args(v.train_batches[0])
            pilot = self.p('demos-recorded', 'pilot')
            rec_cmds.append(self._record_cmd(ref, pilot, PILOT_EPISODES, PILOT_SEED0))
            rec_min += PILOT_EPISODES * RECORD_S_PER_TRIAL_1_WORKER / a.record_workers / 60
            new_sources = []
            for b in sources:
                ba = self._batch_args(b)
                out = self.p('demos-recorded', b.name)
                rec_cmds.append(self._record_cmd(ba, out, int(ba['episodes']), int(ba['seed0'])))
                rec_outputs.append(out)
                new_sources.append(out)
                rec_min += int(ba['episodes']) * RECORD_S_PER_TRIAL_1_WORKER / a.record_workers / 60
            sources = new_sources
        record = Stage('record', h_record, rec_outputs + ([self.p('demos-recorded', 'pilot')] if rec_outputs else []),
                       rec_cmds, rec_min, run=self._run_record,
                       note=f'pilot of {PILOT_EPISODES} must succeed >= {PILOT_MIN_SUCCESS:.0%} before the full recording')
        if v.mode != 'rerecord':
            record.skip = 'station matches the recorded demonstrations: re-rendering is exact, no re-recording'
        elif not a.allow_rerecord:
            record.blocked = ('the measured station differs from the recorded one (re-rendering cannot match it); '
                              're-run with --allow-rerecord to record new demonstrations')
        S.append(record)

        # restage
        n_trials = sum(len(trials(b)) for b in v.train_batches + v.eval_batches)
        h_restage = digest(h_record if v.mode == 'rerecord' else h_validate, tool_hash('tools/restage_fold_scenes.py'))
        restaged = {b.name: self.p('demos-restaged', b.name) for b in v.train_batches + v.eval_batches}
        rs_cmds = []
        for src in sources:
            argv = ['--measurement', m_path, '--batches', src, '--out', restaged[src.name]]
            if v.measurement is not None and not (v.measurement.get('measured') or v.measurement.get('model_derived')):
                argv.append('--allow-unmeasured')
            rs_cmds.append(self.tool('restage_fold_scenes.py', *argv))
        S.append(Stage('restage', h_restage, list(restaged.values()), rs_cmds,
                       n_trials * RESTAGE_S_PER_TRIAL / 60 + .5,
                       note=f'{n_trials} trials re-rendered with the measured cameras (links demo files, new scene.xml)'))

        # dataset
        train_eps = sum(len(trials(b)) for b in v.train_batches) * .9 * .95
        h_dataset = digest(h_restage, tool_hash('tools/fold_demos_to_lerobot.py', 'farm/learning/recorder.py'),
                           a.height, a.width, a.holdout_every, a.max_episodes, POLICY_CAMERA_ARGS, TASK)
        ds = self.p('dataset')
        argv = ['--batches', *[restaged[b.name] for b in v.train_batches], '--out', ds, '--task', TASK,
                '--cameras', *POLICY_CAMERA_ARGS, '--holdout-every', a.holdout_every, '--workers', a.workers,
                '--height', a.height, '--width', a.width]
        if a.max_episodes:
            argv += ['--max-episodes', a.max_episodes]
            train_eps = min(train_eps, a.max_episodes)
        S.append(Stage('dataset', h_dataset, [ds], [self.tool('fold_demos_to_lerobot.py', *argv)],
                       train_eps * CONVERT_S_PER_EPISODE_8_WORKERS * 8 / max(1, a.workers) / 60 + 1,
                       note=f'about {train_eps:.0f} episodes, cameras front + left_wrist + right_wrist at '
                            f'{a.height}x{a.width}; seed % {a.holdout_every} held out'))

        # holdout
        h_holdout = digest(h_dataset, a.holdout_every)
        hold = self.p('holdout.json')
        S.append(Stage('holdout', h_holdout, [hold],
                       [Cmd(['merge', ds / 'holdout.json', f'+ seed % {a.holdout_every} == 0 of',
                             *[restaged[b.name] for b in v.eval_batches], '->', hold], in_process=True)],
                       sum(len(trials(b)) for b in v.eval_batches) / a.holdout_every * .02 + .2,
                       run=self._run_holdout))

        # cloud: push, train, wait, download
        short = h_dataset[:8]
        self.dataset_repo = f'{self.hub_user}/carton_fold_refit_{v.name}_{short}'
        h_push = digest(h_dataset, self.dataset_repo)
        push_mb = train_eps * DATASET_MB_PER_EPISODE
        S.append(Stage('push', h_push, [self.p('lerobot-home', *self.dataset_repo.split('/'))],
                       [Cmd([self.python, '-c', 'from lerobot.datasets import LeRobotDataset; '
                             f'LeRobotDataset({self.dataset_repo!r}, root={str(ds)!r})'
                             '.push_to_hub(private=True, tags=["carton", "simulation", "act"])'],
                            {'HF_LEROBOT_HOME': str(self.p('lerobot-home'))})],
                       push_mb / UPLOAD_MB_PER_S / 60 + 1, cloud=True, run=self._run_push,
                       note=f'PRIVATE dataset repo, about {push_mb / 1000:.1f} GB upload'))
        h_train = digest(h_push, RECIPE)
        self.model_repo = f'{self.hub_user}/act_carton_fold_refit_{v.name}_{h_train[:8]}'
        train_min = RECIPE['steps'] * TRAIN_S_PER_STEP / 60
        usd = (train_min + JOB_OVERHEAD_MIN) / 60 * A100_LARGE_USD_PER_HOUR
        cap = _hours(RECIPE['timeout']) * A100_LARGE_USD_PER_HOUR
        S.append(Stage('train', h_train, [], [self._train_cmd()], .5,
                       cloud=True, usd=usd, run=self._run_train,
                       note=f'HF Jobs {RECIPE["target"]}: ~{train_min + JOB_OVERHEAD_MIN:.0f} min, ~${usd:.2f}; '
                            f'timeout {RECIPE["timeout"]} caps it at ${cap:.2f}; model repo {self.model_repo} (private)'))
        S.append(Stage('wait', h_train, [],
                       [Cmd([f'poll huggingface_hub.inspect_job every {a.poll_s:.0f} s (by hand: hf jobs inspect '
                             '<job id>; hf jobs logs <job id>)'], in_process=True)],
                       train_min + JOB_OVERHEAD_MIN, cloud=True, run=self._run_wait))
        steps = [int(s) for s in a.eval_steps]
        ckpt_root = Path(a.checkpoints).resolve() if a.checkpoints else self.p('train')
        self.checkpoints = {s: ckpt_root / 'checkpoints' / f'{s:06d}' / 'pretrained_model' for s in steps}
        S.append(Stage('download', digest(h_train, steps), list(self.checkpoints.values()),
                       [Cmd(['hf', 'download', self.model_repo, '--include',
                             *[f'checkpoints/{s:06d}/pretrained_model/*' for s in steps], '--local-dir',
                             self.p('train')], in_process=True),
                        Cmd(['drop "dtype": null from each checkpoint config.json (lerobot 0.6.1)'], in_process=True)],
                       len(steps) * CHECKPOINT_MB / UPLOAD_MB_PER_S / 60 + 1, cloud=True, run=self._run_download))
        if a.checkpoints:
            for st in S[-4:]:
                st.skip = f'--checkpoints {ckpt_root} given: evaluating existing checkpoints'

        # eval
        n_hold = self._holdout_count_estimate()
        shards = max(1, math.ceil(n_hold / a.eval_shard_size))
        self.eval_jobs = self._eval_jobs(steps, shards)
        eval_min = math.ceil(len(self.eval_jobs) / a.eval_parallel) * min(n_hold, a.eval_shard_size) * EVAL_S_PER_EPISODE / 60
        h_eval = digest(h_holdout, h_train if not a.checkpoints else str(ckpt_root), steps, a.eval_shard_size,
                        a.device, tool_hash('tools/eval_fold_policy.py'))
        S.append(Stage('eval', h_eval, [self.p('eval', f'{s:06d}', 'summary.json') for s in steps],
                       [c for *_, c in self.eval_jobs], eval_min, run=self._run_eval,
                       note=f'checkpoints {", ".join(str(s) for s in steps)} x {n_hold} held-out starts in '
                            f'{shards} shards, {a.eval_parallel} at a time'))
        h_pick = digest(h_eval, a.min_success, str(a.pilot))
        S.append(Stage('pick', h_pick, [self.p('report.json'), self.p('report.md'),
                                        self.p('fold-policy-checkpoint.json')],
                       [Cmd(['rank by successes, then carton slide, then flap penetration (never "last");',
                             'write report.json, report.md, fold-policy-checkpoint.json'], in_process=True)], .1,
                       run=self._run_pick,
                       note=f'install gate: success rate >= {a.min_success:.0%}'))
        self.stages = S
        self._statuses()
        return S

    def _statuses(self):
        a = self.args
        for st in self.stages:
            if st.skip:
                st.status = 'skip'
                continue
            if self.v.errors:
                st.status = 'blocked'
                st.blocked = st.blocked or 'validation failed'
                continue
            if self.is_done(st):
                st.status = 'done'
                continue
            prior = self.state(st.name)
            st.status = 'stale' if prior else 'todo'
            if st.blocked:
                st.status = 'blocked'
            elif st.cloud and not a.launch:
                st.status, st.blocked = 'needs --launch', 'cloud stage'
            elif st.name in ('eval', 'pick') and not (a.launch or a.checkpoints):
                st.status, st.blocked = 'needs --launch', 'no checkpoints yet (or give --checkpoints DIR)'
            elif st.name == 'eval' and a.checkpoints:
                missing = [str(p) for p in self.checkpoints.values() if not (p / 'model.safetensors').exists()]
                if missing:
                    st.status, st.blocked = 'blocked', f'checkpoints not found: {missing}'

    def _eval_jobs(self, steps, shards):
        jobs = []
        for s in steps:
            for k in range(shards):
                out = self.p('eval', f'{s:06d}', f'shard-{k:02d}')
                cmd = self.tool('eval_fold_policy.py', '--checkpoint', self.checkpoints[s],
                                '--holdout', self.p('eval', f'holdout-shard-{k:02d}.json'),
                                '--episodes', self.args.eval_shard_size, '--gifs', 0, '--videos', 1 if k == 0 else 0,
                                '--device', self.args.device, '--threads', 2, '--temporal-ensemble', .01, '--out', out)
                jobs.append((s, k, out, cmd))
        return jobs

    def _holdout_count_estimate(self):
        hold = self.p('holdout.json')
        if hold.exists():
            try:
                return len(json.loads(hold.read_text()))
            except ValueError:
                pass
        n = sum(len(trials(b)) for b in self.v.train_batches + self.v.eval_batches)
        return max(1, round(n / self.args.holdout_every * .97))

    def _batch_args(self, batch):
        info = json.loads((Path(batch) / 'batch.json').read_text())['args']
        return info

    def _record_cmd(self, ba, out, episodes, seed0):
        def pair(text):
            return [str(float(x)) for x in json.loads(text)]
        return self.tool('record_measured_fold_demos.py', '--measurement', self.p('measurement.json'), '--',
                         '--simulation-root', ba['simulation_root'], '--out', out, '--episodes', episodes,
                         '--seed0', seed0, '--workers', self.args.record_workers, '--task', ba.get('task', TASK),
                         '--hold-after', ba.get('hold_after', '3.0'), '--max-time', ba.get('max_time', '75.0'),
                         '--offset-x', *pair(ba['offset_x']), '--yaw', *pair(ba['yaw']),
                         '--stiffness', *pair(ba['stiffness']), '--python', self.python)

    def _train_cmd(self):
        r = RECIPE
        lerobot_train = str(Path(self.python).parent / 'lerobot-train')
        return Cmd([lerobot_train, f'--dataset.repo_id={self.dataset_repo}', f'--policy.type={r["policy"]}',
                    '--policy.device=cuda', f'--policy.chunk_size={r["chunk_size"]}',
                    f'--policy.n_action_steps={r["n_action_steps"]}', f'--policy.optimizer_lr={r["optimizer_lr"]}',
                    '--policy.private=true', f'--policy.repo_id={self.model_repo}', f'--batch_size={r["batch_size"]}',
                    f'--steps={r["steps"]}', f'--save_freq={r["save_freq"]}', '--save_checkpoint_to_hub=true',
                    '--log_freq=500', f'--num_workers={r["num_workers"]}', '--env_eval_freq=0', '--wandb.enable=false',
                    f'--job_name={self.model_repo.split("/")[1]}', f'--job.target={r["target"]}',
                    f'--job.timeout={r["timeout"]}', '--job.detach=true'],
                   {'HF_LEROBOT_HOME': str(self.p('lerobot-home')), 'WANDB_MODE': 'disabled'},
                   cwd=self.p('train-cwd'))   # lerobot-train writes outputs/ under its working directory

    # -- execution
    def execute(self):
        for st in self.stages:
            if st.status in ('done', 'skip'):
                print(f'[{st.name}] {st.status}' + (f': {st.skip}' if st.skip else ''), flush=True)
                continue
            if st.status in ('blocked', 'needs --launch'):
                print(f'[{st.name}] stopped: {st.blocked}', flush=True)
                if st.name == 'record':
                    raise RefitError(f'Stopped at record: {st.blocked}')
                if st.cloud or st.name in ('eval', 'pick'):
                    print('Local stages finished. To train on the cloud and evaluate, re-run with --launch '
                          f'(about ${sum(s.usd for s in self.stages):.2f}).', flush=True)
                    return
                raise RefitError(f'Stopped at {st.name}: {st.blocked}')
            print(f'[{st.name}] running' + (' (inputs changed; rebuilding)' if st.status == 'stale' else ''),
                  flush=True)
            if st.name not in ('train', 'wait'):   # never drop a submitted job's record
                self.clear_outputs(st)
            extra = {}
            if st.run is not None:
                extra = st.run(st) or {}
            else:
                for i, cmd in enumerate(st.commands):
                    self.run_cmd(cmd, f'{st.name}-{i}')
            missing = [str(o) for o in st.outputs if not Path(o).exists()]
            if missing:
                raise RefitError(f'[{st.name}] finished without its outputs: {missing}')
            self.save_state(st, **extra)
            print(f'[{st.name}] done', flush=True)

    def run_cmd(self, cmd, label, popen=False):
        logs = self.p('logs')
        logs.mkdir(parents=True, exist_ok=True)
        print('  $ ' + cmd.show(), flush=True)
        log = open(logs / f'{label}.log', 'w')
        env = {**os.environ, **{k: str(x) for k, x in cmd.env.items()}}
        if cmd.cwd:
            Path(cmd.cwd).mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen([str(x) for x in cmd.argv], cwd=str(cmd.cwd or SOFTWARE), env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        if popen:
            return proc, log
        code = proc.wait()
        log.close()
        if code:
            tail = (logs / f'{label}.log').read_text().splitlines()[-15:]
            raise RefitError(f'Command failed ({code}); log {logs / label}.log:\n' + '\n'.join(tail))
        return None

    # -- in-process stages
    def _run_validate(self, st):
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.p('measurement.json').write_text(json.dumps(self.v.measurement, indent=1) + '\n')
        self.p('validate.json').write_text(json.dumps(validation_report(self.v), indent=1) + '\n')

    def _run_record(self, st):
        pilot_cmd, *full = st.commands
        self.run_cmd(pilot_cmd, 'record-pilot')
        summary = json.loads((self.p('demos-recorded', 'pilot') / 'summary.json').read_text())
        rate = summary['successes'] / max(1, summary['episodes'])
        print(f'  pilot: {summary["successes"]}/{summary["episodes"]} succeeded', flush=True)
        if rate < PILOT_MIN_SUCCESS:
            raise RefitError(f'The scripted demonstrator succeeds only {rate:.0%} at the measured station '
                             f'(pilot, need {PILOT_MIN_SUCCESS:.0%}); its stages were tuned at 120 mm base height. '
                             'Match the station to the recorded one instead, or retune the controller.')
        for i, cmd in enumerate(full):
            self.run_cmd(cmd, f'record-{i}')
        return {'pilot_success_rate': rate}

    def _run_holdout(self, st):
        hold = json.loads((self.p('dataset') / 'holdout.json').read_text())
        for b in self.v.eval_batches:
            hold += holdout_entries(self.p('demos-restaged', b.name), self.args.holdout_every, TASK)
        self.p('holdout.json').write_text(json.dumps(hold, indent=1))
        return {'held_out': len(hold)}

    def _hf(self):
        from huggingface_hub import HfApi, get_token
        if not get_token():
            raise RefitError('Not logged in to Hugging Face on this machine: run `hf auth login` (the token stays in '
                             'the Hugging Face credential store; this tool never prints it)')
        api = HfApi()
        me = api.whoami()
        user, orgs = me['name'], {o.get('name') for o in me.get('orgs', [])}
        if self.hub_user != user and self.hub_user not in orgs:
            raise RefitError(f'Logged in as {user}, which cannot write to {self.hub_user}; pass --hub-user')
        return api

    def _run_push(self, st):
        self._hf()
        link = self.p('lerobot-home', *self.dataset_repo.split('/'))
        link.parent.mkdir(parents=True, exist_ok=True)
        if not link.exists():
            link.symlink_to(self.p('dataset'), target_is_directory=True)
        self.run_cmd(st.commands[0], 'push')
        return {'dataset_repo': self.dataset_repo}

    def _run_train(self, st):
        api = self._hf()
        prior = self.state('train')
        if prior and prior.get('inputs') == st.inputs and prior.get('job_id'):
            return prior
        if api.repo_exists(self.model_repo):
            raise RefitError(f'Model repo {self.model_repo} already exists but this run has no job record; '
                             'delete the repo or the run directory before training again')
        proc, log = self.run_cmd(st.commands[0], 'train', popen=True)
        code = proc.wait()
        log.close()
        text = self.p('logs', 'train.log').read_text()
        job = re.search(r'Job submitted: (\S+)', text)
        if code or not job:
            raise RefitError(f'Job submission failed; see {self.p("logs", "train.log")}')
        info = {'job_id': job.group(1), 'model_repo': self.model_repo, 'dataset_repo': self.dataset_repo,
                'recipe': RECIPE, 'submitted': time.strftime('%Y-%m-%dT%H:%M:%S')}
        self.save_state(st, **info)
        print(f'  job {info["job_id"]}: https://huggingface.co/jobs/{self.hub_user}/{info["job_id"]}', flush=True)
        return info

    def _run_wait(self, st):
        from huggingface_hub import inspect_job
        self._hf()
        job_id = self.state('train')['job_id']
        deadline = time.time() + (_hours(RECIPE['timeout']) + 1) * 3600
        last = None
        while time.time() < deadline:
            try:
                info = inspect_job(job_id=job_id)
            except OSError as exc:   # transient network trouble: keep polling
                print(f'  poll failed ({type(exc).__name__}); retrying', flush=True)
                time.sleep(self.args.poll_s)
                continue
            stage = str(getattr(info.status.stage, 'value', info.status.stage))
            if stage != last:
                print(f'  job {job_id}: {stage} ({time.strftime("%H:%M")})', flush=True)
                last = stage
            if stage in ('COMPLETED', 'CANCELED', 'ERROR', 'DELETED'):
                break
            time.sleep(self.args.poll_s)
        else:
            raise RefitError(f'Job {job_id} still running after the timeout; re-run later to resume polling')
        if stage != 'COMPLETED':
            print(f'  job ended {stage}; continuing with whatever checkpoints reached the Hub', flush=True)
        return {'job_id': job_id, 'job_stage': stage}

    def _run_download(self, st):
        from huggingface_hub import snapshot_download
        api = self._hf()
        files = set(api.list_repo_files(self.model_repo))
        got = []
        for step, path in self.checkpoints.items():
            rel = f'checkpoints/{step:06d}/pretrained_model'
            if f'{rel}/model.safetensors' not in files:
                print(f'  checkpoint {step} is not on the Hub', flush=True)
                continue
            snapshot_download(self.model_repo, allow_patterns=[f'{rel}/*'], local_dir=str(self.p('train')))
            fix_dtype(path)
            got.append(step)
        if not got:
            raise RefitError(f'No evaluated checkpoint ({sorted(self.checkpoints)}) is on {self.model_repo}')
        missing = sorted(set(self.checkpoints) - set(got))
        if missing:   # evaluate what exists rather than fail the whole refit
            for s in missing:
                self.checkpoints.pop(s)
            st.outputs = [self.checkpoints[s] for s in got]
        return {'downloaded': got, 'missing': missing}

    def _run_eval(self, st):
        hold = json.loads(self.p('holdout.json').read_text())
        size = self.args.eval_shard_size
        shards = [hold[i:i + size] for i in range(0, len(hold), size)]
        self.p('eval').mkdir(parents=True, exist_ok=True)
        for k, shard in enumerate(shards):
            path = self.p('eval', f'holdout-shard-{k:02d}.json')
            path.write_text(json.dumps(shard, indent=1))
        for s, path in list(self.checkpoints.items()):
            if not (path / 'model.safetensors').exists():   # e.g. a job that ended before saving it
                print(f'  checkpoint {s} is missing ({path}); not evaluated', flush=True)
                self.checkpoints.pop(s)
        if not self.checkpoints:
            raise RefitError('No checkpoint to evaluate')
        for path in self.checkpoints.values():
            fix_dtype(path)
        jobs = self._eval_jobs(sorted(self.checkpoints), len(shards))
        pending = []
        for s, k, out, cmd in jobs:
            key = digest(st.inputs, s, k, shards[k], _model_stamp(self.checkpoints[s]))
            stamp = out.parent / f'shard-{k:02d}.inputs'
            if (out / 'result.json').exists() and stamp.exists() and stamp.read_text() == key:
                continue
            if out.exists():
                shutil.rmtree(out)
            pending.append((s, k, out, cmd, stamp, key))
        running = []
        while pending or running:
            while pending and len(running) < self.args.eval_parallel:
                s, k, out, cmd, stamp, key = pending.pop(0)
                out.parent.mkdir(parents=True, exist_ok=True)
                proc, log = self.run_cmd(cmd, f'eval-{s:06d}-{k:02d}', popen=True)
                running.append((proc, log, s, k, out, stamp, key))
            time.sleep(2)
            for item in list(running):
                proc, log, s, k, out, stamp, key = item
                if proc.poll() is None:
                    continue
                log.close()
                running.remove(item)
                if proc.returncode or not (out / 'result.json').exists():
                    for other in running:
                        other[0].terminate()
                    raise RefitError(f'Evaluation of {s} shard {k} failed; see {self.p("logs")}')
                stamp.write_text(key)
                print(f'  evaluated checkpoint {s} shard {k}', flush=True)
        st.outputs = [self.p('eval', f'{s:06d}', 'summary.json') for s in self.checkpoints]
        for s in self.checkpoints:
            rows = []
            for k in range(len(shards)):
                rows += json.loads((self.p('eval', f'{s:06d}', f'shard-{k:02d}') / 'result.json').read_text())['episodes']
            self.p('eval', f'{s:06d}', 'summary.json').write_text(json.dumps(score(s, rows), indent=1))
        return {'steps': sorted(self.checkpoints)}

    def _run_pick(self, st):
        scores = [json.loads(p.read_text()) for p in sorted(self.p('eval').glob('[0-9]*/summary.json'))]
        scores = [s for s in scores if s['step'] in self.checkpoints]
        chosen = pick(scores)
        passed = chosen is not None and chosen['success_rate'] >= self.args.min_success
        ckpt = self.checkpoints[chosen['step']] if chosen else None
        report = build_report(self, scores, chosen, passed, ckpt)
        self.p('report.json').write_text(json.dumps(report, indent=1) + '\n')
        self.p('report.md').write_text(report_markdown(report))
        snippet = {'checkpoint': str(ckpt) if passed else None,
                   'model_sha256': sha256_file(ckpt / 'model.safetensors') if passed else None,
                   'install_gate_passed': passed, 'report': str(self.p('report.md'))}
        self.p('fold-policy-checkpoint.json').write_text(json.dumps(snippet, indent=1) + '\n')
        if passed and self.args.pilot:
            install_checkpoint(Path(self.args.pilot), ckpt)
        print(report_markdown(report), flush=True)
        return {'chosen_step': chosen and chosen['step'], 'install_gate_passed': passed}


def _hours(text):
    m = re.fullmatch(r'(\d+(?:\.\d+)?)([smhd])', text)
    return float(m.group(1)) * {'s': 1 / 3600, 'm': 1 / 60, 'h': 1, 'd': 24}[m.group(2)]


def _model_stamp(pretrained):
    st = (Path(pretrained) / 'model.safetensors').stat()
    return [st.st_size, st.st_mtime_ns]


# ---------------------------------------------------------------- helpers used by stages

def holdout_entries(batch, every, task=TASK):
    """Held-out starts of a batch, chosen exactly as tools/fold_demos_to_lerobot.py chooses them."""
    import mujoco
    import numpy as np
    sys.path.insert(0, str(SOFTWARE / 'tools'))
    from fold_demos_to_lerobot import task_end
    out = []
    for t in trials(batch):
        demo = json.loads((t / 'demo.json').read_text())
        if demo['seed'] % every or (task == 'both-shorts' and not demo.get('success')):
            continue
        model = mujoco.MjModel.from_xml_path(str(t / 'run/scene.xml'))
        z = np.load(t / 'demo.npz')
        end = task_end(z['qpos'], z['time'], model, task)
        if end is None:
            continue
        out.append({'trial': str(t), 'seed': demo['seed'], 'end_index': end,
                    'carton_offset_x': demo['carton_offset_x'], 'carton_yaw_degrees': demo['carton_yaw_degrees'],
                    'hinge_stiffness': demo['hinge_stiffness']})
    return out


def fix_dtype(pretrained):
    """Cloud checkpoints carry `"dtype": null`, which lerobot 0.6.1 rejects; drop it. Returns True if changed."""
    cfg = Path(pretrained) / 'config.json'
    if not cfg.exists():
        return False
    data = json.loads(cfg.read_text())
    if 'dtype' in data and data['dtype'] is None:
        del data['dtype']
        cfg.write_text(json.dumps(data, indent=4))
        return True
    return False


def score(step, rows):
    carton = [r['max_carton_translation_mm'] for r in rows]
    return {'step': int(step), 'episodes': len(rows), 'success': sum(bool(r['success']) for r in rows),
            'success_rate': sum(bool(r['success']) for r in rows) / max(1, len(rows)),
            'carton_median_mm': statistics.median(carton) if carton else None,
            'carton_max_mm': max(carton) if carton else None,
            'flap_pen_max_mm': max((r['max_robot_flap_penetration_mm'] for r in rows), default=None),
            'other_pen_max_mm': max((r['max_robot_other_penetration_mm'] for r in rows), default=None),
            'left_unfolded': sum(r['flaps_degrees']['short_left_hinge'] < 80 for r in rows),
            'right_unfolded': sum(r['flaps_degrees']['short_right_hinge'] < 80 for r in rows),
            'failed_seeds': [r['seed'] for r in rows if not r['success']]}


def pick(scores):
    """Best checkpoint by score: most successes, then least carton slide (max, then median), then least flap
    penetration, then the earlier step. Training checkpoints swing widely, so "last" is never assumed."""
    scores = [s for s in scores if isinstance(s.get('step'), int) and s['episodes']]
    if not scores:
        return None
    return min(scores, key=lambda s: (-s['success'], s['carton_max_mm'], s['carton_median_mm'],
                                      s['flap_pen_max_mm'], s['step']))


def install_checkpoint(pilot, ckpt):
    """Set `checkpoint` in <pilot>/.private/fold-policy.json (backup kept). The chat then requires a new dry run:
    it vouches for the model's sha256, which changes."""
    config = pilot / '.private/fold-policy.json'
    if not config.exists():
        raise RefitError(f'{config} does not exist; install the chat tools first (tools/install_fold_policy_chat.py)')
    data = json.loads(config.read_text())
    backups = pilot / '.private/fold-policy-backups'
    backups.mkdir(parents=True, exist_ok=True)
    (backups / f'fold-policy.{time.strftime("%Y%m%d-%H%M%S")}.json').write_text(config.read_text())
    data['checkpoint'] = str(Path(ckpt).resolve())
    config.write_text(json.dumps(data, indent=1) + '\n')
    print(f'  {config}: checkpoint set; a new dry run is required before any fold run', flush=True)


def validation_report(v):
    return {'manifest': str(v.manifest_path), 'name': v.name, 'errors': v.errors, 'warnings': v.warnings,
            'provenance': v.provenance, 'sources': {k: str(p) for k, p in v.sources.items()},
            'mode': v.mode, 'recorded_station': v.recorded, 'station_mismatch': v.station_mismatch,
            'policy_crops': v.crops, 'train_batches': [str(b) for b in v.train_batches],
            'eval_batches': [str(b) for b in v.eval_batches], 'simulation_only': True}


HUMAN_ITEMS = (
    'Approve the cloud spend: re-run with --launch (uploads the dataset privately, trains on an A100).',
    'Set the head to the tilt/pan it had when the head camera was measured before any robot run.',
    'Run a new chat dry run after the checkpoint changes (the chat vouches for the model sha256).',
    'Everything here is simulation: the first robot run still follows the setup slides (one short flap, STOP in hand).',
)


def build_report(refit, scores, chosen, passed, ckpt):
    v = refit.v
    train = refit.state('train') or {}
    human = list(HUMAN_ITEMS[1:])
    if not passed:
        human.insert(0, f'No checkpoint reached {refit.args.min_success:.0%} success on the held-out starts: do not '
                        'install; look at report.md and the eval videos.')
    if any('crop' in w for w in v.warnings):
        human.append('Make the robot runtime feed the same centred 4:3 crop the policy was rendered with (validate.json '
                     'policy_crops); carton/fold_policy_runner.py does not crop today.')
    if any('distortion' in w for w in v.warnings):
        human.append('Undistort the wrist images on the robot before the policy (the simulation is a pinhole camera).')
    return {'schema': 'xlerobot-fold-refit-report/1', 'name': v.name, 'run_dir': str(refit.run_dir),
            'finished': time.strftime('%Y-%m-%dT%H:%M:%S'), 'simulation_only': True,
            'measurements': {'provenance': v.provenance, 'warnings': v.warnings, 'mode': v.mode,
                             'station_mismatch': v.station_mismatch, 'policy_crops': v.crops},
            'dataset': _conversion_summary(refit.p('dataset')), 'dataset_repo': train.get('dataset_repo'),
            'training': {k: train.get(k) for k in ('job_id', 'model_repo', 'recipe', 'submitted')} if train else
            {'checkpoints': str(refit.args.checkpoints)},
            'scores': sorted(scores, key=lambda s: s['step']), 'chosen': chosen,
            'install_gate': {'min_success': refit.args.min_success, 'passed': passed},
            'checkpoint': str(ckpt) if passed else None,
            'chat_config': {'file': '<pilot>/.private/fold-policy.json',
                            'checkpoint': str(ckpt) if passed else None},
            'needs_a_human': human}


def _conversion_summary(ds):
    path = Path(ds) / 'conversion.json'
    if not path.exists():
        return None
    c = json.loads(path.read_text())
    return {'episodes': len(c['episodes']), 'frames': sum(e['frames'] for e in c['episodes']),
            'skipped': len(c['skipped']), 'cameras': c['cameras']}


def report_markdown(r):
    lines = [f'# Fold policy refit: {r["name"]}', '', f'Run directory: `{r["run_dir"]}` (simulation only).', '',
             '## Measurements', '']
    lines += [f'- {k}: {v}' for k, v in r['measurements']['provenance'].items()]
    lines += [f'- mode: {r["measurements"]["mode"]}']
    lines += [f'- warning: {w}' for w in r['measurements']['warnings']]
    if r.get('dataset'):
        d = r['dataset']
        lines += ['', f'Dataset: {d["episodes"]} episodes, {d["frames"]} frames, cameras {", ".join(d["cameras"])}'
                  + (f'; Hub `{r["dataset_repo"]}`' if r.get('dataset_repo') else '')]
    t = r['training']
    if t.get('job_id'):
        lines += [f'Training: job `{t["job_id"]}`, model `{t["model_repo"]}`.']
    lines += ['', '## Held-out closed-loop scores (temporal ensembling 0.01)', '',
              '| checkpoint | success | carton slide median / max (mm) | flap penetration max (mm) | other contact max (mm) |',
              '|---|---|---|---|---|']
    for s in r['scores']:
        mark = '**' if r['chosen'] and s['step'] == r['chosen']['step'] else ''
        lines.append(f'| {mark}{s["step"]}{mark} | {s["success"]}/{s["episodes"]} | {s["carton_median_mm"]:.1f} / '
                     f'{s["carton_max_mm"]:.1f} | {s["flap_pen_max_mm"]:.2f} | {s["other_pen_max_mm"]:.2f} |')
    gate = r['install_gate']
    lines += ['', (f'Chosen: checkpoint {r["chosen"]["step"]} ' if r['chosen'] else 'No checkpoint scored. ')
              + (f'passes the {gate["min_success"]:.0%} gate. Chat config `checkpoint`: `{r["checkpoint"]}`'
                 if gate['passed'] else f'does NOT pass the {gate["min_success"]:.0%} gate; do not install.'),
              '', '## Needs a human', '']
    lines += [f'- {h}' for h in r['needs_a_human']]
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------- output

def print_plan(refit, file=None):
    v, a = refit.v, refit.args
    mode = 'LAUNCH (cloud stages enabled)' if a.launch else 'RUN LOCAL STAGES' if a.run_local else 'DRY RUN (nothing is run or written)'
    out = [f'Fold policy refit: {v.name}   [{mode}]', f'manifest: {v.manifest_path}', f'run dir:  {refit.run_dir}', '']
    for label, text in v.provenance.items():
        out.append(f'  {label:12s} {text}')
    for e in v.errors:
        out.append(f'  ERROR    {e}')
    for w in v.warnings:
        out.append(f'  warning  {w}')
    out.append(f'  mode     {v.mode}' + (f' (station differs from the recorded demos: {v.station_mismatch})'
                                         if v.station_mismatch else ' (station matches: re-render only)'))
    out.append('')
    total_min = total_usd = 0.
    for st in refit.stages:
        flag = ' [cloud]' if st.cloud else ''
        est = (f'~{st.minutes:.0f} min' if st.minutes >= 1 else '<1 min') + (f', ~${st.usd:.2f}' if st.usd else '')
        out.append(f'{st.name}{flag}: {st.status}   ({est})')
        if st.note:
            out.append(f'    {st.note}')
        if st.skip:
            out.append(f'    skipped: {st.skip}')
        elif st.blocked and st.status != 'done':
            out.append(f'    {st.blocked}')
        if st.status != 'skip':
            for c in st.commands:
                out.append(f'    $ {c.show()}')
        if st.status not in ('done', 'skip'):
            total_min += st.minutes
            total_usd += st.usd
    out += ['', f'Remaining: about {total_min / 60:.1f} h wall time, about ${total_usd:.2f} of cloud GPU '
                f'(cap ${_hours(RECIPE["timeout"]) * A100_LARGE_USD_PER_HOUR:.2f} by the job timeout).']
    if not a.launch:
        out.append('Nothing was uploaded and no job was launched. Cloud stages run only with --launch.')
    print("\n".join(out), file=file or sys.stdout, flush=True)


def plan_json(refit):
    return {'name': refit.v.name, 'run_dir': str(refit.run_dir), 'validation': validation_report(refit.v),
            'stages': [{'name': s.name, 'status': s.status, 'cloud': s.cloud, 'minutes': round(s.minutes, 1),
                        'usd': round(s.usd, 2), 'inputs': s.inputs, 'skip': s.skip, 'blocked': s.blocked,
                        'note': s.note, 'outputs': [str(o) for o in s.outputs],
                        'commands': [c.show() for c in s.commands]} for s in refit.stages]}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('manifest', type=Path)
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--run-local', action='store_true', help='run the local stages (no network)')
    g.add_argument('--launch', action='store_true', help='run everything, including the cloud upload and training')
    ap.add_argument('--allow-rerecord', action='store_true',
                    help='record new demonstrations when the station differs (pilot-gated)')
    ap.add_argument('--allow-unmeasured', action='store_true', help='accept a manifest with "measured" not true')
    ap.add_argument('--checkpoints', type=Path,
                    help='evaluate existing checkpoints (DIR/checkpoints/<step>/pretrained_model) instead of training')
    ap.add_argument('--output-root', type=Path, default=OUTPUT_ROOT)
    ap.add_argument('--run-dir', type=Path, help='default: <output-root>/fold-refit/<name>')
    ap.add_argument('--hub-user', default=DEFAULT_HUB_USER)
    ap.add_argument('--python', type=Path, default=Path(sys.executable))
    ap.add_argument('--workers', type=int, default=8, help='dataset rendering processes')
    ap.add_argument('--record-workers', type=int, default=5)
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--holdout-every', type=int, default=10)
    ap.add_argument('--max-episodes', type=int)
    ap.add_argument('--eval-steps', type=int, nargs='+', default=list(DEFAULT_EVAL_STEPS))
    ap.add_argument('--eval-shard-size', type=int, default=8)
    ap.add_argument('--eval-parallel', type=int, default=4)
    ap.add_argument('--device', default='mps')
    ap.add_argument('--min-success', type=float, default=.95, help='install gate on the held-out success rate')
    ap.add_argument('--pilot', type=Path, help='also set checkpoint in <pilot>/.private/fold-policy.json if the gate passes')
    ap.add_argument('--poll-s', type=float, default=60.)
    ap.add_argument('--json', action='store_true', help='print the plan as JSON')
    args = ap.parse_args(argv)
    if any(s % RECIPE['save_freq'] or not 0 < s <= RECIPE['steps'] for s in args.eval_steps):
        ap.error(f'--eval-steps must be multiples of {RECIPE["save_freq"]} up to {RECIPE["steps"]} (saved checkpoints)')
    return args


def main(argv=None):
    args = parse_args(argv)
    v = validate(args.manifest, allow_unmeasured=args.allow_unmeasured, output_root=args.output_root)
    refit = Refit(v, args)
    refit.plan()
    if args.json:
        print(json.dumps(plan_json(refit), indent=1))
    else:
        print_plan(refit)
    if v.errors:
        raise RefitError(2)
    if args.run_local or args.launch:
        refit.execute()
    return refit


if __name__ == '__main__':
    main()
