"""Render what the fold policy sees for one recorded demonstration (default `top` = scene camera `overhead`
and `front`; `--cameras` for others, e.g. the wrist cameras) and report each camera's intrinsics/extrinsics
in the arm-base frame. Simulation only.

States: the first sample, the task end (both shorts folded and held 3 s, as tools/fold_demos_to_lerobot.py
cuts episodes) and the sample halfway between. With `--measurement`, the scene is first restaged with
measured cameras/appearance (carton/folding_station_measured.py). `--real KEY=IMAGE` adds a column with a
real image, centre-cropped to the policy's 4:3 and resized, for side-by-side comparison; keep such sheets
out of the repository (camera imagery stays local).

    PYTHONPATH=. python tools/render_fold_policy_views.py \
        --trial .../fold-demos/batch-01/trial-001 --out .../fold-station-gap/sim-nominal
"""
from __future__ import annotations

import argparse
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from carton.folding_station_measured import (camera_report, restage_scene_xml,
                                              scene_station)

FOLDED, HOLD = 80., 3.


def task_end(model, qpos, time):
    adr = [model.jnt_qposadr[model.joint(n).id] for n in ('short_left_hinge', 'short_right_hinge')]
    since = None
    for k, q in enumerate(qpos):
        if all(math.degrees(q[a]) >= FOLDED for a in adr):
            since = time[k] if since is None else since
            if time[k] - since >= HOLD:
                return k
        else:
            since = None
    return len(qpos) - 1


def size(text):
    w, h = (int(v) for v in text.lower().split('x'))
    return w, h


def crop_to_policy(image, width, height):
    im = image.convert('RGB')
    aspect = width / height
    w, h = im.size
    if w / h > aspect:
        nw = round(h * aspect)
        im = im.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
    else:
        nh = round(w / aspect)
        im = im.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))
    return im.resize((width, height), Image.LANCZOS)


def sheet(cells, labels, rows, cols, cw, ch, title):
    pad, top = 4, 22
    out = Image.new('RGB', (cols * (cw + pad) + pad, rows * (ch + top + pad) + top + pad), 'white')
    d = ImageDraw.Draw(out)
    d.text((pad, 4), title, fill='black')
    for i, (cell, label) in enumerate(zip(cells, labels)):
        r, c = divmod(i, cols)
        x, y = pad + c * (cw + pad), top + pad + r * (ch + top + pad)
        d.text((x, y), label, fill='black')
        if cell is not None:
            out.paste(cell.resize((cw, ch)), (x, y + top - 6))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trial', type=Path, required=True, help='trial dir with run/scene.xml and demo.npz')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--measurement', type=Path, help='measurement JSON to restage cameras/appearance first')
    ap.add_argument('--sizes', type=size, nargs='+', default=[(320, 240), (960, 720)])
    ap.add_argument('--real', nargs='*', default=[], help='KEY=IMAGE pairs (KEY top or front) for the sheet')
    ap.add_argument('--title', default='')
    ap.add_argument('--cameras', nargs='+', default=['top=overhead', 'front=front'],
                    help='policy KEY=SCENE_CAMERA pairs, e.g. front=front left_wrist=left_wrist right_wrist=right_wrist')
    args = ap.parse_args(argv)
    cameras = dict(item.split('=', 1) for item in args.cameras)
    args.out.mkdir(parents=True, exist_ok=True)
    scene = args.trial / 'run/scene.xml'
    report = {'trial': str(args.trial), 'scene': str(scene), 'simulation_only': True}
    if args.measurement:
        tmp = Path(tempfile.mkdtemp(prefix='restaged-'))
        report['restage'] = restage_scene_xml(scene, tmp / 'scene.xml', args.measurement)
        scene = tmp / 'scene.xml'
    station = scene_station(ET.parse(scene).getroot())
    origin = np.asarray(station['arm_base_origin_world_m'])
    report['station'] = station
    report['frame'] = ('arm_base: origin midway between the SO101 base_link origins; +x robot right, '
                       '+y toward the table, +z up; metres. Tabletop z = -base_height_above_table_m.')
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, *(w for w, _ in args.sizes))
    model.vis.global_.offheight = max(model.vis.global_.offheight, *(h for _, h in args.sizes))
    data = mujoco.MjData(model)
    z = np.load(args.trial / 'demo.npz')
    end = task_end(model, z['qpos'], z['time'])
    states = {'start': 0, 'mid': end // 2, 'end': end}
    report['states'] = {k: {'index': v, 'time_s': float(z['time'][v])} for k, v in states.items()}
    report['cameras'] = {}
    images = {}
    for w, h in args.sizes:
        renderer = mujoco.Renderer(model, h, w)
        try:  # an open renderer finalized at interpreter shutdown segfaults
            for state, k in states.items():
                data.qpos[:] = z['qpos'][k]
                mujoco.mj_forward(model, data)
                for key, cam in cameras.items():
                    renderer.update_scene(data, camera=cam)
                    im = Image.fromarray(renderer.render())
                    im.save(args.out / f'{key}-{state}-{w}x{h}.png')
                    images[(key, state, (w, h))] = im
                    if state == 'start':
                        report['cameras'].setdefault(key, {})[f'{w}x{h}'] = camera_report(model, data, cam, origin,
                                                                                          h, w)
        finally:
            renderer.close()
    real = dict(item.split('=', 1) for item in args.real)
    w0, h0 = args.sizes[0]
    cells, labels = [], []
    for key in cameras:
        for state in states:
            cells.append(images[(key, state, (w0, h0))])
            labels.append(f'sim {key} ({cameras[key]}) {state}')
        if real:
            cells.append(crop_to_policy(Image.open(real[key]), w0, h0) if key in real else None)
            labels.append(f'real {key}: {Path(real[key]).name}' if key in real else f'real {key}: no camera')
    cols = len(states) + (1 if real else 0)
    sheet(cells, labels, len(cameras), cols, w0, h0,
          args.title or f'{args.trial.name} policy views {w0}x{h0}').save(args.out / 'sheet.png')
    (args.out / 'cameras.json').write_text(json.dumps(report, indent=1))
    for key, rep in report['cameras'].items():
        r = rep[f'{w0}x{h0}']
        print(f"{key}: pos(arm_base)={r['position_arm_base_m']} fovy={r['fovy_deg']:.1f} "
              f"fovx={r['fovx_deg']:.1f} pitch={r['pitch_below_horizontal_deg']:.1f} "
              f"height_above_table={r['height_above_table_m']:.3f}")


if __name__ == '__main__':
    main()
