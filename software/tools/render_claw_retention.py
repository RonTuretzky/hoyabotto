"""Present one complete recorded claw attempt without inventing later motion."""
import argparse
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import textwrap
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from tools.render_folding_comparison import font


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    states_path = args.run / 'folding-frames.json'
    states = json.loads(states_path.read_text())
    times = [s['time'] for s in states]
    if len(states) < 10 or times != sorted(times) or times[0] > .001:
        raise ValueError('A complete video-recorded attempt starting at t=0 is required')
    result = json.loads((args.run / 'result.json').read_text())
    if abs(times[-1] - result['time']) > .002:
        raise ValueError('Recorded states omit the final physics state')
    # Visual sky only; source collision geometry and recorded state are intact.
    xml = ET.parse(args.run / 'scene.xml')
    ET.SubElement(xml.getroot().find('asset'), 'texture', type='skybox', builtin='gradient',
                  rgb1='.66 .73 .81', rgb2='.94 .95 .96', width='256', height='1536')
    presentation = args.out / 'presentation.xml'
    xml.write(presentation, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(presentation))
    data = mujoco.MjData(model)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    whole = mujoco.MjvCamera()
    whole.lookat[:] = [0, .22, -.15]
    whole.distance = 2.3
    whole.azimuth = 130
    whole.elevation = -25
    renderer = mujoco.Renderer(model, 360, 640)
    title_font, body_font, small_font = font(23), font(18), font(16)
    timeline = list(np.arange(0, times[-1], .2)) + [times[-1]]
    frames = []
    try:
        for t in timeline:
            state = states[max(0, bisect_right(times, t + 1e-7) - 1)]
            data.qpos[:] = state['qpos']
            mujoco.mj_forward(model, data)
            frame = Image.new('RGB', (960, 670), '#f1f2f3')
            draw = ImageDraw.Draw(frame)
            draw.text((14, 10), 'BARE CLAWS | OFFLINE PHYSICS | COMPLETE RECORDED ATTEMPT',
                      font=title_font, fill='#111')
            draw.text((14, 43), 'PARTIAL: both long flaps and hands-off closure are unfinished.',
                      font=body_font, fill='#111')
            renderer.update_scene(data, camera=whole, scene_option=option)
            frame.paste(Image.fromarray(renderer.render().copy()), (0, 76))
            renderer.update_scene(data, camera='front', scene_option=option)
            frame.paste(Image.fromarray(renderer.render().copy()).resize((480, 270)), (0, 386))
            draw.text((653, 92), f'Simulation: {t:.2f} / {times[-1]:.2f} s', font=body_font, fill='#111')
            draw.text((653, 125), 'Playback: 1x', font=body_font, fill='#111')
            for index, (flap, label) in enumerate((('short_left', 'Left short'), ('short_right', 'Right short'),
                                                  ('long_far', 'Far long'), ('long_near', 'Near long'))):
                angle = np.degrees(data.qpos[model.joint(flap + '_hinge').qposadr[0]])
                draw.text((653, 169 + 30*index), f'{label}: {angle:.1f} degrees', font=body_font, fill='#111')
            draw.text((653, 302), '0 = upright; 90 = folded', font=small_font, fill='#111')
            ended = t >= times[-1] - 1e-6
            verified_support = bool((result.get('open_claw_transfer') or {}).get(
                'both_shorts_retained_by_right_claw'))
            conclusion = ('Both shorts supported by one claw; other hand withdrawn.' if verified_support
                          else 'Partial sequence finished; full closure incomplete.')
            message = (result.get('error') or conclusion) if ended else state['label']
            draw.text((500, 405), 'STOPPED / PARTIAL' if ended else 'CURRENT ACTION', font=body_font, fill='#111')
            for i, line in enumerate(textwrap.wrap(message, width=46)):
                draw.text((500, 439 + i*22), line, font=small_font, fill='#111')
            draw.text((500, 559), 'Free empty carton; resisting spring hinges.', font=small_font, fill='#111')
            draw.text((500, 585), 'Material and station values remain unmeasured.', font=small_font, fill='#111')
            draw.text((500, 611), 'Final display pause: 2.5 s (not physics time).', font=small_font, fill='#111')
            draw.text((500, 637), 'No robot hardware accessed. No tape applied.', font=small_font, fill='#111')
            frames.append(frame)
    finally:
        renderer.close()
    durations = [max(10, round((b-a)*1000/10)*10) for a, b in zip(timeline, timeline[1:])] + [2500]
    target = args.out / 'bare-claws-complete-attempt.gif'
    frames[0].save(target, save_all=True, append_images=frames[1:], duration=durations, loop=0, optimize=True)
    for label, i in [('start', 0), ('middle', len(frames)//2), ('end', -1)]:
        frames[i].save(args.out / (label + '.png'))
    with Image.open(target) as check:
        total = 0
        for i in range(check.n_frames):
            check.seek(i)
            check.load()
            total += check.info.get('duration', 0)
        metadata = dict(frames=check.n_frames, size=check.size, playback_seconds=total/1000,
                        simulation_end_seconds=times[-1], display_pause_seconds=2.5,
                        complete_recorded_timeline=True, full_task_complete=False,
                        source_run=str(args.run.resolve()), source_frames=len(states),
                        max_source_frame_gap_seconds=max(np.diff(times)),
                        states_sha256=hashlib.sha256(states_path.read_bytes()).hexdigest(),
                        gif_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    target.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    main()
