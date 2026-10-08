"""Render a recorded simulation run to a GIF at a given speed-up (recorded states only).

Usage: PYTHONPATH=. python tools/render_run_gif.py RUN_DIR OUT.gif SPEEDUP FPS
"""
import json, sys, bisect
import mujoco, numpy as np
from PIL import Image, ImageDraw, ImageFont
run, out, speed, fps = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
m = mujoco.MjModel.from_xml_path(run + '/scene.xml'); d = mujoco.MjData(m)
states = json.load(open(run + '/folding-frames.json')); times = [s['time'] for s in states]
ix = {n: m.joint(n + '_hinge').qposadr[0] for n in ('short_left', 'short_right', 'long_near', 'long_far')}
W, H = 480, 300
r = mujoco.Renderer(m, H, W)
try: font = ImageFont.truetype('/System/Library/Fonts/Menlo.ttc', 13)
except Exception: font = ImageFont.load_default()
frames = []
n = int(times[-1] / speed * fps) + 1
for k in range(n):
    t = min(times[-1], k * speed / fps)
    s = states[max(0, bisect.bisect_right(times, t) - 1)]
    d.qpos[:] = s['qpos']; mujoco.mj_forward(m, d)
    tiles = []
    for cam in ('station', 'overhead'):
        r.update_scene(d, camera=cam); tiles.append(r.render())
    img = Image.fromarray(np.concatenate(tiles, 1))
    canvas = Image.new('RGB', (img.width, img.height + 38), 'white'); canvas.paste(img, (0, 38))
    dr = ImageDraw.Draw(canvas)
    a = {k: np.degrees(s['qpos'][i]) for k, i in ix.items()}
    dr.text((6, 2), f"t={t:6.1f}s (x{speed:g})  {s['label'][:70]}", fill='black', font=font)
    dr.text((6, 19), f"shorts L {a['short_left']:5.1f}  R {a['short_right']:5.1f}   near {a['long_near']:5.1f}   far {a['long_far']:5.1f}  (deg; 90 = closed)", fill='black', font=font)
    frames.append(canvas.convert('P', palette=Image.ADAPTIVE, colors=128))
frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 / fps), loop=0, optimize=True)
print(len(frames), 'frames', round(n / fps, 1), 's')
