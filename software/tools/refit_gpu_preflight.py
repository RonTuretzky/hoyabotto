"""Fail fast unless a selected EGL device really provides NVIDIA rendering."""
import argparse
import json
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    import os
    from pathlib import Path
    import mujoco
    from OpenGL import GL
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <light pos="0 0 3"/><camera name="front" pos="0 0 2"/>
      <geom type="box" size=".3 .3 .1" rgba=".2 .4 .8 1"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 240, 320) as renderer:
        renderer.update_scene(data, camera='front')
        renderer.render()
        vendor = GL.glGetString(GL.GL_VENDOR).decode()
        device = GL.glGetString(GL.GL_RENDERER).decode()
        report = dict(egl_device=os.environ.get('MUJOCO_EGL_DEVICE_ID'), vendor=vendor, renderer=device)
        if 'NVIDIA' not in vendor.upper() or any(s in device.lower() for s in ('llvmpipe', 'softpipe', 'software')):
            raise RuntimeError(f'Hardware rendering is required: {report}')
        started = time.perf_counter()
        for _ in range(120):
            renderer.update_scene(data, camera='front')
            renderer.render()
        report['simple_frames_per_second'] = 120 / (time.perf_counter() - started)
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
