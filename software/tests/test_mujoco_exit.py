"""Open MuJoCo renderers are closed before interpreter shutdown (farm.mujoco_exit)."""
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")

from farm import mujoco_exit  # noqa: E402

SOFTWARE = Path(__file__).resolve().parents[1]
XML = '<mujoco><worldbody><light pos="0 0 1"/><geom size=".1"/></worldbody></mujoco>'


def _renderer():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    try:
        renderer = mujoco.Renderer(model, 32, 32)
    except Exception as e:  # noqa: BLE001 - no offscreen GL on this machine
        pytest.skip(f"no offscreen rendering: {e}")
    renderer.update_scene(data)
    renderer.render()
    return renderer


def test_hook_closes_every_open_renderer_once():
    renderer = _renderer()
    assert mujoco_exit.close_open_renderers() >= 1
    assert renderer._mjr_context is None and renderer._gl_context is None
    assert mujoco_exit.close_open_renderers() == 0


def test_importing_the_packages_installs_the_hook():
    import carton  # noqa: F401
    import planter  # noqa: F401
    assert mujoco_exit._installed


def test_script_that_leaves_a_renderer_open_exits_cleanly():
    code = textwrap.dedent(f"""
        import carton, mujoco
        class Sim:  # a renderer held in a reference cycle, as the simulations hold theirs
            def __init__(self):
                self.model = mujoco.MjModel.from_xml_string({XML!r})
                self.data = mujoco.MjData(self.model)
                self.renderer = mujoco.Renderer(self.model, 32, 32)
                self.me = self
        sim = Sim()
        sim.renderer.update_scene(sim.data)
        sim.renderer.render()
    """)
    proc = subprocess.run([sys.executable, '-c', code], cwd=SOFTWARE, capture_output=True, text=True,
                          env={**os.environ, 'PYTHONPATH': str(SOFTWARE)}, timeout=120)
    assert proc.returncode == 0, proc.stderr[-2000:]
