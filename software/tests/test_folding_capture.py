"""Search-mode replay recording must preserve state without rendering/dynamics."""
from types import SimpleNamespace
import numpy as np
from carton.folding_sim import FoldingSimulation


def test_trace_only_capture_is_an_independent_timestamped_state():
    sim = object.__new__(FoldingSimulation)
    sim.data = SimpleNamespace(time=3.25, qpos=np.array([1.,2.,3.]))
    sim.frame_states, sim.frames, sim.capture_images = [], [], False
    def forbidden_render(*args):
        raise AssertionError('Trace-only search must not render presentation images')
    sim.render = forbidden_render
    sim.capture('measured step')
    assert sim.frames == []
    assert sim.data.time == 3.25
    np.testing.assert_array_equal(sim.data.qpos, [1.,2.,3.])
    sim.data.qpos[:] = 0
    assert sim.frame_states == [{'time':3.25,'qpos':[1.,2.,3.],'label':'measured step'}]
