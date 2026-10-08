"""Close MuJoCo renderers before interpreter shutdown.

A `mujoco.Renderer` that is still open when Python exits is freed during module teardown, in
whatever order garbage collection reaches it. When its OpenGL context goes first, `mjr_freeContext`
calls `glDeleteTextures` with no current context and the process segfaults after its work is done
(exit 139, a macOS "Python quit unexpectedly" dialog). Seen on 2026-10-08 from simulations that render
without reaching their own cleanup (e.g. `FoldingSimulation` closes its renderer only in `save()`).

`install()` registers an `atexit` hook, which runs before module teardown, that closes every renderer
still alive. It imports nothing heavy and does nothing unless mujoco was imported. The farm, carton and
planter packages install it on import, so their scripts and tests need no changes.
"""
from __future__ import annotations

import atexit
import gc
import sys

_installed = False


def close_open_renderers() -> int:
    """Close every live mujoco.Renderer; returns how many were open."""
    mujoco = sys.modules.get('mujoco')
    renderer_type = getattr(mujoco, 'Renderer', None)
    if renderer_type is None:
        return 0
    closed = 0
    for obj in gc.get_objects():
        # type(), not isinstance(): isinstance reads __class__, which some lazy/deprecated objects warn on.
        if issubclass(type(obj), renderer_type) and (getattr(obj, '_mjr_context', None) is not None
                                                     or getattr(obj, '_gl_context', None) is not None):
            try:
                obj.close()
                closed += 1
            except Exception:  # noqa: BLE001 - shutdown must not raise
                pass
    return closed


def install() -> None:
    global _installed
    if not _installed:
        atexit.register(close_open_renderers)
        _installed = True
