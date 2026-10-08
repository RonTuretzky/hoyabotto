"""MCP door into the farm program: an agent (Claude Code on the robot laptop) can look and can
run named skills. It stops at the skill layer.

    farm mcp -p paper-tray-v0            # stdio MCP server; add it to an MCP client

Look:   get_state, get_camera_image, list_keyframes, recent_evidence
Act:    stop, go_rest, go_keyframe          (same clamps, same STOP, every call written to evidence)
Never:  raw joint targets, wheel commands, limits, authorizing a pour, editing the profile

Modelled on windht/xlerobot-mcp, which exposes raw servo positions. Ours does not, because the
rule here is that a model never writes a joint command directly.
"""
from __future__ import annotations

import time
from typing import Any

from .safety.rules import SafetyStop


class FarmTools:
    """The tool bodies, independent of the MCP transport so they can be tested directly."""

    def __init__(self, system):
        self.s = system

    def _log(self, tool: str, args: dict[str, Any], result: str) -> None:
        self.s.store.event(None, "mcp_call", {"tool": tool, "args": args, "result": result})

    # ---- look ----------------------------------------------------------------
    def get_state(self) -> dict[str, Any]:
        s = self.s
        j, h = s.robot.joints(), s.robot.health()
        return {
            "profile": s.profile.name, "simulated": s.profile.simulated,
            "joints_status": j.status.value, "joints": {k: round(float(v), 1) for k, v in (j.value or {}).items()} if j.ok else {},
            "health_status": h.status.value,
            "stop_pressed": s.skills.estop.is_set(), "needs_person": bool(s.state.get("needs_person")),
            "cameras": {n: c.frame().status.value for n, c in s.cameras.items()},
            "keyframes": s.keyframes.names(), "held": dict(s.skills.held),
        }

    def get_camera_image(self, camera: str = "head") -> tuple[bytes, dict[str, Any]]:
        """JPEG bytes of the latest frame plus its status. Raises ValueError for an unknown camera or a frame that is not OK."""
        import cv2
        cam = self.s.cameras.get(camera)
        if cam is None:
            raise ValueError(f"unknown camera {camera!r}; choose from {sorted(self.s.cameras)}")
        r = cam.frame()
        if not r.ok:
            raise ValueError(f"camera {camera} frame is {r.status.value}: {r.note}")
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(r.value, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise ValueError("could not encode frame")
        return buf.tobytes(), {"camera": camera, "status": r.status.value, "age_s": round(r.age(), 2), "shape": list(r.value.shape)}

    def list_keyframes(self) -> list[dict[str, Any]]:
        out = []
        for n in self.s.keyframes.names():
            m = self.s.keyframes.meta(n) or {}
            out.append({"name": n, "arm": m.get("arm", ""), "learned_by": m.get("learned_by", ""), "note": m.get("note", ""), "joints": len(m.get("joints", {}))})
        return out

    def recent_evidence(self, n: int = 10) -> dict[str, Any]:
        n = max(1, min(int(n), 50))
        return {"cycles": self.s.store.recent_cycles(n), "open_actions": self.s.store.open_actions()}

    # ---- act (skills only) -----------------------------------------------------
    def stop(self) -> str:
        self.s.skills.estop.set()
        self.s.skills.stop()
        self._log("stop", {}, "stopped")
        return "stopped: motors hold position. STOP stays set until a named person resumes from the viewer."

    def _refuse(self) -> str | None:
        if self.s.skills.estop.is_set():
            return "refused: STOP is set. A named person must resume from the viewer."
        if self.s.state.get("needs_person"):
            return "refused: an action is UNKNOWN and needs a person to reconcile it in the viewer."
        return None

    def go_rest(self) -> str:
        why = self._refuse()
        if why:
            self._log("go_rest", {}, why); return why
        try:
            r = self.s.skills.go_rest()
            msg = "at rest" if r.ok else f"did not settle: {r.note}"
        except SafetyStop as e:
            msg = f"safety stop: {e}"
        self._log("go_rest", {}, msg)
        return msg

    def go_keyframe(self, name: str) -> str:
        why = self._refuse()
        if why:
            self._log("go_keyframe", {"name": name}, why); return why
        kf = self.s.keyframes.get(name)
        if kf is None:
            msg = f"refused: no keyframe named {name!r}. Taught keyframes: {self.s.keyframes.names()}"
            self._log("go_keyframe", {"name": name}, msg); return msg
        if any(k.startswith("base_") or "wheel" in k for k in kf):
            msg = "refused: keyframe contains wheel joints"
            self._log("go_keyframe", {"name": name}, msg); return msg
        try:
            r = self.s.skills.move_joints(kf, max_s=10)
            msg = f"at keyframe {name}" if r.ok else f"did not settle: {r.note}"
        except SafetyStop as e:
            msg = f"safety stop: {e}"
        self._log("go_keyframe", {"name": name}, msg)
        return msg


def build_server(system):
    """FastMCP server with the tools above. Imported lazily so the farm program does not need `mcp` unless this is used."""
    try:
        from mcp.server.mcpserver import Image, MCPServer as FastMCP      # mcp 2.x
    except ImportError:
        from mcp.server.fastmcp import FastMCP, Image                      # mcp 1.x

    import contextlib
    import functools
    import sys

    tools = FarmTools(system)

    class _Quiet:
        """Tool bodies may print (skills, safety messages); stdout is the MCP transport, so send that to stderr."""

        def __getattr__(self, name):
            fn = getattr(tools, name)

            @functools.wraps(fn)
            def call(*a, **k):
                with contextlib.redirect_stdout(sys.stderr):
                    return fn(*a, **k)
            return call

    t = _Quiet()
    mcp = FastMCP("xlerobot-farm", instructions=(
        "Read-only views of the farm robot plus three skill-level actions. You cannot set joint targets, drive wheels, "
        "change limits or authorize watering. If a tool answers 'refused' or 'safety stop', report it; do not try another route."))

    @mcp.tool()
    def get_state() -> dict:
        """Joint positions, camera status, taught keyframes, whether STOP is pressed or a person is needed."""
        return t.get_state()

    @mcp.tool()
    def get_camera_image(camera: str = "head"):
        """Latest frame from 'head', 'left_wrist' or 'right_wrist' as a JPEG."""
        data, _ = t.get_camera_image(camera)
        return Image(data=data, format="jpeg")

    @mcp.tool()
    def list_keyframes() -> list:
        """Taught poses: name, arm, who or what taught it."""
        return t.list_keyframes()

    @mcp.tool()
    def recent_evidence(n: int = 10) -> dict:
        """The last n care cycles and any actions whose result is still open or UNKNOWN."""
        return t.recent_evidence(n)

    @mcp.tool()
    def stop() -> str:
        """Hold every motor where it is and set STOP. Only a named person can resume, from the viewer."""
        return t.stop()

    @mcp.tool()
    def go_rest() -> str:
        """Move both arms and the head to the rest pose, in clamped steps."""
        return t.go_rest()

    @mcp.tool()
    def go_keyframe(name: str) -> str:
        """Move to a taught pose by name, in clamped steps. Refuses unknown names."""
        return t.go_keyframe(name)

    return mcp


def serve(system) -> None:
    server = build_server(system)
    try:
        server.run()            # stdio
    finally:
        try:
            system.skills.go_rest()
        except Exception:  # noqa: BLE001
            pass
        system.disconnect()
