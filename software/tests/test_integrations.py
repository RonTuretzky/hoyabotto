"""The MCP tool bodies, all on the simulator. No hardware, no network."""
from farm.adapters.base import ARM_JOINTS, arm_joint


# ---- MCP tool bodies ----------------------------------------------------------------------------
def test_mcp_look_tools(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    t = FarmTools(s)
    st = t.get_state()
    assert st["joints_status"] == "OK" and len(st["joints"]) == 14 and st["stop_pressed"] is False
    assert st["health_status"] == "OK" and "hottest" not in st and st["keyframes"]
    data, meta = t.get_camera_image("head")
    assert data[:2] == b"\xff\xd8" and meta["shape"] == [480, 640, 3]
    import pytest
    with pytest.raises(ValueError, match="unknown camera"):
        t.get_camera_image("ceiling")
    kfs = t.list_keyframes()
    assert kfs and {"name", "arm", "learned_by", "note", "joints"} <= set(kfs[0])
    assert set(t.recent_evidence(500)) == {"cycles", "open_actions"}


def test_mcp_actions_go_through_skills_and_respect_stop(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    t = FarmTools(s)
    name = s.keyframes.names()[0]
    assert t.go_keyframe(name).startswith(("at keyframe", "did not settle"))
    assert t.go_keyframe("no_such_pose").startswith("refused: no keyframe")
    s.keyframes.save("bad_wheels", {"base_left_wheel": 10.0}, arm="")
    assert "wheel" in t.go_keyframe("bad_wheels")
    assert t.go_rest() in ("at rest",) or t.go_rest().startswith("did not settle")
    assert t.stop().startswith("stopped") and s.skills.estop.is_set()
    sent = len(s.robot.sent)
    assert t.go_rest().startswith("refused: STOP is set") and t.go_keyframe(name).startswith("refused: STOP is set")
    assert len(s.robot.sent) == sent                                # nothing commanded after STOP
    calls = s.store.query("select payload_json from events where kind = 'mcp_call'")
    assert len(calls) >= 6                                          # every action call was written down


def test_mcp_refuses_motion_while_an_action_is_unknown(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    s.state["needs_person"] = True
    assert FarmTools(s).go_rest().startswith("refused: an action is UNKNOWN")


def test_mcp_server_exposes_exactly_the_seven_tools(sim):
    import asyncio
    from farm.mcp_server import build_server
    server = build_server(sim())
    names = sorted(t.name for t in asyncio.run(server.list_tools()))
    assert names == ["get_camera_image", "get_state", "go_keyframe", "go_rest", "list_keyframes", "recent_evidence", "stop"]
