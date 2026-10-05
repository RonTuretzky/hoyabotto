import time

import pytest

from carton.supervisor import CartonSupervisor, Primitive
from test_jev_motion import Backend


def state(seq=1, **kw):
    return {"sequence": seq, "observed_at": time.time(), "camera_ok": True, "box_visible": True,
            "health_ok": True, "stop_requested": False, "obstruction": False, "geometry_known": True,
            "station_revision": "station1", "calibration_id": "cal1", "stage": "pick_paddle", **kw}


def setup(**kw):
    motions, stops = [], []
    def execute(guard):
        guard(); motions.append("align"); return {"acknowledged": True}
    p = Primitive("align", "Align the visible paddle handle using the validated correction.", "station1", "cal1",
                  "pick_paddle", execute, lambda s: s.get("handle_visible") is True)
    b = Backend({"route": "execute", "primitive": "align"})
    return CartonSupervisor(b, [p], lambda: stops.append(1), **kw), b, motions, stops


def test_executes_registered_primitive_once_then_requires_fresh_post_observation():
    c, b, motions, stops = setup()
    assert c.step(state(handle_visible=True))["executed"]
    assert c.step(state(2, handle_visible=True))["route"] == "verify_result"
    assert not c.step(state(2, handle_visible=True))["executed"]
    assert c.step(state(3, handle_visible=True, verification={"primitive": "align", "result": "failure"}))["route"] == "request_plan"
    assert c.step(state(4, handle_visible=True))["route"] == "request_plan"
    assert motions == ["align"] and len(stops) == 5 and b.calls == 1


@pytest.mark.parametrize("change,route", [
    ({"stop_requested": True}, "stop"), ({"health_ok": False}, "stop"),
    ({"obstruction": True}, "stop"), ({"camera_ok": False}, "refresh_view"),
    ({"observed_at": 0}, "refresh_view"), ({"geometry_known": False}, "request_plan"),
    ({"station_revision": "moved"}, "request_plan"), ({"calibration_id": "changed"}, "request_plan"),
    ({"handle_visible": False}, "request_plan"), ({"consecutive_failures": 2}, "request_plan"),
    ({"motion_pending_verification": True}, "verify_result"), ({"health_ok": None}, "unknown"),
])
def test_rules_preempt_api_and_motion(change, route):
    c, b, motions, stops = setup()
    assert c.step(state(**{"handle_visible": True, **change}))["route"] == route
    assert not motions and b.calls == 0 and stops


def test_finish_matches_actual_scope_and_model_cannot_claim_it():
    folded = dict.fromkeys(("short_1", "short_2", "long_1", "long_2"), True)
    c, b, motions, _ = setup()
    assert c.step(state(flaps_folded=folded))["route"] == "finish"
    assert not motions and b.calls == 0
    c, b, motions, _ = setup(goal="fold_and_tape")
    b.choice["route"] = "finish"
    assert c.step(state(handle_visible=True, flaps_folded=folded))["route"] == "verify_result"
    assert not motions


def test_stop_during_inference_and_uncertainty_never_execute():
    c, b, motions, _ = setup()
    b.callback = c.request_stop
    assert c.step(state(handle_visible=True))["route"] == "stop"
    assert not motions
    c, b, motions, _ = setup()
    b.p = .7
    assert c.step(state(handle_visible=True))["route"] == "request_plan"
    assert not motions


def test_driving_requires_stowed_clear_setup_and_invalidates_registration():
    c, b, motions, _ = setup()
    old = c.primitives["align"]
    c.primitives = {"align": Primitive("align", "Reposition the base.", "station1", "cal1", "position_base",
                                      old.execute, lambda s: True, drives=True)}
    assert c.step(state(stage="position_base", arms_stowed=False, base_clear=True, in_contact=False))["route"] == "request_plan"
    assert c.step(state(2, stage="position_base", arms_stowed=True, base_clear=True, in_contact=False))["executed"]
    assert c.step(state(3, handle_visible=True))["route"] == "request_plan"
    assert len(motions) == 1
