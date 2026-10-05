import time

import numpy as np

from farm.adapters.sim import Faults, FakeCamera, FakeLight, FakeRobot
from farm.evidence.store import EvidenceStore
from farm.llm.backends import LLMError, ScriptedLLM, extract_json
from farm.llm.jev import Jev
from farm.perception.basic import frame_quality, green_fraction
from farm.safety.rules import check_health, clamp_targets
from farm.status import Reading, Status


def test_reading_ages_to_stale():
    r = Reading(1.0, Status.OK, t=time.time() - 5)
    assert r.aged(1.0).status is Status.STALE
    assert r.aged(10.0).status is Status.OK


def test_store_intent_attempt_result_and_crash(tmp_path):
    st = EvidenceStore(tmp_path)
    cid = st.start_cycle("B", "sim", "h", "cal", True)
    a1 = st.intent(cid, "pour", {"tray": "B"})
    st.attempt(a1)
    a2 = st.intent(cid, "park", {})
    assert st.intent(cid, "pour", {"tray": "B"}, action_id=a1) == a1  # duplicate id returns the same record
    ids = st.mark_unknown_after_crash()
    assert ids == [a1]
    rows = {r["action_id"]: r["result"] for r in st.query("SELECT action_id, result FROM actions")}
    assert rows[a1] == "UNKNOWN" and rows[a2] == "ABORTED"
    st.result(a1, "RECONCILED_OK", note="person looked")
    assert st.open_actions() == []


def test_store_images_by_hash(tmp_path):
    st = EvidenceStore(tmp_path)
    img = np.zeros((8, 8, 3), np.uint8)
    h1 = st.save_image(img); h2 = st.save_image(img)
    assert h1 == h2 and st.image_path(h1).exists()


def test_extract_json_tolerates_fences_and_prose():
    assert extract_json('Sure: ```json\n{"a": 1}\n```')["a"] == 1
    assert extract_json('prefix {"a": {"b": [1,2]}} suffix')["a"]["b"] == [1, 2]
    try:
        extract_json("no json here"); assert False
    except LLMError:
        pass


def test_jev_preserves_typed_probabilities_and_falls_back_to_unknown():
    from conftest import ScriptedDecisions
    j = Jev(ScriptedDecisions([{"choice": "usable", "probabilities": {"usable": 0.75, "reacquire": 0.25, "conflicting": 0, "unknown": 0}}]))
    c = j.evidence_quality({"x": 1})
    assert c.choice == "usable" and abs(sum(c.probabilities.values()) - 1) < 1e-6 and c.p == 0.75
    j2 = Jev(ScriptedDecisions([]))  # exhausted backend -> unknown
    assert j2.next_review({}).choice == "unknown"


def test_safety_clamps_and_health():
    cur = {"right_arm_wrist_flex": 0.0, "right_arm_gripper": 50.0}
    out = clamp_targets(cur, {"right_arm_wrist_flex": 50.0, "right_arm_gripper": -20.0}, 6.0)
    assert out["right_arm_wrist_flex"] == 6.0 and out["right_arm_gripper"] == 44.0
    from farm.config import LimitsCfg
    lim = LimitsCfg()
    assert check_health(Reading({"m": {"temperature": 30, "load": 10}}, Status.OK), lim).ok
    assert not check_health(Reading({"m": {"temperature": 60, "load": 10}}, Status.OK), lim).ok
    assert not check_health(Reading(None, Status.INVALID), lim).ok


def test_fake_camera_faults_become_unknown_in_perception():
    f = Faults(); cam = FakeCamera("head", f); cam.connect()
    assert frame_quality(cam.frame()).status is Status.OK
    f.set("dark"); assert frame_quality(cam.frame()).status is Status.UNKNOWN
    f.clear("dark"); f.set("occluded"); assert frame_quality(cam.frame()).status is Status.UNKNOWN
    f.clear(); f.set("camera_stale"); assert frame_quality(cam.frame()).status is Status.STALE
    f.clear(); f.set("growth", 0.8)
    other = FakeCamera("h", Faults()); other.connect()
    assert green_fraction(cam.frame()).value > green_fraction(other.frame()).value


def test_fake_light_statuses():
    f = Faults(); l = FakeLight(f); l.connect()
    assert l.latest().ok and l.measure(5).ok
    f.set("esp32_silent"); assert l.latest().status is Status.STALE
    f.clear(); f.set("esp32_error"); assert l.latest().status is Status.INVALID
    assert FakeLight(enabled=False).latest().status is Status.NOT_APPLICABLE


def test_fake_robot_moves_and_disconnects():
    f = Faults(); r = FakeRobot(f); r.connect()
    r.move_to({"right_arm_wrist_flex": 30.0}); time.sleep(0.3)
    assert r.joints().value["right_arm_wrist_flex"] > 20
    f.set("robot_disconnect"); assert r.joints().status is Status.INVALID
