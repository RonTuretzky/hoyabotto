"""The care cycle on the simulator: happy path, every pause path, unknown delivery, authority ladder."""
import time

from conftest import AFTER_JUDGEMENT, GOOD_JUDGEMENT, FakeBackends, jev_reply

from farm.adapters.sim import Faults
from farm.cycle.runner import CareCycle
from farm.llm.backends import ScriptedLLM


def vision(*judgements):
    return ScriptedLLM(list(judgements))


def test_happy_path_pours_with_human_authorization(sim):
    # inspect, pre-pour look, verify
    s = sim(vision=vision(GOOD_JUDGEMENT, GOOD_JUDGEMENT, AFTER_JUDGEMENT))
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "POURED", out
    acts = {a["skill"]: a["result"] for a in s.store.query("SELECT skill, result FROM actions WHERE cycle_id=?", (out.cycle_id,))}
    assert acts["pour"] == "VERIFIED" and acts["park"] == "VERIFIED" and acts["pick_tool"] == "VERIFIED"
    assert s.skills.held["right"] == "bottle"
    assert any(i["who"] == "sim-operator" for i in s.store.query("SELECT who FROM interventions"))
    assert s.store.open_actions() == []


def test_no_answer_means_no_water(sim):
    s = sim(script={}, vision=vision(GOOD_JUDGEMENT))
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "NO_POUR"
    assert not any(a["skill"] == "pour" for a in s.store.query("SELECT skill FROM actions"))


def test_stale_camera_pauses_before_any_motion_toward_water(sim):
    f = Faults(); f.set("camera_stale")
    s = sim(script={}, vision=vision(GOOD_JUDGEMENT), faults=f)
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "PAUSED" and "not usable" in out.note
    assert not any(a["skill"] in ("pour", "pick_tool") for a in s.store.query("SELECT skill FROM actions"))


def test_unknown_spill_after_pour_blocks_retry_until_reconciled(sim):
    after_unknown = {**AFTER_JUDGEMENT, "spill": "UNKNOWN"}
    s = sim(script={"pour:": "authorize one pour"}, vision=vision(GOOD_JUDGEMENT, GOOD_JUDGEMENT, after_unknown))
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "PAUSED" and "UNKNOWN" in out.note
    assert s.store.open_actions() and s.store.open_actions()[0]["skill"] == "pour"
    # next cycle refuses to start while the pour is unresolved
    s2 = CareCycle(s, s.profile.tray("A")); out2 = s2.run()
    assert out2.result == "PAUSED" and "unresolved" in out2.note
    # a person reconciles from the viewer
    aid = s.store.open_actions()[0]["action_id"]
    s.store.result(aid, "RECONCILED_OK", note="looked")
    assert s.store.open_actions() == []


def test_upright_recovery_failure_pauses_with_bottle_evidence(sim):
    s = sim(vision=vision(GOOD_JUDGEMENT, GOOD_JUDGEMENT, AFTER_JUDGEMENT))
    # break motion after the pour tilt: robot disconnect during the pour
    f = s.faults
    orig = s.skills.pour

    def pour_then_disconnect(tilt, secs, on_attempt=None):
        f.set("robot_disconnect")
        return orig(tilt, secs, on_attempt=on_attempt)
    s.skills.pour = pour_then_disconnect
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "PAUSED"
    res = {a["skill"]: a["result"] for a in s.store.query("SELECT skill, result FROM actions WHERE cycle_id=?", (out.cycle_id,))}
    assert res.get("pour") in ("UNKNOWN", "ABORTED")


def test_rules_block_when_opening_not_visible(sim):
    j = {**GOOD_JUDGEMENT, "opening_visible": "unknown"}
    s = sim(vision=vision(j))
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "NO_POUR"
    d = s.store.query("SELECT choice, note FROM decisions WHERE kind='rule'")[0]
    assert d["choice"] == "block" and "opening_visible" in d["note"]


def test_budget_exhausted_blocks(sim):
    s = sim(vision=vision(GOOD_JUDGEMENT), backends=FakeBackends(vision=vision(GOOD_JUDGEMENT), budget_exhausted=True))
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "NO_POUR" and "budget" in s.store.query("SELECT note FROM decisions WHERE kind='rule'")[0]["note"]


def test_jev_route_is_honoured_only_after_promotion(sim):
    jev = ScriptedLLM([jev_reply("inspect_water"), jev_reply("skip", choices=("pour", "skip", "reinspect", "unknown"))])
    s = sim(vision=vision(GOOD_JUDGEMENT, GOOD_JUDGEMENT, AFTER_JUDGEMENT), jev=jev)
    assert s.authority.level == "shadow"
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "POURED"    # in shadow the human's authorization wins; Jev was only recorded
    recs = s.store.query("SELECT kind, honoured, note FROM decisions WHERE kind LIKE 'jev%'")
    assert len(recs) == 2 and all(r["honoured"] == 0 for r in recs)
    assert any(r["note"].startswith("disagree") for r in recs if r["kind"] == "jev_pour")


def test_jev_approve_level_authorizes_without_asking(sim):
    jev = ScriptedLLM([jev_reply("routine"), jev_reply("pour", p=0.95, choices=("pour", "skip", "reinspect", "unknown"))])
    s = sim(script={}, vision=vision(GOOD_JUDGEMENT, GOOD_JUDGEMENT, AFTER_JUDGEMENT), jev=jev)
    s.authority.set_level("approve", "test", "forced")
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "POURED"
    v = [e for e in s.store.query("SELECT payload_json FROM events WHERE kind='verdict'")]
    assert '"by": "jev"' in v[0]["payload_json"]
    assert not s.human.log  # nobody was asked


def test_jev_low_probability_at_approve_level_still_asks(sim):
    jev = ScriptedLLM([jev_reply("routine"), jev_reply("pour", p=0.6, choices=("pour", "skip", "reinspect", "unknown"))])
    s = sim(script={}, vision=vision(GOOD_JUDGEMENT), jev=jev)
    s.authority.set_level("approve", "test", "forced")
    out = CareCycle(s, s.profile.tray("B")).run()
    assert out.result == "NO_POUR" and s.human.log  # asked, and silence means no water


def test_authority_promotes_on_evidence(sim):
    s = sim(script={})
    n = s.profile.authority.jev_shadow_cycles
    for _ in range(n):
        s.store.decision("c", "jev_route", "q", "", "routine", {}, "v", note="agree:x")
    s.authority.consider_promotion()
    assert s.authority.level == "route"
    for _ in range(n):
        s.store.decision("c", "jev_pour", "q", "", "pour", {}, "v", note="agree:x")
    s.authority.consider_promotion()
    assert s.authority.level == "approve"
    s.authority.demote_after_false_approval("spill")
    assert s.authority.level == "route"


def test_servo_learns_a_keyframe_from_scripted_model(sim):
    from farm.skills.llm_servo import LLMServo
    s = sim()
    model = ScriptedLLM([
        {"action": "move", "dx_mm": 10, "dy_mm": -5, "confidence": 0.8, "why": "toward the opening"},
        {"action": "move", "dx_mm": 40, "dpitch_deg": 30, "confidence": 0.8, "why": "clamped step"},
        {"action": "done", "confidence": 0.9, "why": "spout over opening"},
    ])
    servo = LLMServo(s.skills, s.cameras, model, s.store, None, max_steps=5)
    out = servo.run("right", "spout above tray B opening", save_as="pour_B_learned")
    assert out.ok and out.steps == 3
    assert "pour_B_learned" in s.keyframes.names()
    assert len(model.calls) == 3 and model.calls[0]["n_images"] == 2
    # the 40 mm request was clamped to 15 mm (unit check on the model itself)
    from farm.skills.arm import ArmModel, ArmPose
    m = ArmModel("right"); m.pose = ArmPose(0.15, 0.10)
    m.apply_delta(dx_mm=40, dpitch=30)
    assert abs(m.pose.x - 0.165) < 1e-9 and m.pose.pitch == 6.0


def test_servo_aborts_when_model_aborts(sim):
    from farm.skills.llm_servo import LLMServo
    s = sim()
    servo = LLMServo(s.skills, s.cameras, ScriptedLLM([{"action": "abort", "why": "target not visible"}]), None, None, max_steps=5)
    out = servo.run("right", "x")
    assert not out.ok and "aborted" in out.reason


def test_astra_apply_safe_respects_bounds(sim, tmp_path):
    from farm.llm.astra import Astra
    s = sim()
    ov = s.profile.data_path / "overrides.yaml"
    astra = Astra(ScriptedLLM([{"title": "faster inspect", "change": {"key": "deadlines.inspect_s", "from": 5, "to": 8}, "expected": "x", "rollback": "y", "priority": "low"}]), s.store, "apply-safe", ov)
    r = astra.review(s.profile.raw)
    assert r["applied"] and ov.exists()
    astra2 = Astra(ScriptedLLM([{"title": "loosen", "change": {"key": "limits.pour_s_max", "from": 4, "to": 40}, "expected": "x", "rollback": "y"}]), s.store, "apply-safe", ov)
    r2 = astra2.review(s.profile.raw)
    assert not r2["applied"]
    assert s.store.query("SELECT status FROM proposals ORDER BY t")[1]["status"] == "OPEN"


def test_head_servo_moves_only_head_and_saves_keyframe(sim):
    from farm.skills.llm_servo import LLMServo
    s = sim()
    model = ScriptedLLM([{"action": "move", "dpan_deg": 20, "dpitch_deg": -3, "confidence": 0.8, "why": "turn"},
                         {"action": "done", "confidence": 0.9, "why": "centred"}])
    before = dict(s.robot.pos)
    out = LLMServo(s.skills, s.cameras, model, s.store, None, max_steps=4).run("head", "centre tray B", save_as="look_B_learned")
    assert out.ok and "look_B_learned" in s.keyframes.names()
    kf = s.keyframes.get("look_B_learned")
    assert set(kf) == {"head_motor_1", "head_motor_2"}
    assert abs(s.robot.goal["head_motor_1"] - 6.0) < 1e-6          # 20° request clamped to the 6° step
    assert all(abs(s.robot.goal[j] - before[j]) < 1e-6 for j in before if j.startswith("right_arm") or j.startswith("left_arm"))


def test_estop_halts_any_skill(sim):
    from farm.safety.rules import SafetyStop
    s = sim()
    s.skills.estop.set()
    try:
        s.skills.go_rest(); assert False, "should have stopped"
    except SafetyStop as e:
        assert "STOP" in str(e)
    s.skills.estop.clear()
    assert s.skills.go_rest().ok
