"""Carton task on the simulator: geometry, plan, judgement parsing, and the cycle's paths."""
import pytest

from carton import perception
from carton.cycle import CartonCycle
from carton.geometry import Box, Stance, reach_report, targets
from carton.plan import KEYFRAMES, STEPS, GRIP_STEPS, RELEASE_AFTER, needed
from carton.sim import SimCartonVision
from farm.adapters.sim import Faults, ScriptedHuman
from farm.config import load_profile
from farm.system import System


def _sys(tmp_path, faults=None, script=None):
    from carton.cli import seed_sim_keyframes
    p = load_profile("carton-sim")
    p.data_dir = str(tmp_path / "d"); p.raw["data_dir"] = p.data_dir
    faults = faults or Faults()
    s = System(p, human=ScriptedHuman(script if script is not None else {}), faults=faults)
    s.backends.vision = SimCartonVision(faults)
    s.connect()
    seed_sim_keyframes(s)
    return s


# ---- geometry ------------------------------------------------------------------------------
def test_measured_box_is_reachable_with_the_paddle_but_not_bare_handed():
    box = Box()
    assert abs(box.seam_gap - 0.003) < 1e-9 and abs(box.short_flap_gap - 0.099) < 1e-9
    r = reach_report(box, Stance(setback=0.06, height=0.15))
    assert r["all_within_reach"]
    far = next(x for x in r["rows"] if x["target"] == "long_flap_far")
    assert far["tool"] == "paddle" and far["distance_m"] > Stance().reach(False)       # fingers alone would not reach it
    assert r["max_setback_far_flap_fingers_m"] < 0.03 < r["max_setback_far_flap_paddle_m"]
    assert not reach_report(box, Stance(setback=0.25, height=0.15))["all_within_reach"]


def test_targets_split_the_work_between_the_arms():
    arms = {t.name: (t.arm, t.paddle) for t in targets(Box(), Stance())}
    assert arms["short_flap_left"] == ("left", False) and arms["long_flap_near"] == ("left", False) and arms["tape_centre"] == ("left", False)
    assert arms["short_flap_right"] == ("right", True) and arms["long_flap_far"] == ("right", True)


# ---- plan ----------------------------------------------------------------------------------
def test_plan_is_consistent():
    names = {k.name for k in KEYFRAMES}
    assert len(names) == len(KEYFRAMES)
    for st in STEPS:
        assert set(st.keyframes) <= names, st.name
        assert all(k.arm == st.arm for k in KEYFRAMES if k.name in st.keyframes), st.name
    assert [s.name for s in STEPS] == ["pick_paddle", "fold_short_left", "fold_short_right", "fold_long_far", "fold_long_near", "tape", "press"]
    assert set(GRIP_STEPS) | set(RELEASE_AFTER) <= {s.name for s in STEPS}
    assert needed({"look_box"})[0].name != "look_box" and len(needed(set())) == len(KEYFRAMES)


def test_satisfied_reads_flaps_and_booleans():
    j = {"short_left": "folded", "long_far": "open", "tape_on_seam": True, "paddle_held": "unknown"}
    assert perception.satisfied(j, "short_left_folded") is True
    assert perception.satisfied(j, "long_far_folded") is False
    assert perception.satisfied(j, "short_right_folded") is None
    assert perception.satisfied(j, "tape_on_seam") is True and perception.satisfied(j, "paddle_held") is None


# ---- cycle ---------------------------------------------------------------------------------
def test_cycle_closes_the_box_and_writes_evidence(tmp_path):
    s = _sys(tmp_path)
    out = CartonCycle(s, Box()).run()
    assert out.result == "CLOSED", out.note
    assert out.steps_done == [st.name for st in STEPS]
    acts = s.store.query("select skill, result from actions order by intent_t")
    assert [a["skill"] for a in acts] == [st.name for st in STEPS] and all(a["result"] == "VERIFIED" for a in acts)
    assert s.skills.held["right"] == "paddle"
    assert s.store.recent_cycles(1)[0]["result"] == "CLOSED"
    s.disconnect()


def test_stuck_flap_is_retried_then_a_person_decides(tmp_path):
    f = Faults(); f.set("flap_stuck", "fold_long_far")
    s = _sys(tmp_path, f, script={"carton:fold_long_far": "done"})
    out = CartonCycle(s, Box()).run()
    assert out.result == "CLOSED" and "fold_long_far" in out.steps_done
    acts = s.store.query("select skill, result, note from actions where skill='fold_long_far' order by intent_t")
    assert len(acts) == 2 and all(a["result"] == "ABORTED" for a in acts)              # two attempts, both failed the check
    assert any(e for e in s.store.query("select kind from events where kind='person_confirmed'"))
    assert s.human.log and s.human.log[0]["question_id"] == "carton:fold_long_far"
    s.disconnect()


def test_unknown_judgement_pauses_and_silence_stops(tmp_path):
    f = Faults()
    s = _sys(tmp_path, f)                       # no script: the person is silent
    cyc = CartonCycle(s, Box())
    orig = cyc._play

    def play_then_blind(step):
        err = orig(step)
        if step.name == "fold_short_left":
            f.set("judge_unknown")
        return err
    cyc._play = play_then_blind
    out = cyc.run()
    assert out.result == "STOPPED" and "fold_short_left" in out.note
    assert s.store.query("select result from actions where skill='fold_short_left'")[0]["result"] == "UNKNOWN"
    assert s.store.recent_cycles(1)[0]["result"] == "STOPPED"
    s.disconnect()


def test_stop_button_ends_the_cycle_immediately(tmp_path):
    s = _sys(tmp_path)
    cyc = CartonCycle(s, Box())
    orig = cyc._play

    def play(step):
        if step.name == "fold_short_right":
            s.skills.estop.set()
        return orig(step)
    cyc._play = play
    out = cyc.run()
    assert out.result == "STOPPED" and "STOP" in out.note and out.steps_done == ["pick_paddle", "fold_short_left"]
    s.disconnect()


def test_already_folded_flaps_are_skipped_and_no_box_ends_early(tmp_path):
    s = _sys(tmp_path)
    v = s.backends.vision
    v.mark("fold_short_left"); v.mark("fold_short_right")
    out = CartonCycle(s, Box()).run()
    assert out.result == "CLOSED"
    acts = [a["skill"] for a in s.store.query("select skill from actions order by intent_t")]
    assert "fold_short_left" not in acts and "fold_short_right" not in acts and "fold_long_far" in acts
    s.disconnect()
    f = Faults(); f.set("no_box")
    s2 = _sys(tmp_path / "b", f)
    out2 = CartonCycle(s2, Box()).run()
    assert out2.result == "NOT_CLOSED" and "no box" in out2.note
    assert all(set(m) <= {"head_motor_1", "head_motor_2"} for m in s2.robot.sent)        # only the head moved to look
    s2.disconnect()


def test_missing_keyframe_pauses_with_a_clear_message(tmp_path):
    s = _sys(tmp_path)
    s.keyframes.delete("far_touch")
    out = CartonCycle(s, Box()).run()
    assert out.result == "NOT_CLOSED" and "far_touch" in out.note and "teach-all" in out.note
    assert out.steps_done == ["pick_paddle", "fold_short_left", "fold_short_right"]
    s.disconnect()


def test_carton_profiles_load_and_sim_matches_real():
    real, sim = load_profile("carton-v0"), load_profile("carton-sim")
    assert real.raw["carton"]["length_m"] == 0.379 and sim.raw["carton"] == real.raw["carton"]
    assert real.trays == [] and real.light.enabled is False and real.policy.state_joints[6].startswith("right_arm_")
    assert sim.simulated and sim.llm.backend == "sim" and real.robot.wheels is False
