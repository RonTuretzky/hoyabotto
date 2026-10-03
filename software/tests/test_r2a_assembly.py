"""R2a assembly contract: frames and units, revision check, stage machine, evidence gating, held-object rule,
paper pick, intervention accounting, fail-closed dataset ticks, dataset/checkpoint compatibility, held-out split,
and the disabled profile. No robot, no motion."""
import json
import math
import time

import numpy as np
import pytest

from farm.assembly import episodes as ep
from farm.assembly import frames as fr
from farm.assembly import r2a
from farm.assembly.schema import TIMING, DatasetSchema, StrictTick
from farm.status import Reading, Status

RIGHT = [f"right_arm_{j}" for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")]


# ---- frames and units -----------------------------------------------------------------
def test_export_to_assembly_translations_match_handoff():
    assert fr.stl_to_assembly_mm("carrier", (0, -36.5, 32.5)).tolist() == [0, -36.5, 12.0]       # carrier fin centre
    assert fr.stl_to_assembly_mm("retainer", (72, 0, 10)).tolist() == [72, 0, 11.0]              # frame fin centre, seated
    assert fr.stl_to_assembly_mm("trough", (1, 2, 3)).tolist() == [1, 2, 3]
    assert fr.assembly_to_stl_mm("carrier", (0, 0, 0)).tolist() == [0, 0, 20.5]
    with pytest.raises(KeyError):
        fr.stl_to_assembly_mm("bottle", (0, 0, 0))                                                # R0/R1 objects do not exist here


def test_render_shift_never_enters_a_transform():
    p = fr.stl_to_assembly_mm("carrier", (0, 0, 20.5))
    assert p[2] == 0.0 and fr.RENDER_WORLD_SHIFT_MM[2] == 23.5


def test_grasp_candidates_are_nominal_and_gap_is_half_mm():
    c = fr.grasp_candidates()
    assert [g.part for g in c] == ["carrier", "carrier", "retainer", "retainer"]
    assert all(g.status == "nominal_cad" for g in c)
    assert c[0].closure_axis == "Y" and c[0].thickness_mm == 6.0
    assert c[2].closure_axis == "X" and c[2].thickness_mm == 4.0 and c[2].point_A_mm[2] == 11.0
    assert fr.nominal_frame_to_fin_gap_mm() == pytest.approx(0.5)


def test_unmeasured_transform_refuses_robot_coordinates(tmp_path):
    t = fr.RobotFromAssembly.load(tmp_path / "missing.yaml")
    assert not t.measured
    with pytest.raises(fr.TransformNotMeasured):
        t.apply((0, 0, 0))
    with pytest.raises(ValueError):   # "measured" without saying how is not measured
        fr.RobotFromAssembly.from_dict({"matrix": np.eye(4).tolist(), "measured": True})


def test_measured_transform_converts_mm_to_metres_and_round_trips(tmp_path):
    m = np.eye(4); m[:3, 3] = [0.30, -0.10, 0.02]                      # fixture origin 30 cm ahead, 10 cm right, 2 cm up
    t = fr.RobotFromAssembly(m, measured=True, method="ruler+tag", measured_on="2026-10-04", by="ron", residual_mm=1.5)
    p = t.apply((0, -36.5, 12))
    assert np.allclose(p, [0.30, -0.1365, 0.032])
    t.save(tmp_path / "station.yaml")
    t2 = fr.RobotFromAssembly.load(tmp_path / "station.yaml")
    assert t2.measured and np.allclose(t2.matrix, m) and t2.by == "ron"
    with pytest.raises(ValueError):
        fr.RobotFromAssembly(np.diag([2.0, 1, 1, 1]), measured=True, method="x", measured_on="d", by="b")   # not a rotation


# ---- parts / revision -----------------------------------------------------------------
def test_release_parts_in_repo_match_handoff_hashes():
    rep = r2a.verify_parts()
    assert r2a.parts_ok(rep), rep
    src = r2a.verify_parts(r2a.PARTS_DIR / "source", r2a.SOURCE_SHA256)
    assert r2a.parts_ok(src), src


def test_wrong_revision_rejected(tmp_path):
    (tmp_path / "carrier.stl").write_bytes(b"not the carrier")
    rep = r2a.verify_parts(tmp_path)
    assert not rep["carrier.stl"]["ok"] and "mismatch" in rep["carrier.stl"]["note"]
    assert rep["retainer.stl"]["note"] == "missing"
    assert not r2a.parts_ok(rep)


# ---- stage machine -----------------------------------------------------------------------
def _ok(names):
    return {n: Status.OK for n in names}


def test_stage_order_per_variant_and_illegal_jumps():
    assert r2a.Stage.PLACE_RETAINER not in r2a.stage_order(r2a.Variant.NO_FRAME)
    assert r2a.Stage.PLACE_RETAINER in r2a.stage_order(r2a.Variant.WITH_FRAME)
    m = r2a.AssemblyMachine(r2a.Variant.NO_FRAME)
    with pytest.raises(r2a.IllegalTransition):
        m.go(r2a.Stage.PLACE_REAL_TOP_PAPER)           # cannot skip the carrier
    with pytest.raises(r2a.IllegalTransition):
        m.go(r2a.Stage.PLACE_RETAINER)                 # not in this variant at all
    m.pause("test")
    with pytest.raises(r2a.IllegalTransition):
        m.go(r2a.Stage.PLACE_PREPARED_CARRIER)         # PAUSED never resumes straight into a placement
    m.go(r2a.Stage.FAILED)
    assert m.stage is r2a.Stage.FAILED and r2a.transitions(r2a.Variant.NO_FRAME)[r2a.Stage.FAILED] == set()


def test_advance_requires_every_required_evidence_ok():
    m = r2a.AssemblyMachine(r2a.Variant.WITH_FRAME)
    req = r2a.required_evidence(r2a.Stage.PREFLIGHT, r2a.Variant.WITH_FRAME)
    assert "frame_present" in req and "station_measured" in req and "grip_thresholds_measured" in req
    with pytest.raises(r2a.EvidenceMissing) as e:
        m.advance({})
    assert set(e.value.problems) == set(req) and all(v == "missing" for v in e.value.problems.values())
    ev = _ok(req); ev["gripper_empty"] = Reading(None, Status.UNKNOWN, note="thresholds not measured")
    with pytest.raises(r2a.EvidenceMissing) as e:
        m.advance(ev)
    assert e.value.problems == {"gripper_empty": "UNKNOWN"}
    ev["gripper_empty"] = Reading(False, Status.OK)
    with pytest.raises(r2a.EvidenceMissing):
        m.advance(ev)                                   # an OK reading whose value is False is still a block
    ev["gripper_empty"] = True
    assert m.advance(ev) is r2a.Stage.PLACE_PREPARED_CARRIER
    ev2 = _ok(r2a.required_evidence(m.stage, m.variant)); ev2["carrier_stays_after_release"] = Status.STALE
    with pytest.raises(r2a.EvidenceMissing):
        m.advance(ev2)


def test_full_with_frame_run_reaches_done_and_no_frame_skips_retainer():
    for variant in r2a.Variant:
        m = r2a.AssemblyMachine(variant)
        seen = []
        while m.stage is not r2a.Stage.DONE:
            seen.append(m.stage)
            m.advance(_ok(r2a.required_evidence(m.stage, variant)))
        assert seen == r2a.stage_order(variant)


def test_deadline_expiry_blocks_advance():
    m = r2a.AssemblyMachine(r2a.Variant.NO_FRAME, deadlines_s={"PREFLIGHT": 0.01})
    time.sleep(0.03)
    assert m.overdue()
    with pytest.raises(r2a.EvidenceMissing) as e:
        m.advance(_ok(r2a.required_evidence(m.stage, m.variant)))
    assert "deadline" in e.value.problems


def test_bounded_attempts_no_blind_retry():
    m = r2a.AssemblyMachine(r2a.Variant.NO_FRAME)
    m.advance(_ok(r2a.required_evidence(m.stage, m.variant)))
    assert m.stage is r2a.Stage.PLACE_PREPARED_CARRIER
    assert m.begin_attempt() == 1
    with pytest.raises(r2a.IllegalTransition) as e:
        m.begin_attempt()
    assert "physical reset" in str(e.value)


# ---- held object, paper, accounting -------------------------------------------------------
def test_unknown_held_object_forbids_open_and_park_but_allows_stop_and_ask():
    for a in ("open_gripper", "release", "park", "go_rest", "retry_insertion"):
        ok, why = r2a.recovery_allowed(r2a.HeldState.UNKNOWN, a)
        assert not ok and "UNKNOWN" in why
    for a in ("stop", "hold", "ask_person", "capture_evidence"):
        assert r2a.recovery_allowed(r2a.HeldState.UNKNOWN, a)[0]
    assert not r2a.recovery_allowed(r2a.HeldState.HELD, "park")[0]
    assert r2a.recovery_allowed(r2a.HeldState.EMPTY, "park")[0]


def test_held_state_needs_measured_thresholds_not_bottle_numbers():
    g = Reading(30.0, Status.OK)
    assert r2a.held_state_from_gripper(g, None, None) is r2a.HeldState.UNKNOWN
    assert r2a.held_state_from_gripper(g, 8.0, 20.0) is r2a.HeldState.HELD
    assert r2a.held_state_from_gripper(Reading(5.0, Status.OK), 8.0, 20.0) is r2a.HeldState.EMPTY
    assert r2a.held_state_from_gripper(Reading(12.0, Status.OK), 8.0, 20.0) is r2a.HeldState.UNKNOWN   # between: ambiguous
    assert r2a.held_state_from_gripper(Reading(30.0, Status.STALE), 8.0, 20.0) is r2a.HeldState.UNKNOWN


def test_paper_pick_double_none_and_unknown():
    assert r2a.check_paper_pick(1).value is True
    d = r2a.check_paper_pick(2)
    assert d.value is False and "double" in d.note
    assert r2a.check_paper_pick(0).value is False
    assert r2a.check_paper_pick(None).status is Status.UNKNOWN


def test_episode_labels_interventions_end_success_and_rigid_is_never_success():
    V, S = r2a.Variant, r2a.Stage
    a = r2a.EpisodeAccount(V.WITH_FRAME, real_paper=True)
    assert a.label() == "INCOMPLETE"
    for s in r2a.stage_order(V.WITH_FRAME):
        a.record(s, "VERIFIED")
    assert a.label() == "SUCCESS" and a.counts_as_autonomous_success()
    a.intervene("ron", S.PLACE_REAL_TOP_PAPER, "straightened the sheet")
    assert a.label() == "INTERVENED" and not a.counts_as_autonomous_success()
    b = r2a.EpisodeAccount(V.NO_FRAME, real_paper=False)
    for s in r2a.stage_order(V.NO_FRAME):
        b.record(s, "VERIFIED")
    assert b.label() == "RIGID_PRACTICE"
    c = r2a.EpisodeAccount(V.WITH_FRAME, real_paper=True)
    for s in r2a.stage_order(V.WITH_FRAME):
        c.record(s, "VERIFIED")
    c.record(S.PLACE_RETAINER, "SKIPPED")                  # a silently omitted frame stage is not a completed with-frame episode
    assert c.label() == "FAILED"
    c.safety_faults.append("load limit")
    assert c.label() == "FAILED"


def test_gates_block_on_any_safety_fault():
    assert r2a.gate_passed("placement_stage", 18, 20)[0]
    assert not r2a.gate_passed("placement_stage", 17, 20)[0]
    assert not r2a.gate_passed("full_dry_assembly", 10, 10, safety_faults=1)[0]
    assert not r2a.gate_passed("full_dry_assembly", 9, 9)[0]


# ---- dataset schema: fail closed ------------------------------------------------------------
def _schema():
    return DatasetSchema("right", RIGHT, {"wrist": "right_wrist", "head": "head"}, fps=10, frame_hw=(48, 64))


def _frames(now, cams=("right_wrist", "head"), age=0.0):
    return {c: Reading(np.zeros((48, 64, 3), np.uint8), Status.OK, t=now - age) for c in cams}


def _joints(now, **over):
    j = {n: 0.0 for n in RIGHT}
    j.update({f"left_arm_{k}": 0.0 for k in ("shoulder_pan", "gripper")})
    j.update(over)
    return Reading(j, Status.OK, t=now)


def test_schema_rejects_mixed_arms_and_names_joints_in_order():
    with pytest.raises(ValueError):
        DatasetSchema("right", RIGHT + ["left_arm_gripper"], {"wrist": "right_wrist"})
    s = _schema()
    assert s.state_names[0] == "right_arm_shoulder_pan.pos" and len(s.features()) == 2 + 2 + 1
    assert s.features()[TIMING]["names"] == ["t_obs", "t_action", "dt_prev", "age_wrist", "age_head"]


def test_strict_tick_accepts_a_good_tick_with_timing():
    now = time.time()
    st = StrictTick(_schema(), step_max=6.0, watchdog_s=0.5)
    act = {n: 2.0 for n in RIGHT}
    v = st.build(_joints(now), act, _frames(now, age=0.1), now)
    assert v.ok, v.reason
    assert v.frame["action"]["right_arm_gripper.pos"] == 2.0
    tm = v.frame["extra"][TIMING]
    assert tm.shape == (5,) and math.isnan(float(tm[2])) and abs(float(tm[3]) - 0.1) < 0.01
    v2 = st.build(_joints(now + 0.1), act, _frames(now + 0.1), now + 0.1)
    assert v2.ok and abs(float(v2.frame["extra"][TIMING][2]) - 0.1) < 1e-3
    assert st.summary()["accepted"] == 2


def test_strict_tick_rejects_missing_stale_malformed_and_unclamped():
    now = time.time()
    st = StrictTick(_schema(), step_max=6.0, watchdog_s=0.5)
    good = {n: 1.0 for n in RIGHT}
    cases = [
        (Reading(None, Status.INVALID, t=now, note="bus"), good, _frames(now), "joints not OK"),
        (_joints(now - 1.0), good, _frames(now), "joints stale"),
        (Reading({k: v for k, v in _joints(now).value.items() if k != "right_arm_wrist_roll"}, Status.OK, t=now), good, _frames(now), "joint missing"),
        (_joints(now), None, _frames(now), "action missing"),
        (_joints(now), {**good, "right_arm_gripper": float("nan")}, _frames(now), "action non-finite"),
        (_joints(now), {**good, "right_arm_gripper": "closed"}, _frames(now), "action malformed"),
        (_joints(now), {**good, "right_arm_gripper": -5.0}, _frames(now), "action out of bounds"),
        (_joints(now), {**good, "right_arm_elbow_flex": 40.0}, _frames(now), "action exceeds step clamp"),
        (_joints(now), good, _frames(now, cams=("right_wrist",)), "frame missing"),
        (_joints(now), good, {**_frames(now), "head": Reading(None, Status.STALE, t=now - 2)}, "frame not OK"),
        (_joints(now), good, _frames(now, age=3.0), "frame stale"),
        (_joints(now), good, {**_frames(now), "head": Reading(np.zeros((48, 64), np.uint8), Status.OK, t=now)}, "frame malformed"),
    ]
    for joints, act, frames, expect in cases:
        v = st.build(joints, act, frames, now)
        assert not v.ok and v.reason.startswith(expect), (expect, v.reason)
    assert st.accepted == 0 and st.summary()["rejected_total"] == len(cases)


def test_checkpoint_compatibility_is_by_exact_keys_and_dims():
    s = _schema()
    good = {"input_features": {"observation.state": {"type": "STATE", "shape": [6]}, "observation.images.wrist": {"type": "VISUAL", "shape": [3, 480, 640]}, "observation.images.head": {"type": "VISUAL", "shape": [3, 480, 640]}},
            "output_features": {"action": {"type": "ACTION", "shape": [6]}}}
    assert s.problems_with_checkpoint(good) == []
    pour = json.loads(json.dumps(good)); pour["input_features"].pop("observation.images.head"); pour["input_features"]["observation.images.overhead"] = {"type": "VISUAL", "shape": [3, 480, 640]}
    p = s.problems_with_checkpoint(pour)
    assert p and "overhead" in p[0] and "not interchangeable" in p[0]
    fourteen = json.loads(json.dumps(good)); fourteen["input_features"]["observation.state"]["shape"] = [14]
    assert any("state dim" in x for x in s.problems_with_checkpoint(fourteen))


def test_info_compatibility_catches_fourteen_joint_watering_dataset():
    s = _schema()
    info = {"fps": 10, "features": s.features()}
    assert s.problems_with_info(info) == []
    fourteen = {"fps": 10, "features": {**s.features(), "observation.state": {"dtype": "float32", "shape": (14,), "names": ["x"] * 14}}}
    assert any("observation.state" in p for p in s.problems_with_info(fourteen))


@pytest.mark.skipif(pytest.importorskip("lerobot.datasets.lerobot_dataset", reason="lerobot") is None, reason="lerobot")
def test_r2a_recorder_writes_timing_and_refuses_incompatible_resume(tmp_path):
    from farm.assembly.schema import IncompatibleDataset, R2aRecorder
    from farm.learning.recorder import EpisodeRecorder
    s = _schema()
    root = tmp_path / "ds"
    rec = R2aRecorder(s, root, "farm/test_r2a")
    st = StrictTick(s, step_max=6.0, watchdog_s=0.5)
    rec.start_episode("R2a_no_frame PLACE_PREPARED_CARRIER")
    now = time.time()
    n = 0
    for i in range(4):
        t = now + i * 0.1
        v = st.build(_joints(t), {k: 1.0 for k in RIGHT}, _frames(t), t)
        n += rec.tick(v)
    bad = st.build(_joints(now + 1), None, _frames(now + 1), now + 1)
    assert rec.tick(bad) is False
    assert rec.end_episode(save=True) == 4 == n
    rec.close()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("farm/test_r2a", root=root)
    assert ds.num_frames == 4 and ds.meta.features[TIMING]["names"] == s.timing_names
    assert ds.meta.features["observation.state"]["names"] == s.state_names
    assert (root / "meta" / "r2a_schema.json").exists()
    # same schema resumes; a different schema (other camera set) is refused
    R2aRecorder(s, root, "farm/test_r2a").close()
    other = DatasetSchema("right", RIGHT, {"wrist": "right_wrist"}, fps=10, frame_hw=(48, 64))
    with pytest.raises(IncompatibleDataset):
        R2aRecorder(other, root, "farm/test_r2a")
    # and the 14-joint watering recorder's dataset is refused too
    wroot = tmp_path / "watering"
    w = EpisodeRecorder(wroot, "farm/own-runs", 10, ["head"], (48, 64), [f"{a}_arm_{j}.pos" for a in ("left", "right") for j in ("shoulder_pan", "gripper")])
    w.close()
    with pytest.raises(IncompatibleDataset):
        R2aRecorder(s, wroot, "farm/own-runs")


# ---- episodes and split -------------------------------------------------------------------
def _meta(eid, session, split=""):
    return ep.EpisodeMeta(episode_id=eid, session_id=session, mesh_sha256=r2a.MESH_SHA256, printed_instance="carrier#1", material_profile="M5C PLA+",
                          support_cleanup="brim+grid removed", manual_modifications="none", variant="R2a_no_frame", stage_results={"PREFLIGHT": "VERIFIED"},
                          label="FAILED", human_wick_preload="4 wicks by ron", top_medium_type="kitchen paper", top_sheet_cut_mm=[146, 64], top_sheet_count=1,
                          top_sheet_moisture="dry", trough_instance="trough#1", trough_orientation="refill bay -X", robot_id="farm_xlerobot", calibration_id="cal",
                          tool_jaw_geometry="stock SO-101 jaws", joint_order=RIGHT, station_transform_file="data/r2a/station.yaml",
                          camera_identities={"right_wrist": "usb-1", "head": "usb-2"}, images={"pre_grasp": "h1"}, schema_version="r2a-dataset-1", code_commit="abc", split=split)


def test_episode_meta_requires_every_field(tmp_path):
    m = ep.EpisodeMeta(episode_id="e1")
    miss = m.missing()
    assert "session_id" in miss and "label" in miss and "split" in miss
    with pytest.raises(ValueError):
        m.save(tmp_path)
    full = _meta("e1", "s1", "train")
    assert full.missing() == []
    p = full.save(tmp_path)
    assert ep.EpisodeMeta.load(p).session_id == "s1"


def test_split_is_by_session_and_holdout_is_test():
    plan = ep.SplitPlan(holdout_sessions=["s-test"], val_percent=15)
    assert plan.assign("s-test") == "test"
    a = plan.assign("s1"); assert a == plan.assign("s1") and a in ("train", "val")
    with pytest.raises(ValueError):
        plan.assign("")


def test_manifest_and_held_out_check(tmp_path):
    plan = ep.SplitPlan(holdout_sessions=["s9"])
    eps = [_meta("e1", "s1"), _meta("e2", "s1"), _meta("e3", "s9")]
    p = ep.write_manifest(tmp_path, eps, plan)
    man = json.loads(p.read_text())
    assert man["counts"]["test"] == 1
    assert ep.is_held_out(man, ["e3"]) == (True, "")
    ok, why = ep.is_held_out(man, ["e1"]); assert not ok and "not test" in why
    with pytest.raises(ValueError):
        ep.write_manifest(tmp_path, [_meta("e4", "s9", split="train")], plan)   # recorded split must agree with the plan


# ---- profile ----------------------------------------------------------------------------
def test_profile_loads_disabled_and_execution_is_refused_with_reasons():
    p = r2a.load_assembly_profile("r2a-assembly-v0")
    assert p.execution_enabled is False and p.controlled_arm == "right" and p.variant is r2a.Variant.NO_FRAME
    assert p.state_joints == RIGHT and p.cameras == {"wrist": "right_wrist", "head": "head"}
    assert p.dataset_root.endswith("data/r2a/dataset") and p.dataset_root != "data/dataset"
    blockers = r2a.execution_blockers(p)
    assert any("execution_enabled" in b for b in blockers)
    assert any("not measured" in b and "station" in b for b in blockers)
    assert any("grip thresholds" in b for b in blockers)
    assert not any(b.startswith("parts") for b in blockers)
    with pytest.raises(r2a.ExecutionDisabled):
        r2a.assert_execution_allowed(p)


def test_profile_loader_refuses_the_watering_profile_and_other_revisions(tmp_path):
    with pytest.raises(ValueError):
        r2a.load_assembly_profile("paper-tray-v0")
    bad = tmp_path / "r0.yaml"
    bad.write_text("kind: r2a-assembly\nrevision: R0\ncontrolled_arm: right\nstate_joints: [right_arm_gripper]\ncameras: {wrist: right_wrist}\n")
    with pytest.raises(ValueError):
        r2a.load_assembly_profile(str(bad))
    mixed = tmp_path / "mixed.yaml"
    mixed.write_text("kind: r2a-assembly\nrevision: R2a\ncontrolled_arm: right\nstate_joints: [right_arm_gripper, left_arm_gripper]\ncameras: {wrist: right_wrist}\n")
    with pytest.raises(ValueError):
        r2a.load_assembly_profile(str(mixed))
