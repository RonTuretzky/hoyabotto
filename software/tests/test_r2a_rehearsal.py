import json
import time

import numpy as np
import pytest
import yaml

from farm.assembly.measure import fit_station, fit_station_file
from farm.assembly.r2a import (
    GripCfg, HeldState, Stage, Variant, check_paper_pick, evidence_problems,
    held_state_from_gripper, load_assembly_profile,
)
from farm.assembly.rehearsal import FAULTS, Rehearsal, required_keyframes, save_trace
from farm.status import Reading, Status


def profile(variant=Variant.NO_FRAME):
    p = load_assembly_profile()
    p.variant = variant
    return p


@pytest.mark.parametrize("variant", list(Variant))
def test_nominal_rehearsal_is_not_hardware_success_or_training_data(variant):
    p = profile(variant)
    result = Rehearsal(p).run()
    assert result["result"] == "SIMULATED_COMPLETE"
    assert not result["physical_success"] and not result["training_eligible"]
    assert result["account"]["label"] == "RIGID_PRACTICE"
    assert result["held"] == "EMPTY" and p.execution_enabled is False
    parts = [e["part"] for e in result["events"] if e["kind"] == "synthetic_action" and e["action"] == "release"]
    assert parts == (["carrier", "paper", "retainer"] if variant is Variant.WITH_FRAME else ["carrier", "paper"])
    move_safe = next(e["seq"] for e in result["events"] if e.get("action") == "safe_pose" and e["kind"] == "synthetic_action")
    verify_safe = next(e["seq"] for e in result["events"] if e.get("name") == "robot_at_safe_pose")
    assert move_safe < verify_safe


@pytest.mark.parametrize("fault", FAULTS[1:])
def test_every_fault_stops_without_retry_or_recovery_motion(fault):
    result = Rehearsal(profile(Variant.WITH_FRAME), fault).run()
    assert result["result"] == "SIMULATED_STOPPED"
    assert all(n == 1 for n in result["attempts"].values())
    stop = next(i for i, e in enumerate(result["events"]) if e["kind"] == "stop_hold")
    assert not any(e["kind"] == "synthetic_action" for e in result["events"][stop + 1:])
    assert not result["physical_success"] and result["reason"]


@pytest.mark.parametrize("fault", ["no_pick", "lost_carrier", "seat_unknown", "deadline", "stop_during_transfer"])
def test_carrier_failure_never_releases_or_parks(fault):
    result = Rehearsal(profile(), fault).run()
    actions = [e["action"] for e in result["events"] if e["kind"] == "synthetic_action"]
    assert "release" not in actions and "safe_pose" not in actions and "retreat" not in actions


@pytest.mark.parametrize("fault", ["double_paper", "no_paper", "paper_unknown"])
def test_paper_count_checked_before_transfer(fault):
    result = Rehearsal(profile(), fault).run()
    actions = [e["action"] for e in result["events"] if e["kind"] == "synthetic_action" and e["part"] == "paper"]
    assert actions == ["approach", "grasp", "close", "lift"]


def test_trace_is_immutable_and_episode_cannot_resume(tmp_path):
    sim = Rehearsal(profile(), "seat_unknown")
    result = sim.run()
    path = tmp_path / "trace.json"
    save_trace(result, path)
    assert json.loads(path.read_text())["held"] == "HELD"
    with pytest.raises(FileExistsError):
        save_trace(result, path)
    with pytest.raises(RuntimeError):
        sim.run()
    with pytest.raises(ValueError):
        Rehearsal(profile(), "frame_failed")


def test_keyframes_are_isolated_from_watering():
    assert len(required_keyframes(Variant.NO_FRAME)) == 13
    assert len(required_keyframes(Variant.WITH_FRAME)) == 19
    assert all(n.startswith("r2a_") for n in required_keyframes(Variant.WITH_FRAME))


@pytest.mark.parametrize("value", [None, 1, "true", {}, []])
def test_ok_status_does_not_make_unknown_value_affirmative(value):
    assert evidence_problems({"x": Reading(value, Status.OK)}, ("x",))


@pytest.mark.parametrize("timestamp", [0, float("nan"), float("inf"), time.time() + 1000])
def test_old_or_invalid_observation_timestamp_blocks(timestamp):
    assert evidence_problems({"x": Reading(True, Status.OK, t=timestamp)}, ("x",))


@pytest.mark.parametrize("count", [1.5, True, "1", -1])
def test_paper_count_does_not_coerce_invalid_values(count):
    assert check_paper_pick(count).status is Status.UNKNOWN


@pytest.mark.parametrize("low,high", [(20, 10), (10, 10), (-1, 20), (1, float("nan")), (1, float("inf"))])
def test_invalid_grips_cannot_infer_a_held_part(low, high):
    assert GripCfg(carrier_empty_max=low, carrier_holding_min=high).problems()
    assert held_state_from_gripper(Reading(30, Status.OK), low, high) is HeldState.UNKNOWN


POINTS = np.array([[-50, -30, 0], [50, -30, 0], [50, 30, 0], [-50, 30, 0]], dtype=float)
ROTATION = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
TRANSLATION = np.array([.3, -.1, .025])
META = dict(method="test measurements", measured_on="2026-10-03", by="test", calibration_id="test-cal")


def points_robot(a):
    return (ROTATION @ (np.asarray(a) / 1000).T).T + TRANSLATION


def test_station_fit_preserves_units_rotation_and_translation():
    station, report = fit_station(POINTS, points_robot(POINTS), **META)
    assert np.allclose(station.matrix[:3, :3], ROTATION)
    assert np.allclose(station.apply([10, 20, 30]), points_robot([10, 20, 30]))
    assert report["max_error_mm"] < 1e-8


@pytest.mark.parametrize("bad", ["units", "collinear", "outlier", "nan", "cluster"])
def test_station_fit_rejects_bad_measurements(bad):
    a, b = POINTS.copy(), points_robot(POINTS)
    if bad == "units":
        b *= 1000
    elif bad == "collinear":
        a[:, 1] = 0
    elif bad == "outlier":
        b[0, 2] += .020
    elif bad == "nan":
        a[0, 0] = np.nan
    elif bad == "cluster":
        a *= .001
        b = points_robot(a)
    with pytest.raises(ValueError):
        fit_station(a, b, **META)


def test_fit_file_requires_independent_validation_and_preserves_existing_file(tmp_path):
    data = dict(kind="r2a-station-measurements", **META, assembly_mm=POINTS.tolist(),
                robot_m=points_robot(POINTS).tolist(), check_assembly_mm=[0, 0, 0],
                check_robot_m=points_robot([0, 0, 0]).tolist())
    source, dest = tmp_path / "points.yaml", tmp_path / "station.yaml"
    source.write_text(yaml.safe_dump(data))
    assert fit_station_file(source, dest)["check_error_mm"] < 1e-8
    with pytest.raises(FileExistsError):
        fit_station_file(source, dest)
    data["check_robot_m"][2] += .010
    source.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="independent check error"):
        fit_station_file(source, tmp_path / "bad.yaml")
    assert not (tmp_path / "bad.yaml").exists()


def test_cli_simulation_and_fault_exit_codes(tmp_path, capsys):
    from farm.cli import main
    main(["r2a", "--simulate", "--output", str(tmp_path / "ok.json")])
    assert "SIMULATED_COMPLETE" in capsys.readouterr().out
    with pytest.raises(SystemExit) as exc:
        main(["r2a", "--simulate", "--fault", "double_paper", "--output", str(tmp_path / "bad.json")])
    assert exc.value.code == 2
    assert json.loads((tmp_path / "bad.json").read_text())["result"] == "SIMULATED_STOPPED"


@pytest.mark.parametrize("change", [
    {"execution_enabled": "false"}, {"deadlines_s": {"PREFLIGHT": float("nan")}},
    {"deadlines_s": {"PREFLIGHT": 0}}, {"max_attempts": {"PLACE_PREPARED_CARRIER": 2}},
    {"max_attempts": {"PLACE_PREPARED_CARRIER": 1.5}},
    {"state_joints": ["right_arm_gripper"]},
])
def test_malformed_profile_cannot_enable_motion_or_retries(tmp_path, change):
    data = {**profile().raw, **change}
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_assembly_profile(str(path))


def test_station_refuses_millimetre_matrix():
    from farm.assembly.frames import RobotFromAssembly
    with pytest.raises(ValueError, match="units"):
        RobotFromAssembly.from_dict({"units": "millimetres"})
