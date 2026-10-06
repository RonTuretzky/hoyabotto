"""Automatic calibration of one arm, plus a two-joint manual step for the head.

The limit-seeking itself is LeRobot PR #3282 (vendored in farm/vendor/autocal): each joint is
driven to its mechanical stops and the stall is detected, so nobody sweeps the arm by hand.
That code was tested by its author on a single free-standing SO-101. On the XLeRobot cart the
arm shares its space with the neck, the other arm and the tray rim, and THIS HAS NOT BEEN RUN
ON OUR ROBOT. Hence the staged modes: try one small joint, then the unfold, then the whole arm.

What this module adds around the vendored code:
  - the arm's port comes from the profile (left = port1, right = port2); head and wheel servos
    on the same chain are not in the motor table, so they are not touched
  - the six results are merged into the XLeRobot calibration file under left_arm_* / right_arm_*
    names, leaving the other arm and the head as they were; wheel entries are filled in (full turn)
  - `calibrate_head`: the head has two joints and a camera cable, so it stays a hands-on step
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Callable

ARM_PORT = {"left": "port1", "right": "port2"}
WHEELS = {"base_left_wheel": 9, "base_right_wheel": 10}
HEAD = {"head_motor_1": 7, "head_motor_2": 8}

CHECKLIST = """Before it moves ({arm} arm):
  - the OTHER arm is folded and turned away from this one
  - nothing is on the cart's top tray near the arm; bottle, paddle and trays are out of reach
  - the head camera cable has slack and is not in the arm's path
  - you can reach the battery switch; switching 12 V off stops everything
  - Ctrl-C makes the motors go limp: be ready to catch the arm
The base rotation is swept last, through its whole travel. Watch that stage: if the arm is
going to touch the neck or the other arm, switch 12 V off."""


def merge_arm(path: Path, arm: str, results: dict[str, Any]) -> dict[str, dict[str, int]]:
    """Write six SO-101 joint calibrations into the XLeRobot file as {arm}_arm_{joint}; keep everything else."""
    cal = json.loads(path.read_text()) if path.is_file() else {}
    for name, mc in results.items():
        get = (lambda k: mc[k]) if isinstance(mc, dict) else (lambda k: getattr(mc, k))
        cal[f"{arm}_arm_{name}"] = {"id": int(get("id")), "drive_mode": int(get("drive_mode")), "homing_offset": int(get("homing_offset")),
                                    "range_min": int(get("range_min")), "range_max": int(get("range_max"))}
    for name, mid in WHEELS.items():          # wheels turn freely: full range, no homing (same as the manual procedure)
        cal.setdefault(name, {"id": mid, "drive_mode": 0, "homing_offset": 0, "range_min": 0, "range_max": 4095})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cal, indent=4))
    return cal


def missing_for_connect(cal: dict[str, Any]) -> list[str]:
    """Motor names the farm program needs in the calibration file before it can connect."""
    from ..adapters.base import ARM_JOINTS, arm_joint
    need = [arm_joint(a, j) for a in ("left", "right") for j in ARM_JOINTS] + list(HEAD) + list(WHEELS)
    return [n for n in need if n not in cal]


def run_arm(robot_cfg, arm: str, mode: str = "full", motor: str | None = None, velocity: int | None = None,
            timeout_s: float | None = None, ask: Callable[[str], str] = input, out: Callable[[str], None] = print,
            workflow=None, cal_path: Path | None = None) -> int:
    """mode: 'motor' (one joint, nothing saved to the file), 'unfold' (init + unfold only), 'full' (whole arm, saved).
    Returns 0 on success, 1 on error, 130 on Ctrl-C, 2 if the person declined."""
    if arm not in ARM_PORT:
        out("--arm must be left or right")
        return 1
    port = getattr(robot_cfg, ARM_PORT[arm])
    if not port:
        out(f"robot.{ARM_PORT[arm]} is empty in the profile. Run `farm devices --probe` and fill it in first.")
        return 1
    if workflow is None:
        from ..vendor.autocal import workflow
    from ..vendor.autocal import calibration_defaults as d
    velocity = velocity or d.DEFAULT_VELOCITY_LIMIT
    timeout_s = timeout_s or d.DEFAULT_TIMEOUT
    out(CHECKLIST.format(arm=arm))
    what = {"motor": f"one joint ({motor}) to both of its stops", "unfold": "the unfold only (shoulder, elbow, wrist lift a little)", "full": "the WHOLE arm, every joint to both stops"}[mode]
    if ask(f"This will move {what} on {port}. Type yes to start: ").strip().lower() != "yes":
        out("not started")
        return 2
    if mode == "motor":
        if motor not in d.MOTOR_NAMES:
            out(f"--motor must be one of {d.MOTOR_NAMES}")
            return 1
        return workflow.calibrate_single_motor(port, motor, velocity_limit=velocity, timeout_s=timeout_s, interactive=True)
    if mode == "unfold":
        return workflow.unfold_joints(port, d.DEFAULT_UNFOLD_ANGLE, interactive=True)
    sink: dict[str, Any] = {}
    rc = workflow.run_full_calibration(port, save=True, velocity_limit=velocity, timeout_s=timeout_s, interactive=True, result_sink=sink)
    if rc != 0:
        out(f"auto-calibration ended with code {rc}; the calibration file was not changed")
        return rc
    if set(sink) != set(d.MOTOR_NAMES):
        out(f"auto-calibration returned {sorted(sink)}, expected all six joints; the calibration file was not changed")
        return 1
    if cal_path is None:
        from .calibration_report import calibration_path
        cal_path = calibration_path(robot_cfg)
    cal = merge_arm(cal_path, arm, sink)
    out(f"{arm} arm saved into {cal_path}")
    still = missing_for_connect(cal)
    if still:
        out("still missing before the farm program can connect: " + ", ".join(still))
    return 0


def calibrate_head(robot_cfg, ask: Callable[[str], str] = input, out: Callable[[str], None] = print, bus=None, cal_path: Path | None = None) -> int:
    """Hands-on, two joints: face the head forward and level, ENTER, then turn and nod it through its travel, ENTER."""
    if not robot_cfg.port1:
        out("robot.port1 is empty in the profile. Run `farm devices --probe` and fill it in first.")
        return 1
    if bus is None:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        bus = FeetechMotorsBus(port=robot_cfg.port1, motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for n, i in HEAD.items()})
    if cal_path is None:
        from .calibration_report import calibration_path
        cal_path = calibration_path(robot_cfg)
    cal = json.loads(cal_path.read_text()) if cal_path.is_file() else {}
    registers = {"homing_offset": "Homing_Offset", "range_min": "Min_Position_Limit", "range_max": "Max_Position_Limit"}

    def read_settings():
        return {n: {key: int(bus.read(reg, n, normalize=False, num_retry=3))
                    for key, reg in registers.items()} for n in HEAD}

    def ensure_released():
        bus.disable_torque(num_retry=3)
        for n in HEAD:
            if bus.read("Torque_Enable", n, normalize=False, num_retry=3) != 0:
                raise RuntimeError(f"Cannot confirm torque off for {n}")

    def write_and_verify(settings):
        ensure_released()
        for n in HEAD:
            for key, reg in registers.items():
                bus.write(reg, n, settings[n][key], normalize=False, num_retry=3)
        if read_settings() != {n: {k: settings[n][k] for k in registers} for n in HEAD}:
            raise RuntimeError("Head calibration readback does not match")

    original = None
    temp_path = None
    bus.connect()
    try:
        ensure_released()
        original = read_settings()
        ask("Head torque is verified off. Point the camera straight ahead and level, looking at the horizon. Press ENTER when ready... ")
        homing = bus.set_half_turn_homings(list(HEAD))
        ensure_released()
        out("RECORDING: gently turn the head left and right, then tilt it up and down by hand. Keep the camera cable loose; do not force a stop or turn a full circle. Return it forward and level, then press ENTER.")
        mins, maxes = bus.record_ranges_of_motion(list(HEAD), display_values=False)
        from .calibration_report import DEG, EDGE_TICKS, NOMINAL
        head = {}
        for name, mid in HEAD.items():
            lo, hi = int(mins[name]), int(maxes[name])
            span = (hi - lo) * DEG
            _, minimum, maximum = NOMINAL[name]
            if not (EDGE_TICKS < lo < hi < 4095 - EDGE_TICKS) or not minimum <= span <= maximum:
                raise ValueError(f"{name}: measured {span:.1f} degrees ({lo}..{hi}); needs review, not saved")
            head[name] = {"id": mid, "drive_mode": 0, "homing_offset": int(homing[name]), "range_min": lo, "range_max": hi}
        write_and_verify(head)
        cal.update(head)
        cal_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=cal_path.parent, prefix=cal_path.name + ".", suffix=".tmp", delete=False) as f:
            temp_path = Path(f.name)
            f.write(json.dumps(cal, indent=4))
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp_path, cal_path)
        temp_path = None
    except BaseException:
        if original is not None:
            try:
                write_and_verify(original)
                out("Previous head settings restored and verified with torque off; calibration file unchanged.")
            except BaseException as restore_error:
                out(f"Could not verify restoration of head settings: {restore_error}")
        raise
    finally:
        try:
            ensure_released()
        finally:
            bus.disconnect(disable_torque=False)
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
    out(f"Head calibration saved and hardware verified with torque off: {cal_path}")
    still = missing_for_connect(cal)
    if still:
        out("still missing before the farm program can connect: " + ", ".join(still))
    return 0
