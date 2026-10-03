"""The R2a assembly contract: revision, stages, evidence, held-object rules, episode accounting.

This is a proposed contract made executable as checks, not a working robot skill. Every
stage is bounded, needs named evidence to advance, and anything UNKNOWN blocks. The
farm's existing safety rules (farm/safety/rules.py) are not touched and not relaxed here.
"""
from __future__ import annotations

import hashlib
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

import yaml

from ..status import Reading, Status, unknown
from .frames import RobotFromAssembly

REVISION = "R2a"
CONTRACT_VERSION = "r2a-contract-1"

# Release hashes from the design handoff (2026-10-03). A part that does not match is a different part.
MESH_SHA256 = {
    "carrier.stl": "438fd4cdaa9ca21353d6958f6854e96f7fa7dffeac08d432e9c352c9838348f2",
    "retainer.stl": "e276ba3f244630971ae02e79efd09929df6dd4bda424b1558e2e915ac100de4c",
    "grip_coupon.stl": "33d8bb116c3120c6c6e8c8ee06866d9a6051125afb754888a7da82d73c00eb3f",
}
SOURCE_SHA256 = {
    "cressmaster-holder.stl": "e4bd4281d031ce14d04ae58e3d353f7d9806f1aff131266eeb13181577c5aebf",
    "cressmaster-trough.stl": "5af456157c9492d5d08c91c981f051d8f2ce7de5e931b36cc9bebcdc5302c5f8",
    "cressmaster-inset.stl": "c72c55b1196989881689eeec1394e4635fd5005f249cecfde2846f653e748037",
}
PLATE_SHA256 = {
    "cress_R2a_full_prototype_plate.3mf": "c95d92b74958a7ee2fed9e820f562e6e02d40eaaec0a3cb9207ddcc4409bcdcb",
    "cress_R2a_full_prototype_plate.gcode": "ee76470bdab8628a0141b8ebe5705b80dcee0c74c1461ce35642cd3b70733ade",
}
PARTS_DIR = Path(__file__).resolve().parents[2] / "parts" / "r2a"
PROFILES_DIR = Path(__file__).resolve().parents[2] / "profiles"


class Variant(str, Enum):
    NO_FRAME = "R2a_no_frame"
    WITH_FRAME = "R2a_with_frame"


class Stage(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    PLACE_PREPARED_CARRIER = "PLACE_PREPARED_CARRIER"
    VERIFY_CARRIER = "VERIFY_CARRIER"
    PLACE_REAL_TOP_PAPER = "PLACE_REAL_TOP_PAPER"
    PLACE_RETAINER = "PLACE_RETAINER"
    VERIFY_DRY_ASSEMBLY = "VERIFY_DRY_ASSEMBLY"
    PAUSED = "PAUSED"
    DONE = "DONE"
    FAILED = "FAILED"


WORK_STAGES = (Stage.PREFLIGHT, Stage.PLACE_PREPARED_CARRIER, Stage.VERIFY_CARRIER, Stage.PLACE_REAL_TOP_PAPER, Stage.PLACE_RETAINER, Stage.VERIFY_DRY_ASSEMBLY)


def stage_order(variant: Variant) -> list[Stage]:
    order = list(WORK_STAGES)
    if variant is Variant.NO_FRAME:
        order.remove(Stage.PLACE_RETAINER)
    return order


def transitions(variant: Variant) -> dict[Stage, set[Stage]]:
    order = stage_order(variant)
    t: dict[Stage, set[Stage]] = {}
    for i, s in enumerate(order):
        nxt = order[i + 1] if i + 1 < len(order) else Stage.DONE
        t[s] = {nxt, Stage.PAUSED, Stage.FAILED}
    # PAUSED resolves only to the verification stages, FAILED or DONE-without-success; never straight into a placement.
    t[Stage.PAUSED] = {Stage.VERIFY_CARRIER, Stage.VERIFY_DRY_ASSEMBLY, Stage.FAILED}
    t[Stage.DONE] = set()
    t[Stage.FAILED] = set()
    return t


# Evidence that must be OK (not UNKNOWN, not STALE, not False) before the stage may be left.
REQUIRED_EVIDENCE: dict[Stage, tuple[str, ...]] = {
    Stage.PREFLIGHT: ("calibration_valid", "cameras_fresh", "trough_secured_empty", "part_revision_ok", "wicks_loaded_4", "paper_present", "gripper_empty", "no_open_actions", "station_measured", "grip_thresholds_measured"),
    Stage.PLACE_PREPARED_CARRIER: ("carrier_held_through_transfer", "trough_unmoved", "carrier_seated_level", "carrier_stays_after_release"),
    Stage.VERIFY_CARRIER: ("seat_not_tilted", "wick_pockets_intact", "wicks_in_place", "gripper_clear_of_part"),
    Stage.PLACE_REAL_TOP_PAPER: ("sheet_count_correct", "sheet_oriented_and_covering", "no_sheet_dropped", "no_sheet_on_jaws"),
    Stage.PLACE_RETAINER: ("frame_even_on_stack", "carrier_and_paper_unmoved", "jaws_released_clean"),
    Stage.VERIFY_DRY_ASSEMBLY: ("carrier_seated", "paper_covers_target", "frame_matches_variant", "gripper_empty", "fixture_undisturbed", "robot_at_safe_pose"),
}
FRAME_ONLY_EVIDENCE = ("frame_present",)   # added to PREFLIGHT when the variant includes the frame

DEFAULT_DEADLINES_S: dict[str, float] = {
    "PREFLIGHT": 60, "PLACE_PREPARED_CARRIER": 60, "VERIFY_CARRIER": 30, "PLACE_REAL_TOP_PAPER": 60, "PLACE_RETAINER": 60, "VERIFY_DRY_ASSEMBLY": 30,
}
DEFAULT_MAX_ATTEMPTS: dict[str, int] = {"PLACE_PREPARED_CARRIER": 1, "PLACE_REAL_TOP_PAPER": 1, "PLACE_RETAINER": 1}


def required_evidence(stage: Stage, variant: Variant) -> tuple[str, ...]:
    req = REQUIRED_EVIDENCE.get(stage, ())
    if stage is Stage.PREFLIGHT and variant is Variant.WITH_FRAME:
        req = req + FRAME_ONLY_EVIDENCE
    return req


class IllegalTransition(RuntimeError):
    pass


class EvidenceMissing(RuntimeError):
    """The stage may not be left: some required evidence is absent, UNKNOWN, STALE or False."""

    def __init__(self, stage: Stage, problems: dict[str, str]):
        self.stage = stage
        self.problems = problems
        super().__init__(f"{stage.value}: " + "; ".join(f"{k}={v}" for k, v in problems.items()))


def evidence_problems(evidence: dict[str, Any], required: tuple[str, ...]) -> dict[str, str]:
    """Which required items are not an affirmative OK. Accepts Reading, Status, bool or str."""
    out: dict[str, str] = {}
    for name in required:
        v = evidence.get(name)
        if v is None:
            out[name] = "missing"
        elif isinstance(v, Reading):
            if v.status is not Status.OK:
                out[name] = v.status.value
            elif v.value is not True:
                out[name] = "false" if v.value is False else "not affirmative"
            elif not math.isfinite(v.t) or not 0 <= v.age() <= 1.0:
                out[name] = "STALE or invalid timestamp"
        elif isinstance(v, Status):
            if v is not Status.OK:
                out[name] = v.value
        elif isinstance(v, bool):
            if not v:
                out[name] = "false"
        elif isinstance(v, str):
            if v.upper() != "OK":
                out[name] = v
        else:
            out[name] = f"unsupported evidence type {type(v).__name__}"
    return out


@dataclass
class AssemblyMachine:
    variant: Variant
    deadlines_s: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_DEADLINES_S))
    max_attempts: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_MAX_ATTEMPTS))
    stage: Stage = Stage.PREFLIGHT
    entered_t: float = field(default_factory=time.time)
    history: list[dict[str, Any]] = field(default_factory=list)
    attempts: dict[str, int] = field(default_factory=dict)
    pause_reason: str = ""
    fail_reason: str = ""

    def __post_init__(self) -> None:
        self._t = transitions(self.variant)

    def order(self) -> list[Stage]:
        return stage_order(self.variant)

    def next_stage(self) -> Stage:
        o = self.order()
        if self.stage not in o:
            raise IllegalTransition(f"no next stage from {self.stage.value}")
        i = o.index(self.stage)
        return o[i + 1] if i + 1 < len(o) else Stage.DONE

    def go(self, new: Stage, why: str = "") -> None:
        if new not in self._t[self.stage]:
            raise IllegalTransition(f"{self.stage.value} -> {new.value} ({self.variant.value})")
        self.history.append({"from": self.stage.value, "to": new.value, "t": time.time(), "why": why})
        self.stage = new
        self.entered_t = time.time()

    def advance(self, evidence: dict[str, Any]) -> Stage:
        """Leave the current stage for the next one, only if every required item is OK."""
        req = required_evidence(self.stage, self.variant)
        problems = evidence_problems(evidence, req)
        if problems:
            raise EvidenceMissing(self.stage, problems)
        if self.overdue():
            raise EvidenceMissing(self.stage, {"deadline": f"{self.elapsed():.1f}s > {self.deadline_s()}s"})
        nxt = self.next_stage()
        self.go(nxt, "evidence ok: " + ",".join(req))
        return nxt

    def begin_attempt(self) -> int:
        """Count an attempt of the current stage; raise when the bounded number is exhausted (no blind retries)."""
        n = self.attempts.get(self.stage.value, 0) + 1
        cap = self.max_attempts.get(self.stage.value)
        if cap is not None and n > cap:
            raise IllegalTransition(f"{self.stage.value}: attempt {n} exceeds max_attempts {cap}; request a physical reset instead")
        self.attempts[self.stage.value] = n
        return n

    def pause(self, reason: str) -> None:
        self.pause_reason = reason
        self.go(Stage.PAUSED, reason)

    def fail(self, reason: str) -> None:
        self.fail_reason = reason
        self.go(Stage.FAILED, reason)

    def deadline_s(self) -> float | None:
        return self.deadlines_s.get(self.stage.value)

    def elapsed(self) -> float:
        return time.time() - self.entered_t

    def overdue(self) -> bool:
        d = self.deadline_s()
        return d is not None and self.elapsed() > d


# ---- held-object rule --------------------------------------------------------------
class HeldState(str, Enum):
    EMPTY = "EMPTY"
    HELD = "HELD"
    UNKNOWN = "UNKNOWN"


# With an unknown object in the jaws nothing may open them, park, or push: a held assembly must not be dropped.
FORBIDDEN_WHEN_UNKNOWN = frozenset({"open_gripper", "release", "park", "go_rest", "retry_insertion", "generic_recovery"})
FORBIDDEN_WHEN_HELD = frozenset({"park", "go_rest", "retry_insertion"})   # a held part goes down a validated path, not a generic one
ALWAYS_ALLOWED = frozenset({"stop", "hold", "ask_person", "capture_evidence"})


def recovery_allowed(held: HeldState, action: str) -> tuple[bool, str]:
    if action in ALWAYS_ALLOWED:
        return True, ""
    if held is HeldState.UNKNOWN and action in FORBIDDEN_WHEN_UNKNOWN:
        return False, f"held object is UNKNOWN: {action} refused; stop, hold, and ask a person to determine a safe recovery"
    if held is HeldState.HELD and action in FORBIDDEN_WHEN_HELD:
        return False, f"a part is held: {action} refused; only a validated place/retreat path may run"
    return True, ""


def held_state_from_gripper(gripper_pos: Reading[float], empty_max: float | None, holding_min: float | None) -> HeldState:
    """Measured thresholds only. With unmeasured thresholds the answer is UNKNOWN, never a guess from the bottle's numbers."""
    if empty_max is None or holding_min is None or gripper_pos.status is not Status.OK or gripper_pos.value is None:
        return HeldState.UNKNOWN
    try:
        g, low, high = float(gripper_pos.value), float(empty_max), float(holding_min)
    except (TypeError, ValueError):
        return HeldState.UNKNOWN
    if (not all(math.isfinite(v) for v in (g, low, high, gripper_pos.t))
            or not 0 <= low < high <= 100 or not 0 <= g <= 100
            or not 0 <= gripper_pos.age() <= 0.5):
        return HeldState.UNKNOWN
    if g <= empty_max:
        return HeldState.EMPTY
    if g >= holding_min:
        return HeldState.HELD
    return HeldState.UNKNOWN


# ---- paper pick --------------------------------------------------------------------
def check_paper_pick(sheets_detected: int | None, declared: int = 1) -> Reading[bool]:
    """One declared sheet (or stack) only. None means the count could not be made: UNKNOWN, not success."""
    if sheets_detected is None:
        return unknown("r2a.paper_pick", "sheet count could not be determined")
    if type(sheets_detected) is not int or type(declared) is not int or declared < 1 or sheets_detected < 0:
        return unknown("r2a.paper_pick", "sheet counts must be nonnegative integers; declared count must be positive")
    n = sheets_detected
    if n == declared:
        return Reading(True, Status.OK, source="r2a.paper_pick", meta={"sheets": n, "declared": declared})
    why = "double pickup" if n > declared else "no sheet picked" if n == 0 else "fewer sheets than declared"
    return Reading(False, Status.OK, source="r2a.paper_pick", note=f"{why}: {n} vs declared {declared}", meta={"sheets": n, "declared": declared})


# ---- episode accounting ------------------------------------------------------------
STAGE_RESULTS = ("VERIFIED", "FAILED", "INTERVENED", "SKIPPED", "UNKNOWN")


@dataclass
class EpisodeAccount:
    """What one episode may be called. Interventions end the autonomous claim; rigid-only practice is never a success."""
    variant: Variant
    real_paper: bool
    stage_results: dict[str, str] = field(default_factory=dict)
    interventions: list[dict[str, Any]] = field(default_factory=list)
    safety_faults: list[str] = field(default_factory=list)

    def declared_stages(self) -> list[str]:
        return [s.value for s in stage_order(self.variant)]

    def record(self, stage: Stage, result: str) -> None:
        if result not in STAGE_RESULTS:
            raise ValueError(result)
        self.stage_results[stage.value] = result

    def intervene(self, who: str, stage: Stage, what: str) -> None:
        self.interventions.append({"who": who, "stage": stage.value, "what": what, "t": time.time()})
        self.stage_results[stage.value] = "INTERVENED"

    def label(self) -> str:
        if self.safety_faults:
            return "FAILED"
        if self.interventions:
            return "INTERVENED"
        for s in self.declared_stages():
            if self.stage_results.get(s) != "VERIFIED":
                return "INCOMPLETE" if self.stage_results.get(s) is None else "FAILED"
        if not self.real_paper:
            return "RIGID_PRACTICE"
        return "SUCCESS"

    def counts_as_autonomous_success(self) -> bool:
        return self.label() == "SUCCESS"

    def to_dict(self) -> dict[str, Any]:
        return {"variant": self.variant.value, "real_paper": self.real_paper, "stage_results": dict(self.stage_results),
                "interventions": list(self.interventions), "safety_faults": list(self.safety_faults), "label": self.label()}


# ---- engineering gates (prototype; agreed before testing, not performance claims) ------
GATES = {"placement_stage": (18, 20), "full_dry_assembly": (9, 10)}


def gate_passed(gate: str, successes: int, trials: int, safety_faults: int = 0) -> tuple[bool, str]:
    need, of = GATES[gate]
    if safety_faults:
        return False, f"{safety_faults} safety fault(s): blocked regardless of success count"
    if trials < of:
        return False, f"{trials}/{of} trials run"
    if successes < need:
        return False, f"{successes}/{trials} successes < {need}/{of}"
    return True, f"{successes}/{trials} within a declared setup envelope"


# ---- parts ------------------------------------------------------------------------
def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_parts(parts_dir: Path | None = None, expected: dict[str, str] | None = None) -> dict[str, dict[str, Any]]:
    """Hash every release mesh. A missing or mismatched file is a different revision, not R2a."""
    d = Path(parts_dir or PARTS_DIR)
    exp = expected or MESH_SHA256
    out: dict[str, dict[str, Any]] = {}
    for name, want in exp.items():
        p = d / name
        if not p.exists():
            out[name] = {"ok": False, "sha256": None, "expected": want, "note": "missing"}
            continue
        got = sha256_of(p)
        out[name] = {"ok": got == want, "sha256": got, "expected": want, "note": "" if got == want else "hash mismatch: not the R2a release"}
    return out


def parts_ok(report: dict[str, dict[str, Any]]) -> bool:
    return bool(report) and all(v["ok"] for v in report.values())


# ---- profile -------------------------------------------------------------------------
class ExecutionDisabled(RuntimeError):
    pass


@dataclass
class GripCfg:
    """Per-part jaw thresholds in the runtime's gripper units (0..100). None = not measured. The bottle's numbers do not apply."""
    carrier_empty_max: float | None = None
    carrier_holding_min: float | None = None
    retainer_empty_max: float | None = None
    retainer_holding_min: float | None = None
    paper_empty_max: float | None = None
    paper_holding_min: float | None = None
    measured_on: str = ""
    by: str = ""

    def missing(self) -> list[str]:
        return [k for k, v in self.__dict__.items() if k.endswith(("_max", "_min")) and v is None]

    def problems(self) -> list[str]:
        out = []
        for part in ("carrier", "retainer", "paper"):
            lo, hi = getattr(self, part + "_empty_max"), getattr(self, part + "_holding_min")
            if lo is None or hi is None:
                continue
            if (type(lo) not in (int, float) or type(hi) not in (int, float)
                    or not math.isfinite(lo) or not math.isfinite(hi) or not 0 <= lo < hi <= 100):
                out.append(f"{part}: require finite 0 <= empty_max < holding_min <= 100")
        if not self.missing() and not all(isinstance(v, str) and v.strip() for v in (self.measured_on, self.by)):
            out.append("measurement date and operator missing")
        return out


@dataclass
class AssemblyProfile:
    name: str
    execution_enabled: bool
    variant: Variant
    controlled_arm: str
    state_joints: list[str]
    cameras: dict[str, str]                   # dataset image key -> farm camera name
    fps: int
    frame_hw: tuple[int, int]
    deadlines_s: dict[str, float]
    max_attempts: dict[str, int]
    grip: GripCfg
    station_file: str
    dataset_repo_id: str
    dataset_root: str
    holdout_sessions: list[str]
    val_percent: int
    parts_dir: str
    require_real_paper: bool
    farm_profile: str                         # the robot/limits profile this runs under (never repurposed)
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def software_dir(self) -> Path:
        return PROFILES_DIR.parent

    def station_path(self) -> Path:
        p = Path(self.station_file)
        return p if p.is_absolute() else self.software_dir / p

    def parts_path(self) -> Path:
        p = Path(self.parts_dir)
        return p if p.is_absolute() else self.software_dir / p

    def dataset_path(self) -> Path:
        p = Path(self.dataset_root)
        return p if p.is_absolute() else self.software_dir / p


def load_assembly_profile(name_or_path: str = "r2a-assembly-v0") -> AssemblyProfile:
    path = Path(name_or_path)
    if not path.exists():
        path = PROFILES_DIR / f"{name_or_path}.yaml"
    raw = yaml.safe_load(path.read_text()) or {}
    if raw.get("kind") != "r2a-assembly":
        raise ValueError(f"{path} is not an r2a-assembly profile (kind={raw.get('kind')!r}); the watering profile is not repurposed")
    if raw.get("revision") != REVISION:
        raise ValueError(f"profile revision {raw.get('revision')!r} is not {REVISION}; R0/R1 geometry is not accepted")
    arm = raw.get("controlled_arm", "")
    if arm not in ("left", "right"):
        raise ValueError("controlled_arm must be 'left' or 'right'")
    joints = list(raw.get("state_joints") or [])
    from ..adapters.base import ARM_JOINTS, arm_joint
    expected = [arm_joint(arm, j) for j in ARM_JOINTS]
    if len(joints) != len(expected) or set(joints) != set(expected):
        raise ValueError(f"state_joints must contain each of the six controlled {arm} arm joints exactly once")
    if type(raw.get("execution_enabled", False)) is not bool:
        raise ValueError("execution_enabled must be a YAML boolean, not a string")
    cams = dict(raw.get("cameras") or {})
    if not cams:
        raise ValueError("cameras must map at least one dataset image key to a farm camera")
    grip = GripCfg(**{k: v for k, v in (raw.get("grip") or {}).items() if k in GripCfg.__dataclass_fields__})
    ds = raw.get("dataset") or {}
    hw = raw.get("frame_hw") or [480, 640]
    deadlines = {**DEFAULT_DEADLINES_S, **{k: float(v) for k, v in (raw.get("deadlines_s") or {}).items()}}
    if any(not math.isfinite(v) or v <= 0 for v in deadlines.values()):
        raise ValueError("every stage deadline must be finite and positive")
    attempts = {**DEFAULT_MAX_ATTEMPTS, **(raw.get("max_attempts") or {})}
    if any(type(v) is not int or v != 1 for v in attempts.values()):
        raise ValueError("R2a permits one attempt per placement; a failure needs a physical reset")
    return AssemblyProfile(
        name=raw.get("profile", path.stem),
        execution_enabled=bool(raw.get("execution_enabled", False)),
        variant=Variant(raw.get("variant", Variant.NO_FRAME.value)),
        controlled_arm=arm,
        state_joints=joints,
        cameras=cams,
        fps=int(raw.get("fps", 10)),
        frame_hw=(int(hw[0]), int(hw[1])),
        deadlines_s=deadlines,
        max_attempts=attempts,
        grip=grip,
        station_file=raw.get("station_file", "data/r2a/station.yaml"),
        dataset_repo_id=ds.get("repo_id", "farm/r2a-assembly"),
        dataset_root=ds.get("root", "data/r2a/dataset"),
        holdout_sessions=list(ds.get("holdout_sessions") or []),
        val_percent=int(ds.get("val_percent", 15)),
        parts_dir=raw.get("parts_dir", "parts/r2a"),
        require_real_paper=bool(raw.get("require_real_paper", True)),
        farm_profile=raw.get("farm_profile", "paper-tray-v0"),
        raw=raw,
    )


def execution_blockers(profile: AssemblyProfile, station: RobotFromAssembly | None = None, parts_report: dict[str, dict[str, Any]] | None = None) -> list[str]:
    """Static contract checks only. Runtime readiness, sensing and paths are additional gates."""
    out: list[str] = []
    if not profile.execution_enabled:
        out.append("profile: execution_enabled is false")
    st = station if station is not None else RobotFromAssembly.load(profile.station_path())
    if not st.measured:
        out.append(f"station: T_robot_from_A not measured ({profile.station_path()})")
    else:
        if not (st.method and st.measured_on and st.by and st.calibration_id):
            out.append("station: measurement provenance or calibration id missing")
        if st.residual_mm is None or not math.isfinite(st.residual_mm) or not 0 <= st.residual_mm <= 2:
            out.append("station: finite fit residual of at most 2 mm required")
    miss = profile.grip.missing()
    if miss:
        out.append("grip thresholds not measured: " + ", ".join(miss))
    out.extend("grip: " + problem for problem in profile.grip.problems())
    rep = parts_report if parts_report is not None else verify_parts(profile.parts_path())
    if not parts_ok(rep):
        bad = [k for k, v in rep.items() if not v["ok"]]
        out.append("parts: not the R2a release: " + ", ".join(bad))
    return out


def assert_execution_allowed(profile: AssemblyProfile, **kw) -> None:
    b = execution_blockers(profile, **kw)
    if b:
        raise ExecutionDisabled("R2a execution refused: " + " | ".join(b))
