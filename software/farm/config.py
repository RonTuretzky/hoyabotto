"""Profile loading. One YAML names what exists; code enforces what may move."""
from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


@dataclass
class RobotCfg:
    kind: str = "lerobot"                # lerobot | sim
    id: str = "farm_xlerobot"
    port1: str = ""                      # bus 1: left arm + head
    port2: str = ""                      # bus 2: right arm + wheels
    max_relative_target: float = 8.0     # LeRobot clamp, per tick, in normalized units
    wheels: bool = False                 # base driving is off in V0
    calibration_dir: str = ""            # defaults to ~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels


@dataclass
class CameraCfg:
    name: str
    match: str = ""                      # substring of the OpenCV device name/path; identity, not index
    index_or_path: str | int | None = None  # explicit override (e.g. /dev/video2 or an int on macOS)
    width: int = 640
    height: int = 480
    fps: int = 30
    max_age_s: float = 1.0


@dataclass
class LightCfg:
    enabled: bool = True
    port: str = ""                       # serial port of the ESP32; empty = auto-detect by USB VID/PID
    baud: int = 115200
    max_age_s: float = 1.0
    samples: int = 10                    # distinct conversions per measurement
    settle_s: float = 0.5


@dataclass
class TrayCfg:
    id: str
    nest: str                            # human name of the fixed nest position
    kind: str = "cress-wick"             # cress-wick | square-kit | household-dish
    tag_id: int | None = None            # optional AprilTag id printed on the nest
    look_pose: str = ""                  # keyframe name for look_at(tray)
    measure_pose: str = ""               # keyframe name for measure_pose(tray)
    pour_pose: str = ""                  # keyframe name for pour(tray)
    approach_pose: str = ""
    refill_ml_target: float = 30.0


@dataclass
class ArmsCfg:
    bottle: str = "right"
    paddle: str = "left"


@dataclass
class LimitsCfg:
    pour_tilt_max_deg: float = 40.0
    pour_s_max: float = 4.0
    servo_temp_max_c: float = 55.0
    servo_load_max: int = 800            # raw Present_Load ceiling (STS3215 scale 0..1000)
    watchdog_s: float = 0.5
    step_deg_max: float = 6.0            # per-tick joint step for LLM-servo moves
    llm_servo_max_steps: int = 40


@dataclass
class DeadlinesCfg:
    identify_s: float = 10
    inspect_s: float = 20
    measure_s: float = 15
    ask_s: float = 600
    pick_s: float = 30
    approach_s: float = 30
    pour_s: float = 8
    verify_s: float = 20
    park_s: float = 30


@dataclass
class LLMCfg:
    backend: str = "claude-cli"          # claude-cli (subscription) | openrouter
    vision_model: str = "anthropic/claude-sonnet-5"   # openrouter model for perception/servo
    jev_model: str = "typesafe/jev-router"
    astra_model: str = "openai/gpt-6-astra"
    claude_cli_model: str = ""           # empty = CLI default
    max_cost_usd_per_day: float = 5.0
    timeout_s: float = 120


@dataclass
class AuthorityCfg:
    """How much decision power the models hold. Rules always win; a person can always override."""
    jev: str = "route"                   # shadow | route | approve
    astra: str = "propose"               # shadow | propose | apply-safe
    jev_approve_min_p: float = 0.85      # min probability for Jev to approve a routine pour
    jev_shadow_cycles: int = 10          # cycles logged before authority is honoured
    notify_webhook: str = ""             # optional chat webhook for questions/notifications


@dataclass
class PolicyCfg:
    """A learned policy for one skill. state_joints are our joint names in the order the policy expects;
    camera_map maps the policy's image keys to our camera names."""
    enabled: bool = False
    skill: str = "pour"                       # which skill it replaces when enabled and healthy
    checkpoint: str = ""                      # local dir or HF repo id
    device: str = "mps"
    state_joints: list[str] = field(default_factory=lambda: [f"right_arm_{j}" for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")])
    camera_map: dict[str, str] = field(default_factory=lambda: {"observation.images.wrist": "right_wrist", "observation.images.front": "head"})
    hz: float = 10.0
    max_steps: int = 300
    shadow: bool = True                       # log what the policy would do; keyframes still execute


@dataclass
class Profile:
    name: str
    robot: RobotCfg
    cameras: list[CameraCfg]
    light: LightCfg
    trays: list[TrayCfg]
    arms: ArmsCfg
    limits: LimitsCfg
    deadlines: DeadlinesCfg
    llm: LLMCfg
    authority: AuthorityCfg
    policy: PolicyCfg = field(default_factory=PolicyCfg)
    data_dir: str = "data"
    viewer_port: int = 8765
    simulated: bool = False
    keyframes_file: str = "keyframes.yaml"
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def config_hash(self) -> str:
        return hashlib.sha256(yaml.safe_dump(self.raw, sort_keys=True).encode()).hexdigest()[:12]

    def tray(self, tray_id: str) -> TrayCfg:
        for t in self.trays:
            if t.id == tray_id:
                return t
        raise KeyError(tray_id)

    @property
    def data_path(self) -> Path:
        p = Path(self.data_dir)
        if not p.is_absolute():
            p = PROFILES_DIR.parent / p
        p.mkdir(parents=True, exist_ok=True)
        return p


def _build(cls, d: dict[str, Any] | None):
    d = dict(d or {})
    known = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
    return cls(**known)


def load_profile(name_or_path: str) -> Profile:
    path = Path(name_or_path)
    if not path.exists():
        path = PROFILES_DIR / f"{name_or_path}.yaml"
    raw = yaml.safe_load(path.read_text()) or {}
    return Profile(
        name=raw.get("profile", path.stem),
        robot=_build(RobotCfg, raw.get("robot")),
        cameras=[_build(CameraCfg, {"name": k, **(v or {})}) for k, v in (raw.get("cameras") or {}).items()],
        light=_build(LightCfg, raw.get("light")),
        trays=[_build(TrayCfg, {"id": k, **(v or {})}) for k, v in (raw.get("trays") or {}).items()],
        arms=_build(ArmsCfg, raw.get("arms")),
        limits=_build(LimitsCfg, raw.get("limits")),
        deadlines=_build(DeadlinesCfg, raw.get("deadlines")),
        llm=_build(LLMCfg, raw.get("llm")),
        authority=_build(AuthorityCfg, raw.get("authority")),
        policy=_build(PolicyCfg, raw.get("policy")),
        data_dir=raw.get("data_dir", "data"),
        viewer_port=int(raw.get("viewer_port", 8765)),
        simulated=bool(raw.get("simulated", False)),
        keyframes_file=raw.get("keyframes_file", "keyframes.yaml"),
        raw=raw,
    )


def load_env(path: Path | None = None) -> None:
    """Load KEY=VALUE lines from software/.env into os.environ (never committed)."""
    p = path or PROFILES_DIR.parent / ".env"
    if not p.exists():
        return
    for line in p.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip())
