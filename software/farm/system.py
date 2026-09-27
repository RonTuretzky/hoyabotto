"""Build the whole farm from a profile: real devices or simulator fakes behind the
same interfaces, one evidence store, one human channel, the LLM backends, the
authority ladder and the skill runner."""
from __future__ import annotations

import logging
import subprocess
import time
from pathlib import Path
from typing import Any

import yaml

from .adapters.human_web import WebHuman
from .config import Profile, load_env, load_profile
from .cycle.authority import Authority
from .evidence.store import EvidenceStore
from .llm.backends import Backends
from .skills.keyframes import KeyframeStore
from .skills.runner import SkillRunner

log = logging.getLogger(__name__)


def code_version() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=Path(__file__).parent, timeout=3).stdout.strip() or "dev"
    except Exception:  # noqa: BLE001
        return "dev"


class System:
    def __init__(self, profile: Profile, human=None, faults=None, backends=None):
        load_env()
        self.profile = self._with_overrides(profile)
        self.store = EvidenceStore(self.profile.data_path, code_version())
        self.state: dict[str, Any] = {"profile": self.profile.name, "simulated": self.profile.simulated, "started": time.time()}
        self.human = human or self._default_human()
        self.backends = backends or Backends(self.profile.llm, self.store)
        self.authority = Authority(self.profile.authority, self.store)
        self.faults = faults
        self.keyframes = KeyframeStore(self.profile.data_path / self.profile.keyframes_file)
        self.recorder = None
        self.robot = None
        self.cameras: dict[str, Any] = {}
        self.light = None
        self.skills: SkillRunner | None = None
        self._build_devices()

    def _default_human(self):
        """Viewer always; Telegram too when a bot token and chat ids are configured. First answer wins."""
        import os
        web = WebHuman(self.profile.authority.notify_webhook)
        if os.environ.get("TELEGRAM_BOT_TOKEN") and os.environ.get("TELEGRAM_CHAT_IDS"):
            try:
                from .adapters.human_multi import MultiHuman
                from .adapters.human_telegram import TelegramHuman
                tg = TelegramHuman(image_dir=self.store.root / "images")
                log.info("Telegram channel enabled")
                return MultiHuman(web, tg)
            except Exception as e:  # noqa: BLE001
                log.warning("Telegram channel unavailable: %s", e)
        return web

    def enable_recording(self, repo_id: str = "farm/own-runs", fps: int = 10) -> None:
        """Record the robot's own runs (joints, targets, frames) into a LeRobotDataset under data/dataset."""
        from .adapters.base import ARM_JOINTS, HEAD_JOINTS, arm_joint
        from .learning.recorder import EpisodeRecorder
        joints = [arm_joint(a, j) for a in ("left", "right") for j in ARM_JOINTS] + HEAD_JOINTS
        cams = [c.name for c in self.profile.cameras]
        self.recorder = EpisodeRecorder(self.profile.data_path / "dataset", repo_id, fps, cams, (self.profile.cameras[0].height, self.profile.cameras[0].width), joints)
        self._rec_next = 0.0

        def on_tick(cur):
            import time as _t
            if _t.time() < self._rec_next or not getattr(self.recorder, "recording", True):
                return
            self._rec_next = _t.time() + 1.0 / fps
            frames = {}
            for n, cam in self.cameras.items():
                r = cam.frame()
                if r.ok:
                    frames[n] = r.value
            self.recorder.tick(cur, dict(getattr(self.skills, "last_sent", {}) or {}), frames)
        self.skills.on_tick = on_tick

    @staticmethod
    def _with_overrides(profile: Profile) -> Profile:
        """Astra's apply-safe changes live in data/overrides.yaml and overlay the profile."""
        ov = profile.data_path / "overrides.yaml"
        if ov.exists():
            o = yaml.safe_load(ov.read_text()) or {}
            for sect, vals in o.items():
                target = getattr(profile, sect, None)
                if target is not None and isinstance(vals, dict):
                    for k, v in vals.items():
                        if hasattr(target, k):
                            setattr(target, k, v)
        return profile

    def _build_devices(self) -> None:
        p = self.profile
        if p.robot.kind == "sim" or p.simulated:
            from .adapters.sim import FakeCamera, FakeLight, FakeRobot, Faults
            self.faults = self.faults or Faults()
            self.robot = FakeRobot(self.faults)
            self.cameras = {c.name: FakeCamera(c.name, self.faults) for c in p.cameras}
            self.light = FakeLight(self.faults, enabled=p.light.enabled)
        else:
            from .adapters.camera_opencv import OpenCVCameraAdapter
            from .adapters.esp32_serial import ESP32Light
            from .adapters.robot_lerobot import LeRobotXLeRobot
            self.robot = LeRobotXLeRobot(p.robot)
            self.cameras = {c.name: OpenCVCameraAdapter(c) for c in p.cameras}
            self.light = ESP32Light(p.light) if p.light.enabled else None
        self.skills = SkillRunner(self.robot, p.limits, p.arms, self.keyframes)
        if p.llm.backend == "sim":
            from .adapters.sim import SimVision
            self.backends.vision = SimVision(self.faults)

    # ---- lifecycle -------------------------------------------------------------
    def connect(self) -> list[str]:
        """Connect every enabled device. Returns problems (a missing optional device is a problem, not a crash)."""
        problems = []
        unknown = self.store.mark_unknown_after_crash()
        if unknown:
            problems.append(f"{len(unknown)} action(s) were ATTEMPTED before a restart and are now UNKNOWN: reconcile in the viewer")
            self.state["needs_person"] = True
        try:
            self.robot.connect()
        except Exception as e:  # noqa: BLE001
            problems.append(f"robot: {e}")
        for name, cam in self.cameras.items():
            try:
                cam.connect()
            except Exception as e:  # noqa: BLE001
                problems.append(f"camera {name}: {e}")
        if self.light is not None:
            try:
                self.light.connect()
            except Exception as e:  # noqa: BLE001
                problems.append(f"light: {e}")
        self.store.event(None, "startup", {"problems": problems, "profile": self.profile.name, "config_hash": self.profile.config_hash})
        self.state["problems"] = problems
        return problems

    def disconnect(self) -> None:
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception:  # noqa: BLE001
                pass
        if self.light is not None:
            try:
                self.light.disconnect()
            except Exception:  # noqa: BLE001
                pass
        try:
            self.robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self.store.event(None, "shutdown", {})

    # ---- checks --------------------------------------------------------------------
    def verify_views(self) -> dict[str, Any]:
        """Ask the vision model which camera each frame comes from; a mismatch blocks operation."""
        from .llm.backends import NoLLM
        from .perception.vlm import VLMPerception
        if isinstance(self.backends.vision, NoLLM):
            return {"status": "SKIPPED", "note": "no vision backend"}
        frames = []
        for n, cam in self.cameras.items():
            r = cam.frame()
            if r.ok:
                frames.append((n, r.value))
        if not frames:
            return {"status": "INVALID", "note": "no frames"}
        v = VLMPerception(self.backends.vision, self.store).check_views(frames)
        self.store.observation(None, "views.check", v.status.value, v.value, note=v.note)
        return {"status": v.status.value, "views": v.value, "note": v.note}

    def ready_for_cycle(self) -> bool:
        """Robot bus alive and not stopped; try one reconnect if the bus dropped (USB hiccup)."""
        from .status import Status
        if self.skills.estop.is_set():
            return False
        j = self.robot.joints()
        if j.status is Status.OK:
            return True
        log.warning("robot joints %s (%s); attempting reconnect", j.status.value, j.note)
        try:
            self.robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.robot.connect()
            self.store.event(None, "reconnected", {"device": "robot"})
            return self.robot.joints().status is Status.OK
        except Exception as e:  # noqa: BLE001
            self.store.event(None, "reconnect_failed", {"device": "robot", "error": str(e)})
            self.state["problems"] = [f"robot: {e}"]
            return False

    def idle_rest(self, seconds: float) -> None:
        """Between cycles: rest the arms, release torque so servos cool, watch temperature, then re-engage."""
        from .safety.rules import check_health
        try:
            self.skills.go_rest()
        except Exception as e:  # noqa: BLE001
            log.warning("go_rest before idle failed: %s", e)
        try:
            self.robot.torque_off()
            self.state["idle"] = True
            self.store.event(None, "idle", {"seconds": seconds})
        except Exception as e:  # noqa: BLE001
            log.warning("torque_off failed: %s", e)
        end = time.time() + seconds
        while time.time() < end:
            h = self.robot.health()
            v = check_health(h, self.profile.limits)
            self.state["health"] = h.value if h.ok else h.status.value
            if not v.ok:
                self.store.event(None, "health_warning", {"reason": v.reason})
            time.sleep(min(30.0, max(1.0, end - time.time())))
        try:
            if hasattr(self.robot, "torque_on"):
                self.robot.torque_on()
        except Exception as e:  # noqa: BLE001
            log.warning("torque_on failed: %s", e)
        self.state["idle"] = False

    def pour_calibration(self) -> dict[str, float]:
        p = self.profile.data_path / "pour_calibration.yaml"
        if p.exists():
            return yaml.safe_load(p.read_text()) or {}
        return {}

    def save_pour_calibration(self, tilt_deg: float, seconds: float, ml: float, who: str) -> None:
        p = self.profile.data_path / "pour_calibration.yaml"
        p.write_text(yaml.safe_dump({"tilt_deg": tilt_deg, "seconds": seconds, "measured_ml": ml, "who": who, "t": time.time()}))
        self.store.event(None, "pour_calibration", {"tilt_deg": tilt_deg, "seconds": seconds, "measured_ml": ml, "who": who})


def build(profile_name: str, **kw) -> System:
    return System(load_profile(profile_name), **kw)
