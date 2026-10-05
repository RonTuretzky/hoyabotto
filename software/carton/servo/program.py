"""Run a commissioned carton sequence in one local process and motor session.

There are no model calls, automatic calibration probes, blind joint replays,
lease renewals, or fault retries here. Geometry and image targets must come
from station commissioning. This runner retains the existing local envelope;
it cannot turn an alignment model into a whole-workspace motion planner.
"""
from __future__ import annotations

import copy
import time
from pathlib import Path

import numpy as np

from .common import Refused, Trace, atomic_json, binding, finite, read_json, validate_config, vector
from .controller import Experiment
from .transport import SessionTransport
from .vision import Observer


FLAPS = ("short_left", "short_right", "far_long", "near_long")


def template(config_path):
    """Uncommissioned, intentionally non-executable; no invented joint poses."""
    return {"schema": 1, "config": str(Path(config_path).resolve()), "model": None,
            "minimum_lease_s": 120, "approach": [None],
            "gripper": {"open_ticks": None, "empty_closed_ticks": None,
                        "goal_ticks": None, "min_aperture_ticks": None},
            "lift": {"targets": [None], "tool": "tool", "object": "target",
                     "bottom": "paddle_bottom", "table": "table_edge",
                     "up_normal": None, "min_motion_px": 8, "max_slip_px": 3,
                     "min_clearance_px": None},
            "folds": [{"name": name, "targets": [None], "retract": [None],
                       "verification": {"seconds": 1.0, "constraints": [None]}}
                      for name in FLAPS]}


def load_recipe(path):
    from .cli import load_config
    path = Path(path).resolve()
    recipe = read_json(path)
    config = load_config(path.parent / recipe["config"])
    model_path = recipe.get("model")
    model = read_json(path.parent / model_path) if model_path else None
    if recipe.get("depth") is not None:
        for key in ("manifest", "reference"):
            recipe["depth"][key] = str((path.parent / recipe["depth"][key]).resolve())
    return recipe, config, model


def preflight(recipe, config, model):
    """Collect commissioning problems before connecting to an active owner."""
    problems = []

    def check(label, fn):
        try:
            fn()
        except (Refused, KeyError, TypeError, ValueError, IndexError) as exc:
            problems.append(f"{label}: {exc}")

    def require(ok, message):
        if not ok:
            raise Refused(message)

    check("schema", lambda: require(recipe.get("schema") == 1, "expected schema 1"))
    check("config", lambda: validate_config(config))
    limits = validate_config(config) if not problems else None
    check("minimum_lease_s", lambda: require(
        0 < finite(recipe.get("minimum_lease_s")) <= (limits.max_seconds if limits else 300),
        "must fit the total program time budget"))

    def check_model():
        require(limits is not None, "repair the experiment configuration first")
        require(isinstance(model, dict), "supply the measured, validated local model")
        require(model.get("fingerprint") == binding(config), "model does not match this configuration")
        # Validate the full sample/holdout structure without reading a motor or camera.
        # Live session, age, scene and stream checks run again immediately before dispatch.
        from types import SimpleNamespace
        e = Experiment(config, SimpleNamespace(limits=limits, started=model["motor_session_started"]),
                       None, None, binding(config), clock=lambda: model["created_at"])
        e.validate_model(model, SimpleNamespace(values=vector(model["origin_features"]),
                                                streams=model["camera_streams"]), model["origin"])

    check("model", check_model)

    def targets(label, values):
        require(isinstance(values, list) and 0 < len(values) <= 40, f"{label} needs measured image targets")
        for value in values:
            target = vector(value, len(config["measurements"]))
            if isinstance(model, dict) and limits:
                j = np.asarray(model["jacobian"], float)
                dq = np.linalg.lstsq(j, target-vector(model["origin_features"]), rcond=None)[0]
                require(max(abs(dq)) <= min(limits.trust_ticks, model["limits"]["trust_ticks"]),
                        f"{label} exceeds the local model neighborhood")
                require(np.max(np.abs(j @ dq + vector(model["origin_features"]) - target)) <= limits.tolerance_px,
                        f"{label} is not reachable by the measured local model")
                for name, delta in zip(config["joints"], dq):
                    lo, hi = config["ranges"][name]
                    require(lo+4 <= model["origin"][name]+delta <= hi-4, f"{label} exceeds saved motor range")

    check("approach", lambda: targets("approach", recipe.get("approach")))
    check("lift.targets", lambda: targets("lift", recipe["lift"]["targets"]))

    def grip():
        g = recipe["gripper"]
        opened, empty, goal, aperture = [finite(g[k], k) for k in
                                        ("open_ticks", "empty_closed_ticks", "goal_ticks", "min_aperture_ticks")]
        require(all(type(g[k]) is int for k in ("open_ticks", "empty_closed_ticks", "goal_ticks")),
                "jaw positions must be integer encoder ticks")
        lo, hi = config["ranges"][f'{config["arm"]}_arm_gripper']
        require(lo+4 <= min(opened, empty, goal) and max(opened, empty, goal) <= hi-4, "jaw baseline outside saved range")
        require(min(opened, empty) < goal < max(opened, empty), "jaw goal must be between measured open and empty closure")
        require(0 < aperture <= abs(goal-empty), "jaw goal must leave a measurable nonempty aperture")
        if limits:
            require(abs(goal-opened) <= limits.trust_ticks, "jaw closure exceeds local travel envelope")
    check("gripper", grip)

    def lift():
        spec = recipe["lift"]
        require(abs(np.linalg.norm(vector(spec["up_normal"], 2))-1) < .001,
                "supply a measured unit image direction away from the tabletop")
        require(finite(spec["min_motion_px"]) >= 8 and 0 < finite(spec["max_slip_px"]) <= 3,
                "lift must be at least 8px with no more than 3px relative slip")
        require(finite(spec["min_clearance_px"]) >= 8, "supply measured paddle-bottom clearance of at least 8px")
        require(len({spec[k] for k in ("tool", "object", "bottom", "table")}) == 4,
                "tool, object, bottom and stationary table must be independent tracked features")
        for key in ("tool", "object", "bottom", "table"):
            require(spec[key] in config["cameras"]["head"]["regions"], f"seed head feature {spec[key]}")
        require(config["cameras"]["head"]["regions"][spec["table"]].get("anchor") is True,
                "table reference must be a stationary anchor")
        for key in ("tool", "object"):
            require(spec[key] in config["cameras"][f'{config["arm"]}_wrist']["regions"], f"seed wrist feature {spec[key]}")
    check("lift.evidence", lift)

    folds = recipe.get("folds", [])
    check("fold order", lambda: require([f["name"] for f in folds] == list(FLAPS), "commission all four flaps in order"))
    for index, fold in enumerate(folds):
        name = fold.get("name", str(index))
        check(name+".targets", lambda f=fold: targets(name, f["targets"]))
        check(name+".retract", lambda f=fold: targets(name+" retract", f["retract"]))
        check(name+".verification", lambda f=fold: validate_gate(f["verification"], config))
        def gate_roles(f=fold):
            for constraint in f["verification"]["constraints"]:
                if constraint["role"] == "flap":
                    require(constraint["a"] == f["name"], "flap evidence must track this flap's named feature")
                    require(config["cameras"][constraint["camera"]]["regions"][constraint["b"]].get("anchor") is True,
                            "flap outcome must be relative to a stationary reference")
                else:
                    require(constraint["a"] == recipe["lift"]["tool"] and constraint["b"] == f["name"],
                            "clearance must compare the tool and this flap")
        check(name+".evidence identity", gate_roles)
    # Control features must describe the arm against the station, not a paddle
    # or flap that starts moving at contact. Otherwise the old Jacobian lies.
    def stationary_model():
        moving = {recipe["lift"][k] for k in ("object", "bottom")} | set(FLAPS)
        for m in config["measurements"]:
            require(m["a"] not in moving and m.get("b") not in moving,
                    "control model must exclude the paddle; track it independently for grasp evidence")
    check("control features", stationary_model)
    if recipe.get("depth") is not None:
        from .depth import validate_depth_spec
        check("depth", lambda: validate_depth_spec(recipe["depth"]))
    return {"status": "PROGRAM_STATIC_CHECKS_PASSED" if not problems else "PROGRAM_NOT_READY",
            "problems": problems, "motor_writes": 0, "physical_task_completed": False,
            "requires_live_checks": True}


def validate_gate(gate, config):
    if not .5 <= finite(gate["seconds"]) <= 5:
        raise Refused("Verify the released flap for 0.5–5 seconds")
    constraints = gate["constraints"]
    if not isinstance(constraints, list) or not constraints:
        raise Refused("Supply independent flap and tool-clearance measurements")
    roles = set()
    for c in constraints:
        roles.add(c["role"])
        if c["role"] not in ("flap", "tool_clearance"):
            raise Refused("Gate role must be flap or tool_clearance")
        regions = config["cameras"][c["camera"]]["regions"]
        if c["a"] == c["b"] or c["a"] not in regions or c["b"] not in regions:
            raise Refused("Gate needs two distinct seeded features")
        if abs(np.linalg.norm(vector(c["direction"], 2))-1) > .001:
            raise Refused("Gate direction must be a unit image vector")
        lo, hi = vector(c["interval_px"], 2)
        if lo >= hi:
            raise Refused("Gate interval must have positive width")
        if c["role"] == "flap" and hi-lo > 8:
            raise Refused("Flap acceptance interval cannot exceed 8px")
        if c["role"] == "tool_clearance" and lo < 8:
            raise Refused("Retracted tool must have at least 8px measured clearance")
    if roles != {"flap", "tool_clearance"}:
        raise Refused("Need both flap outcome and retracted-tool clearance checks")


def gate_values(observation, gate):
    values = []
    for c in gate["constraints"]:
        points = observation.points[c["camera"]]
        value = float((vector(points[c["a"]], 2)-vector(points[c["b"]], 2)) @ vector(c["direction"], 2))
        lo, hi = c["interval_px"]
        if not lo <= value <= hi:
            raise Refused(f'{c["role"]} evidence failed: {value:.2f}px outside [{lo}, {hi}]')
        values.append(value)
    return values


def grasp_evidence(before, after, spec, arm, jaw_ticks, gripper):
    head0, head1 = before.points["head"], after.points["head"]
    normal = vector(spec["up_normal"], 2)
    tool_motion = vector(head1[spec["tool"]], 2)-vector(head0[spec["tool"]], 2)
    object_motion = vector(head1[spec["object"]], 2)-vector(head0[spec["object"]], 2)
    clearance = float((vector(head1[spec["bottom"]], 2)-vector(head1[spec["table"]], 2)) @ normal)
    wrist = f"{arm}_wrist"
    relative = lambda obs: vector(obs.points[wrist][spec["object"]], 2)-vector(obs.points[wrist][spec["tool"]], 2)
    if (min(tool_motion @ normal, object_motion @ normal) < spec["min_motion_px"]
            or np.linalg.norm(tool_motion-object_motion) > spec["max_slip_px"]
            or np.linalg.norm(relative(after)-relative(before)) > spec["max_slip_px"]
            or clearance < spec["min_clearance_px"]
            or abs(jaw_ticks-gripper["empty_closed_ticks"]) < gripper["min_aperture_ticks"]):
        raise Refused("Lift not verified: require upward paddle co-motion, table clearance and nonempty jaw aperture")
    return {"head_tool_motion_px": tool_motion.tolist(), "head_object_motion_px": object_motion.tolist(),
            "bottom_clearance_px": clearance, "wrist_relative_px": relative(after).tolist()}


class Program:
    def __init__(self, recipe, experiment, depth_observer=None):
        self.recipe, self.e = recipe, experiment
        self.transport, self.trace = experiment.transport, experiment.trace
        self.arm = experiment.config["arm"]
        self.gripper = f"{self.arm}_arm_gripper"
        self.held_relative = None
        self.folded = []
        self.stage = "preflight"
        self.depth_observer, self.latest_depth = depth_observer, None

    def _observe(self, after=0):
        return self.e.observe(after)

    def _align(self, targets, model):
        for target in targets:
            self.e.align(model, target=target)

    def _progress(self, stage):
        self.stage = stage
        self.trace.write("program_stage", stage=stage)
        atomic_json(self.trace.folder / "progress.json", {"stage": stage, "time": self.e.clock(),
                    "verified_flaps": self.folded, "path_ticks": self.transport.path_ticks})

    def run(self, model):
        ready = preflight(self.recipe, self.e.config, model)
        if ready["problems"]:
            raise Refused("; ".join(ready["problems"]))
        if self.recipe.get("depth") is not None and self.depth_observer is None:
            raise Refused("Recipe requires the OAK depth observer")
        status, q = self.transport.status()
        generation = status.get("gripper_release_generation")
        if (type(generation) is not int or generation < 0
                or status.get("automatic_gripper_reenable") is not False):
            raise Refused("Owner must expose gripper release generation and disable automatic jaw re-enable for grasp execution")
        if status["lease_remaining"] < self.recipe["minimum_lease_s"]:
            raise Refused("Insufficient owner lease for this program; prepare everything before starting the owner")
        if abs(q[self.gripper]-self.recipe["gripper"]["open_ticks"]) > self.e.limits.settle_ticks:
            raise Refused("Gripper does not match the measured open start; no automatic recovery")
        # Install retention monitoring in the observer, so it runs on every
        # correction as well as at phase boundaries, not just after a long path.
        observer = self.e.observer
        program = self

        class MonitoredObserver:
            def observe(self, **kwargs):
                obs = observer.observe(**kwargs)
                current, _ = program.transport.status()
                if (current.get("gripper_release_generation") != generation
                        or current.get("automatic_gripper_reenable") is not False):
                    raise Refused("Claw torque/recovery state changed; previous grasp evidence is invalid")
                if program.depth_observer is not None:
                    program.latest_depth = program.depth_observer.observe(obs.captured_at)
                    program.trace.write("depth_observation", **program.latest_depth)
                if program.held_relative is not None:
                    spec = program.recipe["lift"]
                    p = obs.points[f"{program.arm}_wrist"]
                    relative = vector(p[spec["object"]], 2)-vector(p[spec["tool"]], 2)
                    if np.linalg.norm(relative-program.held_relative) > spec["max_slip_px"]:
                        raise Refused("Paddle moved relative to the jaw; retention lost")
                return obs

            def evidence(self, folder, obs):
                if hasattr(observer, "evidence"):
                    observer.evidence(folder, obs)

        self.e.observer = MonitoredObserver()
        try:
            self._progress("approach")
            self._align(self.recipe["approach"], model)
            self._progress("close_gripper")
            goal = self.recipe["gripper"]["goal_ticks"]
            for _ in range(self.e.limits.max_steps):
                obs, q = self._observe()
                remaining = int(round(goal-q[self.gripper]))
                if abs(remaining) <= self.e.limits.settle_ticks:
                    break
                # A fresh camera/position pair is required before every jaw step.
                if not 0 <= self.e.clock()-obs.captured_at <= self.e.limits.frame_age_s:
                    raise Refused("Camera became stale before jaw movement")
                _, stamp = self.transport.move_gripper(int(np.clip(remaining, -self.e.limits.step_ticks, self.e.limits.step_ticks)))
                self._observe(after=stamp)
            else:
                raise Refused("Jaw closure step budget exhausted")
            before, _ = self._observe()
            before_depth = copy.deepcopy(self.latest_depth)
            self._progress("test_lift")
            self._align(self.recipe["lift"]["targets"], model)
            # Three independent post-lift observations prevent a one-frame success.
            for _ in range(3):
                after, q = self._observe()
                evidence = grasp_evidence(before, after, self.recipe["lift"], self.arm,
                                           q[self.gripper], self.recipe["gripper"])
                self.trace.write("grasp_observation", **evidence)
                if self.depth_observer is not None:
                    from .depth import verify_depth_lift
                    self.trace.write("depth_grasp_observation", **verify_depth_lift(
                        before_depth, self.latest_depth, self.recipe["depth"]))
            self.held_relative = vector(evidence["wrist_relative_px"], 2)
            for fold in self.recipe["folds"]:
                self._progress("fold_"+fold["name"])
                self._align(fold["targets"], model)
                self._progress("retract_"+fold["name"])
                self._align(fold["retract"], model)
                self._progress("verify_"+fold["name"])
                started, count = None, 0
                while True:
                    obs, _ = self._observe()
                    values = gate_values(obs, fold["verification"])
                    started = obs.captured_at if started is None else started
                    count += 1
                    self.trace.write("fold_observation", flap=fold["name"], values_px=values, captured_at=obs.captured_at)
                    if count >= 3 and obs.captured_at-started >= fold["verification"]["seconds"]:
                        break
                self.folded.append(fold["name"])
            # Folding a later flap can reopen an earlier one. Check all four
            # together in the final retracted pose rather than trusting history.
            self._progress("verify_all_flaps")
            self.folded = []
            first = None
            count = 0
            while True:
                obs, _ = self._observe()
                for fold in self.recipe["folds"]:
                    gate_values(obs, fold["verification"])
                first = obs.captured_at if first is None else first
                count += 1
                if count >= 3 and obs.captured_at-first >= max(f["verification"]["seconds"] for f in self.recipe["folds"]):
                    break
            self.folded = list(FLAPS)
            self._progress("visual_checks_passed")
            return {"status": "CARTON_VISUAL_CHECKS_PASSED", "verified_flaps": self.folded,
                    "grasp_visual_evidence": True, "physical_task_completed": False,
                    "limitation": "Image checks do not establish a durable closed carton or tape seal.",
                    "model_calls": 0, "path_ticks": self.transport.path_ticks,
                    "owner_session_started": self.transport.started}
        finally:
            self.e.observer = observer


def execute(recipe, config, model, out, *, execute=False):
    ready = preflight(recipe, config, model)
    if ready["problems"] or not execute:
        return ready
    trace = Trace(Path(out).resolve())
    transport = SessionTransport(config, validate_config(config), execute=True)
    program = None
    started = time.monotonic()
    try:
        with transport:
            observer = Observer(config, transport.limits)
            e = Experiment(config, transport, observer, trace, binding(config))
            depth_observer = None
            if recipe.get("depth") is not None:
                from .depth import DepthObserver
                depth_observer = DepthObserver(recipe["depth"])
            program = Program(copy.deepcopy(recipe), e, depth_observer)
            trace.write("program_start", recipe=recipe, fingerprint=binding(config))
            result = program.run(model)
    except BaseException as exc:
        # Context manager requests the owner's existing STOP policy on failure.
        # Never retry an unreliable temperature read or re-energize a stopped arm.
        result = {"status": "PROGRAM_STOPPED", "reason": str(exc),
                  "stage": program.stage if program else "preflight", "physical_task_completed": False,
                  "verified_flaps": program.folded if program else [], "path_ticks": transport.path_ticks}
        if not isinstance(exc, Exception):
            atomic_json(trace.folder / "result.json", result)
            trace.close()
            raise
    result["elapsed_s"] = time.monotonic()-started
    result["owner_release_requested"] = transport.aborted
    atomic_json(trace.folder / "result.json", result)
    trace.write("program_finish", result=result)
    trace.close()
    return result
