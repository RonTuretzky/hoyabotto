"""Identify a local image Jacobian, validate it, then reduce measured error.

No trained policy, analytical IK, language-model action or open-loop replay is
used. A Jacobian describes this local pose/scene only, not the whole workspace.
"""
from __future__ import annotations

import time
from dataclasses import asdict

import numpy as np

from .common import Limits, Refused, vector


def fit_model(samples, joints, limits: Limits):
    x = np.asarray([s["dq"] for s in samples], float)
    y = np.asarray([s["dy"] for s in samples], float)
    if x.ndim != 2 or y.ndim != 2 or len(x) != len(y) or x.shape[1] != len(joints):
        raise Refused("Invalid calibration sample dimensions")
    if not np.isfinite(x).all() or not np.isfinite(y).all() or len(x) < 2 * len(joints):
        raise Refused("Insufficient finite calibration samples")
    if np.linalg.matrix_rank(x) != len(joints):
        raise Refused("Calibration did not excite every selected joint")
    jacobian = np.linalg.lstsq(x, y, rcond=None)[0].T
    singular = np.linalg.svd(jacobian, compute_uv=False)
    if len(singular) < len(joints) or singular[-1] <= 1e-6:
        raise Refused("Visual features cannot distinguish the selected joint motions")
    condition = float(singular[0] / singular[-1])
    if condition > limits.condition_max:
        raise Refused(f"Poor visual observability: condition number {condition:.1f}")
    responses = np.linalg.norm(jacobian, axis=0) * limits.probe_ticks
    if min(responses) < limits.min_response_px:
        raise Refused("At least one joint produces too little visible movement")
    residual = float(np.max(np.linalg.norm(y - x @ jacobian.T, axis=1)))
    if residual > limits.model_error_px:
        raise Refused(f"Calibration is inconsistent/nonlinear: residual {residual:.2f}px")
    return {"jacobian": jacobian.tolist(), "condition": condition, "fit_error_px": residual,
            "response_px": responses.tolist(), "samples": samples}


class Experiment:
    def __init__(self, config, transport, observer, trace, fingerprint, clock=time.time):
        self.config, self.transport, self.observer, self.trace = config, transport, observer, trace
        self.fingerprint, self.clock = fingerprint, clock
        self.limits = transport.limits
        self.joints = config["joints"]
        self.counter = 0
        self.latest = None

    def observe(self, after=0.0):
        obs = self.observer.observe(after=after)
        q = self.transport.positions()
        self.latest = (obs, q)
        self.counter += 1
        self.trace.write("observation", index=self.counter, features=obs.values.tolist(),
                         captured_at=obs.captured_at, sequences=obs.sequences, streams=obs.streams, joints=q, points=obs.points)
        if hasattr(self.observer, "evidence"):
            self.observer.evidence(self.trace.folder / f"frame-{self.counter:04d}", obs)
        return obs, q

    def move(self, joint, ticks):
        if self.latest is None or not 0 <= self.clock()-self.latest[0].captured_at <= self.limits.frame_age_s:
            raise Refused("Observation became stale before command dispatch")
        current = self.transport.positions()
        if any(abs(current[n]-self.latest[1][n]) > self.limits.settle_ticks for n in current):
            raise Refused("Robot moved after the observation used to choose this command")
        self.trace.write("command", joint=joint, delta_ticks=ticks)
        started = self.clock()
        q, finished = self.transport.move(joint, ticks)
        self.trace.write("acknowledgement", joint=joint, positions=q, duration_s=self.clock()-started)
        return self.observe(after=finished)

    def return_joint(self, name, origin):
        q = self.transport.positions()
        delta = int(round(origin[name] - q[name]))
        if abs(delta) > 68:
            raise Refused("Return to calibration origin would exceed one guarded step")
        if abs(delta) > self.limits.settle_ticks:
            obs, q = self.move(name, delta)
        else:
            obs, q = self.observe()
        if abs(q[name] - origin[name]) > self.limits.settle_ticks:
            raise Refused("Calibration probe did not return to its measured origin")
        return obs, q

    def calibrate(self):
        base, origin = self.observe()
        # Three distinct image pairs expose jitter before any motor command.
        rest = [base.values]
        for _ in range(2):
            rest.append(self.observe()[0].values)
        if np.max(np.ptp(rest, axis=0)) > self.limits.return_error_px / 2:
            raise Refused("Stationary image measurements are too noisy for calibration")
        samples = []
        for joint in self.joints:
            for sign in (1, -1):
                before, q0 = self.observe()
                moved, q1 = self.move(joint, sign * self.limits.probe_ticks)
                if sign * (q1[joint] - q0[joint]) < self.limits.probe_ticks / 2:
                    raise Refused("Command acknowledged but encoder movement is insufficient/wrong-way")
                samples.append({"joint": joint, "dq": [q1[n]-q0[n] for n in self.joints],
                                "dy": (moved.values-before.values).tolist()})
                returned, _ = self.return_joint(joint, origin)
                if np.max(np.abs(returned.values-base.values)) > self.limits.return_error_px:
                    raise Refused("Scene/contact changed or movement did not reverse; calibration rejected")
        model = fit_model(samples, self.joints, self.limits)
        jacobian = np.asarray(model["jacobian"])
        # Independent half-sized moves are withheld from fitting.
        holdout = []
        probe = max(8, self.limits.probe_ticks // 2)
        for joint in self.joints:
            before, q0 = self.observe()
            moved, q1 = self.move(joint, probe)
            dq = np.array([q1[n]-q0[n] for n in self.joints])
            actual = moved.values-before.values
            error = float(np.linalg.norm(actual - jacobian @ dq))
            if abs(q1[joint]-q0[joint]) < probe / 2 or error > self.limits.model_error_px:
                raise Refused(f"Independent model validation failed for {joint}: {error:.2f}px")
            holdout.append({"joint": joint, "error_px": error, "dq": dq.tolist(), "dy": actual.tolist()})
            returned, _ = self.return_joint(joint, origin)
            if np.max(np.abs(returned.values-base.values)) > self.limits.return_error_px:
                raise Refused("Scene changed during model validation")
        result = {"schema": 1, "units": "encoder_ticks", "fingerprint": self.fingerprint,
                  "created_at": self.clock(), "motor_session_started": self.transport.started,
                  "camera_streams": base.streams,
                  "status": "LOCAL_MODEL_VALIDATED", "physical_task_completed": False,
                  "joints": self.joints, "origin": origin, "origin_features": base.values.tolist(),
                  "limits": asdict(self.limits), "holdout": holdout, **model}
        self.trace.write("calibration_validated", condition=model["condition"], holdout=holdout)
        return result

    def validate_model(self, model, observation, q):
        if model.get("schema") != 1 or model.get("units") != "encoder_ticks" or model.get("status") != "LOCAL_MODEL_VALIDATED":
            raise Refused("Model was not validated through the physical calibration workflow")
        if model.get("fingerprint") != self.fingerprint or model.get("joints") != self.joints:
            raise Refused("Camera seeds, measurements, motor calibration or joint order changed")
        if model.get("motor_session_started") != self.transport.started:
            raise Refused("Motor session changed since local model validation")
        if model.get("camera_streams") != observation.streams:
            raise Refused("Camera stream changed since local model validation")
        if not 0 <= self.clock() - model["created_at"] <= 900:
            raise Refused("Local model expired; remeasure it")
        if not model.get("holdout") or len(model["holdout"]) != len(self.joints):
            raise Refused("Independent calibration holdout evidence is missing")
        refit = fit_model(model.get("samples", []), self.joints, self.limits)
        j = np.asarray(model["jacobian"], float)
        if j.shape != (len(observation.values), len(self.joints)) or not np.isfinite(j).all():
            raise Refused("Invalid stored image Jacobian")
        if np.linalg.matrix_rank(j) < len(self.joints) or np.linalg.cond(j) > self.limits.condition_max:
            raise Refused("Stored image Jacobian has insufficient observability")
        if not np.allclose(j, np.asarray(refit["jacobian"]), atol=1e-9):
            raise Refused("Stored model no longer matches its measured samples")
        if {v["joint"] for v in model["holdout"]} != set(self.joints):
            raise Refused("Holdout must validate each selected joint independently")
        for sample in model["holdout"]:
            dq = vector(sample["dq"], len(self.joints))
            dy = vector(sample["dy"], len(observation.values))
            if (abs(dq[self.joints.index(sample["joint"])]) < max(8, self.limits.probe_ticks // 2) / 2
                    or np.linalg.norm(dy-j@dq) > self.limits.model_error_px):
                raise Refused("Stored independent holdout does not validate this model")
        dq = np.array([q[n]-model["origin"][n] for n in self.joints])
        if max(abs(dq)) > min(self.limits.trust_ticks, model["limits"]["trust_ticks"]):
            raise Refused("Current pose is outside the local model's measured neighborhood")
        if np.linalg.norm(observation.values - vector(model["origin_features"]) - j @ dq) > self.limits.model_error_px:
            raise Refused("Current scene no longer agrees with the measured local model")
        return j

    def align(self, model, target=None, shadow=False):
        obs, q = self.observe()
        j = self.validate_model(model, obs, q)
        target = vector(self.config["target"] if target is None else target, len(obs.values))
        stable = 0
        history = []
        for step in range(self.limits.max_steps + 1):
            error = obs.values-target
            norm = float(np.linalg.norm(error))
            self.trace.write("error", step=step, error_px=error.tolist(), norm_px=norm)
            # All features must agree, on three distinct camera pairs.
            if np.max(np.abs(error)) <= self.limits.tolerance_px:
                stable += 1
                if stable >= 3:
                    return {"status": "ALIGNED_ONLY", "physical_task_completed": False,
                            "grasp_verified": False, "steps": step, "error_px": error.tolist(),
                            "joints": q, "fingerprint": self.fingerprint}
                obs, q = self.observe()
                continue
            stable = 0
            if step == self.limits.max_steps:
                raise Refused("Visual control step budget exhausted")
            history.append(norm)
            if len(history) >= 9 and min(history[-4:]) >= .98 * min(history[-9:-4]):
                raise Refused("No measurable progress; recalibrate or change the grasp/fixture")
            # Coordinate descent keeps the owner's one-joint-at-a-time invariant.
            choices = []
            for k, joint in enumerate(self.joints):
                column = j[:, k]
                delta = int(np.rint(np.clip(-.7 * column.dot(error) / (column.dot(column)+1e-8),
                                            -self.limits.step_ticks, self.limits.step_ticks)))
                if not delta:
                    continue
                future = q[joint] + delta
                lo, hi = self.config["ranges"][joint]
                if not lo+4 <= future <= hi-4:
                    continue
                if abs(future-model["origin"][joint]) > min(self.limits.trust_ticks, model["limits"]["trust_ticks"]):
                    continue
                predicted = float(np.linalg.norm(error + column*delta))
                if predicted < norm:
                    choices.append((predicted, joint, delta))
            if not choices:
                raise Refused("No observable improving movement inside the local envelope")
            _, joint, delta = min(choices)
            if shadow:
                return {"status": "SHADOW_PROPOSAL", "physical_task_completed": False,
                        "joint": joint, "delta_ticks": delta, "error_px": error.tolist()}
            previous, previous_q = obs, q
            obs, q = self.move(joint, delta)
            dq = np.array([q[n]-previous_q[n] for n in self.joints])
            actual = obs.values-previous.values
            prediction_error = float(np.linalg.norm(actual-j@dq))
            self.trace.write("model_check", error_px=prediction_error, measured_change=actual.tolist())
            if abs(q[joint]-previous_q[joint]) < max(2, abs(delta)*.4):
                raise Refused("Motor did not produce the commanded displacement")
            if prediction_error > self.limits.model_error_px:
                raise Refused("Measured movement disagrees with local model; possible contact, slip or tracking loss")
        raise Refused("Unexpected controller exit")
