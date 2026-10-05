"""Client of the Mac's existing guarded session. Never opens a motor port.

The motor owner retains STOP, speed, torque, temperature, travel and lease
checks. Commands remain one joint at a time. This client adds a much smaller
total experiment envelope, strict acknowledgements, and measured settling.
"""
from __future__ import annotations

import fcntl
import hashlib
import time
from pathlib import Path

from .common import Limits, Refused, atomic_json, finite, read_json


class SessionTransport:
    def __init__(self, config, limits: Limits, execute=False, clock=time.time, sleep=time.sleep):
        self.config, self.limits, self.execute = config, limits, execute
        self.clock, self.sleep = clock, sleep
        self.folder = Path(config["session_dir"])
        self.lock = None
        self.started = None
        self.path_ticks = 0
        self.origin = None
        self.last_id = 0
        self.aborted = False
        self.commands_sent = 0
        self.deadline = clock() + limits.max_seconds

    def __enter__(self):
        # All clients of this new controller share this lock. The old manual
        # command tool must be idle; foreign command-file writes are detected.
        if self.execute:
            self.lock = (self.folder / "visual-controller.lock").open("a")
            try:
                fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                self.lock.close()
                self.lock = None
                raise Refused("Another visual-controller client owns this session") from exc
        try:
            self.status()
            existing = self.folder / "command.json"
            if existing.exists():
                self.last_id = read_json(existing).get("id")
        except BaseException:
            self.__exit__()
            raise
        return self

    def __exit__(self, exc_type=None, *_):
        try:
            # Request STOP before another client can acquire our writer lock.
            if exc_type is not None:
                self.abort()
        finally:
            if self.lock:
                fcntl.flock(self.lock, fcntl.LOCK_UN)
                self.lock.close()
                self.lock = None

    def status(self):
        if self.clock() > self.deadline:
            raise Refused("Experiment time budget exhausted")
        p = Path(self.config["calibration_file"])
        if hashlib.sha256(p.read_bytes()).hexdigest() != self.config["calibration_sha256"]:
            raise Refused("Saved motor calibration changed")
        s = read_json(self.folder / "status.json")
        if s.get("arm") != self.config["arm"] or s.get("phase") not in ("holding", "moving") or s.get("ok") is not True:
            raise Refused("No healthy active session for the selected arm")
        if not 0 <= self.clock() - finite(s.get("time"), "status time") <= self.limits.status_age_s:
            raise Refused("Motor-owner status is stale")
        if self.started is not None and s.get("started") != self.started:
            raise Refused("Motor session restarted; recalibrate the visual experiment")
        self.started = finite(s.get("started"), "session start")
        if finite(s.get("lease_remaining"), "lease remaining") <= self.limits.command_timeout_s:
            raise Refused("Insufficient supervision lease remaining for a bounded command")
        rows = s.get("rows", {})
        expected = {f'{self.config["arm"]}_arm_{j}' for j in
                    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
        if set(rows) != expected:
            raise Refused("Incomplete or unexpected motor telemetry")
        q = {}
        for name, row in rows.items():
            if finite(row.get("Status"), "motor status") != 0:
                raise Refused(f"{name}: motor fault")
            if finite(row.get("Present_Temperature"), "temperature") >= self.limits.temperature_c:
                raise Refused(f"{name}: temperature limit")
            if abs(finite(row.get("Present_Load"), "load")) >= self.limits.load_raw:
                raise Refused(f"{name}: load limit")
            q[name] = finite(row.get("Present_Position"), "encoder position")
            lo, hi = self.config["ranges"][name]
            if not lo <= q[name] <= hi:
                raise Refused(f"{name}: outside saved encoder range")
        if self.origin is None:
            self.origin = q.copy()
        self.check_envelope(q)
        return s, q

    def check_envelope(self, q):
        for name in self.config["joints"]:
            if abs(q[name] - self.origin[name]) > self.limits.trust_ticks + self.limits.settle_ticks:
                raise Refused("Actual joint position left the local experiment envelope")

    def positions(self):
        return self.status()[1]

    def move(self, joint, ticks, *, reviewed_settle_ticks=None):
        return self._move(joint,ticks,reviewed_settle_ticks=reviewed_settle_ticks,gripper=False)

    def move_gripper(self, ticks):
        """Exact jaw-position step; a stall never establishes grasp success."""
        return self._move(f'{self.config["arm"]}_arm_gripper',ticks,gripper=True)

    def _move(self, joint, ticks, *, reviewed_settle_ticks=None, gripper=False):
        # Explicit reviewed positioning only; calibrated callers retain default5.
        # The elbow alone may use the owner's24tick endpoint criterion, with
        # additional direction, minimum actual motion and stability checks.
        settle_ticks = self.limits.settle_ticks
        extended_elbow = False
        if reviewed_settle_ticks is not None:
            maximum = 24 if joint == 'right_arm_elbow_flex' else 11
            if type(reviewed_settle_ticks) is not int or not 1 <= reviewed_settle_ticks <= maximum:
                raise Refused('Camera-reviewed settling tolerance exceeds joint-specific limit')
            settle_ticks = reviewed_settle_ticks
            extended_elbow = joint == 'right_arm_elbow_flex' and settle_ticks > 11
        if not self.execute:
            raise Refused("Read-only mode cannot send motor commands")
        allowed = [f'{self.config["arm"]}_arm_gripper'] if gripper else self.config["joints"]
        if joint not in allowed or type(ticks) is not int or not 0 < abs(ticks) <= 68:
            raise Refused("Invalid single-joint encoder command")
        s, before = self.status()
        if s["phase"] != "holding":
            raise Refused("Previous motor command has not finished")
        if self.path_ticks + abs(ticks) > self.limits.max_path_ticks:
            raise Refused("Total experiment travel budget exhausted")
        goal = before[joint] + ticks
        lo, hi = self.config["ranges"][joint]
        if not lo + 4 <= goal <= hi - 4 or abs(goal - self.origin[joint]) > self.limits.trust_ticks:
            raise Refused("Command leaves the calibrated local joint envelope")
        path = self.folder / "command.json"
        if path.exists() and read_json(path).get("id") != self.last_id:
            raise Refused("Another command writer is active; stop the manual command loop")
        command_id = max(time.time_ns(), (self.last_id or 0) + 1)
        atomic_json(path, {"id": command_id, "op": "move", "delta_ticks": {joint: ticks}})
        self.last_id = command_id
        self.commands_sent += 1
        self.path_ticks += abs(ticks)
        end = self.clock() + self.limits.command_timeout_s
        stable = 0
        previous_position = None
        previous_stamp = -1
        while self.clock() < end:
            s, q = self.status()
            if read_json(path).get("id") != command_id:
                raise Refused("Command overwritten by another client")
            if (s.get("last_rejected") or {}).get("id") == command_id:
                raise Refused(f"Motor owner rejected command: {s['last_rejected']['reason']}")
            for n in before:
                if n != joint and abs(q[n] - before[n]) > self.limits.settle_ticks:
                    raise Refused(f"Uncommanded joint drift: {n}")
            if extended_elbow and not min(before[joint],goal)-3 <= q[joint] <= max(before[joint],goal)+3:
                raise Refused('Reviewed elbow left the commanded travel envelope')
            if s.get("completed") == command_id and s["phase"] == "holding" and s["time"] > previous_stamp:
                previous_stamp = s["time"]
                if extended_elbow:
                    displacement = (q[joint]-before[joint]) * (1 if ticks>0 else -1)
                    if displacement < max(12,abs(ticks)//2):
                        raise Refused('Reviewed elbow moved too little or in the wrong direction')
                    if previous_position is not None and abs(q[joint]-previous_position)>3:
                        stable = 0
                    previous_position = q[joint]
                stable = stable + 1 if abs(q[joint] - goal) <= settle_ticks else 0
                if stable >= 2:
                    return q, self.clock()
            self.sleep(.03)
        raise Refused("No matching acknowledgement with measured settling before deadline")

    def abort(self):
        """Ask the established owner to STOP, preserving its release policy.

        Used only after this client issued a move. It never reconnects, clears
        STOP, renews a lease blindly, or changes the owner's torque behavior.
        """
        if not self.execute or (self.path_ticks == 0 and self.commands_sent == 0) or self.aborted:
            return
        s = read_json(self.folder / "status.json")
        if s.get("started") != self.started:
            return  # Do not send STOP to a replacement session owned by someone else.
        atomic_json(self.folder / "command.json", {"id": time.time_ns(), "op": "stop"})
        self.aborted = True

    def release_and_verify(self):
        """STOP only at a commissioned supported park; verify owner release."""
        self.abort()
        if not self.aborted:
            raise Refused("No release was requested from this owner")
        command = read_json(self.folder / "command.json")
        end = self.clock()+self.limits.command_timeout_s
        while self.clock() < end:
            s = read_json(self.folder / "status.json")
            if s.get("started") != self.started:
                raise Refused("Owner restarted during park release")
            if read_json(self.folder / "command.json").get("id") != command["id"]:
                raise Refused("Park release command was overwritten")
            if (s.get("released") is True and s.get("release_errors") == []
                    and s.get("completed") == command["id"] and s.get("phase") in ("released", "stopped")
                    and 0 <= self.clock()-finite(s.get("time")) <= self.limits.status_age_s):
                return
            self.sleep(.03)
        raise Refused("Owner did not confirm all-motor release at supported park")
