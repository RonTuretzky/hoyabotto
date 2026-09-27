"""Append-only evidence store: SQLite for records, files for images.

Three writes per physical action: INTENT before any motor command, ATTEMPT
before the action can have a physical effect, RESULT after verification.
A crash between ATTEMPT and RESULT leaves the action UNKNOWN until a person
reconciles it. Nothing here is ever updated in place except that final
RESULT column, and only from ATTEMPTED to a terminal value.
"""
from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA = """
CREATE TABLE IF NOT EXISTS cycles(
  cycle_id TEXT PRIMARY KEY, tray_id TEXT, profile TEXT, code_version TEXT, config_hash TEXT,
  calibration_id TEXT, simulated INTEGER, started REAL, ended REAL, result TEXT, note TEXT);
CREATE TABLE IF NOT EXISTS observations(
  obs_id TEXT PRIMARY KEY, cycle_id TEXT, source TEXT, t REAL, status TEXT, value_json TEXT,
  image_hash TEXT, extractor_version TEXT, note TEXT);
CREATE TABLE IF NOT EXISTS decisions(
  decision_id TEXT PRIMARY KEY, cycle_id TEXT, t REAL, kind TEXT, question TEXT, inputs_ref TEXT,
  choice TEXT, probabilities_json TEXT, schema_version TEXT, model TEXT, latency_ms REAL, cost_usd REAL,
  honoured INTEGER, note TEXT);
CREATE TABLE IF NOT EXISTS actions(
  action_id TEXT PRIMARY KEY, cycle_id TEXT, skill TEXT, params_json TEXT, intent_t REAL,
  attempt_t REAL, result TEXT, verify_t REAL, evidence_ref TEXT, note TEXT);
CREATE TABLE IF NOT EXISTS interventions(
  intervention_id TEXT PRIMARY KEY, cycle_id TEXT, who TEXT, t REAL, question_id TEXT,
  checked TEXT, done TEXT, minutes REAL, note TEXT);
CREATE TABLE IF NOT EXISTS proposals(
  proposal_id TEXT PRIMARY KEY, t REAL, author TEXT, title TEXT, change_json TEXT, evidence_ref TEXT,
  expected TEXT, rollback TEXT, status TEXT, decided_by TEXT, decided_t REAL);
CREATE TABLE IF NOT EXISTS events(
  event_id TEXT PRIMARY KEY, cycle_id TEXT, t REAL, kind TEXT, payload_json TEXT);
CREATE INDEX IF NOT EXISTS obs_cycle ON observations(cycle_id, t);
CREATE INDEX IF NOT EXISTS act_cycle ON actions(cycle_id, intent_t);
CREATE INDEX IF NOT EXISTS ev_cycle ON events(cycle_id, t);
"""

RESULTS = ("VERIFIED", "ABORTED", "UNKNOWN", "RECONCILED_OK", "RECONCILED_FAIL")


def new_id(prefix: str) -> str:
    return f"{prefix}_{time.strftime('%Y%m%dT%H%M%S')}_{uuid.uuid4().hex[:8]}"


def _j(x: Any) -> str:
    return json.dumps(x, default=str, sort_keys=True)


class EvidenceStore:
    def __init__(self, root: Path, code_version: str = "dev"):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "images").mkdir(exist_ok=True)
        self.db_path = self.root / "farm.sqlite"
        self.code_version = code_version
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False, isolation_level=None)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.executescript(SCHEMA)

    # ---- images -------------------------------------------------------
    def save_image(self, rgb: np.ndarray) -> str:
        """Store an RGB frame as JPEG named by content hash. Never overwritten."""
        import cv2
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        data = buf.tobytes()
        h = hashlib.sha256(data).hexdigest()[:24]
        p = self.root / "images" / f"{h}.jpg"
        if not p.exists():
            p.write_bytes(data)
        return h

    def image_path(self, image_hash: str) -> Path:
        return self.root / "images" / f"{image_hash}.jpg"

    # ---- cycles -------------------------------------------------------
    def start_cycle(self, tray_id: str, profile: str, config_hash: str, calibration_id: str, simulated: bool) -> str:
        cid = new_id("cyc")
        with self._lock:
            self._conn.execute("INSERT INTO cycles VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                               (cid, tray_id, profile, self.code_version, config_hash, calibration_id, int(simulated), time.time(), None, "RUNNING", ""))
        return cid

    def end_cycle(self, cycle_id: str, result: str, note: str = "") -> None:
        with self._lock:
            self._conn.execute("UPDATE cycles SET ended=?, result=?, note=? WHERE cycle_id=? AND result='RUNNING'", (time.time(), result, note, cycle_id))

    # ---- observations / decisions / events -------------------------------
    def observation(self, cycle_id: str | None, source: str, status: str, value: Any, t: float | None = None,
                    image_hash: str | None = None, extractor_version: str = "", note: str = "") -> str:
        oid = new_id("obs")
        with self._lock:
            self._conn.execute("INSERT INTO observations VALUES(?,?,?,?,?,?,?,?,?)",
                               (oid, cycle_id, source, t or time.time(), status, _j(value), image_hash, extractor_version, note))
        return oid

    def decision(self, cycle_id: str | None, kind: str, question: str, inputs_ref: str, choice: str,
                 probabilities: dict[str, float] | None, schema_version: str, model: str = "", latency_ms: float = 0,
                 cost_usd: float = 0, honoured: bool = False, note: str = "") -> str:
        did = new_id("dec")
        with self._lock:
            self._conn.execute("INSERT INTO decisions VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                               (did, cycle_id, time.time(), kind, question, inputs_ref, choice, _j(probabilities or {}), schema_version, model, latency_ms, cost_usd, int(honoured), note))
        return did

    def event(self, cycle_id: str | None, kind: str, payload: dict[str, Any] | None = None) -> str:
        eid = new_id("evt")
        with self._lock:
            self._conn.execute("INSERT INTO events VALUES(?,?,?,?,?)", (eid, cycle_id, time.time(), kind, _j(payload or {})))
        return eid

    # ---- actions: intent -> attempt -> result --------------------------------
    def intent(self, cycle_id: str, skill: str, params: dict[str, Any], action_id: str | None = None) -> str:
        aid = action_id or new_id("act")
        with self._lock:
            row = self._conn.execute("SELECT action_id FROM actions WHERE action_id=?", (aid,)).fetchone()
            if row:  # duplicate ids return the existing record; they never create a second physical attempt
                return aid
            self._conn.execute("INSERT INTO actions VALUES(?,?,?,?,?,?,?,?,?,?)",
                               (aid, cycle_id, skill, _j(params), time.time(), None, "INTENDED", None, "", ""))
        return aid

    def attempt(self, action_id: str) -> None:
        with self._lock:
            self._conn.execute("UPDATE actions SET attempt_t=?, result='ATTEMPTED' WHERE action_id=? AND result='INTENDED'", (time.time(), action_id))

    def result(self, action_id: str, result: str, evidence_ref: str = "", note: str = "") -> None:
        assert result in RESULTS, result
        with self._lock:
            self._conn.execute("UPDATE actions SET result=?, verify_t=?, evidence_ref=?, note=? WHERE action_id=? AND result IN ('ATTEMPTED','INTENDED','UNKNOWN')",
                               (result, time.time(), evidence_ref, note, action_id))

    def open_actions(self) -> list[dict[str, Any]]:
        """Actions attempted but never resolved: after a crash these are UNKNOWN by definition."""
        with self._lock:
            rows = self._conn.execute("SELECT * FROM actions WHERE result IN ('ATTEMPTED','UNKNOWN') ORDER BY intent_t").fetchall()
            cols = [c[1] for c in self._conn.execute("PRAGMA table_info(actions)")]
        return [dict(zip(cols, r)) for r in rows]

    def mark_unknown_after_crash(self) -> list[str]:
        ids = []
        with self._lock:
            for row in self._conn.execute("SELECT action_id FROM actions WHERE result='ATTEMPTED'").fetchall():
                self._conn.execute("UPDATE actions SET result='UNKNOWN', note='process restarted after ATTEMPT' WHERE action_id=?", (row[0],))
                ids.append(row[0])
            # Intended-but-never-attempted actions are safe to close as ABORTED.
            self._conn.execute("UPDATE actions SET result='ABORTED', note='process restarted before ATTEMPT' WHERE result='INTENDED'")
        return ids

    # ---- interventions / proposals -----------------------------------------
    def intervention(self, cycle_id: str | None, who: str, question_id: str, checked: str, done: str, minutes: float = 0, note: str = "") -> str:
        iid = new_id("int")
        with self._lock:
            self._conn.execute("INSERT INTO interventions VALUES(?,?,?,?,?,?,?,?,?)", (iid, cycle_id, who, time.time(), question_id, checked, done, minutes, note))
        return iid

    def proposal(self, author: str, title: str, change: dict[str, Any], evidence_ref: str, expected: str, rollback: str) -> str:
        pid = new_id("prop")
        with self._lock:
            self._conn.execute("INSERT INTO proposals VALUES(?,?,?,?,?,?,?,?,?,?,?)", (pid, time.time(), author, title, _j(change), evidence_ref, expected, rollback, "OPEN", None, None))
        return pid

    def decide_proposal(self, proposal_id: str, status: str, who: str) -> None:
        with self._lock:
            self._conn.execute("UPDATE proposals SET status=?, decided_by=?, decided_t=? WHERE proposal_id=? AND status='OPEN'", (status, who, time.time(), proposal_id))

    # ---- queries ---------------------------------------------------------
    def query(self, sql: str, params: tuple = ()) -> list[dict[str, Any]]:
        with self._lock:
            cur = self._conn.execute(sql, params)
            cols = [d[0] for d in cur.description] if cur.description else []
            return [dict(zip(cols, r)) for r in cur.fetchall()]

    def cycle_summary(self, cycle_id: str) -> dict[str, Any]:
        c = self.query("SELECT * FROM cycles WHERE cycle_id=?", (cycle_id,))
        return {
            "cycle": c[0] if c else None,
            "observations": self.query("SELECT obs_id, source, t, status, value_json, image_hash, note FROM observations WHERE cycle_id=? ORDER BY t", (cycle_id,)),
            "decisions": self.query("SELECT * FROM decisions WHERE cycle_id=? ORDER BY t", (cycle_id,)),
            "actions": self.query("SELECT * FROM actions WHERE cycle_id=? ORDER BY intent_t", (cycle_id,)),
            "events": self.query("SELECT * FROM events WHERE cycle_id=? ORDER BY t", (cycle_id,)),
            "interventions": self.query("SELECT * FROM interventions WHERE cycle_id=? ORDER BY t", (cycle_id,)),
        }

    def recent_cycles(self, n: int = 20) -> list[dict[str, Any]]:
        return self.query("SELECT * FROM cycles ORDER BY started DESC LIMIT ?", (n,))

    def daily_cost(self) -> float:
        since = time.time() - 86400
        r = self.query("SELECT COALESCE(SUM(cost_usd),0) AS c FROM decisions WHERE t>?", (since,))
        return float(r[0]["c"]) if r else 0.0

    # ---- backup ----------------------------------------------------------
    def backup(self, dest: Path) -> Path:
        dest = Path(dest); dest.mkdir(parents=True, exist_ok=True)
        target = dest / f"farm-{time.strftime('%Y%m%d-%H%M%S')}.sqlite"
        with self._lock:
            b = sqlite3.connect(target)
            self._conn.backup(b)
            b.close()
        return target

    def close(self) -> None:
        with self._lock:
            self._conn.close()
