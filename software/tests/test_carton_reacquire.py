"""Bounded stationary visual reacquisition; no devices or motor commands."""
import hashlib
import json

import pytest

from carton.servo.common import Refused
from carton.servo.simulation import PixelRig
from carton.servo.vision import Observer


def setup_observer(tmp_path, advance=True):
    rig = PixelRig(tmp_path / "frames")
    rig.publish()
    def sleep(seconds):
        rig.now += seconds
        if advance:
            rig.publish()
    observer = Observer(rig.config, rig.limits, clock=rig.clock, sleep=sleep)
    observer.failure_folder = tmp_path / "failures"
    return rig, observer


def fail_patch(observer, monkeypatch, failures=1, feature="tool",
               reason="Tracked patch does not move as one rigid region"):
    tracker = observer.trackers["head"].trackers[feature]
    original = tracker.locate
    calls = []
    def locate(image):
        calls.append(1)
        if failures is None or len(calls) <= failures:
            raise Refused(reason)
        return original(image)
    monkeypatch.setattr(tracker, "locate", locate)
    return calls


def test_single_transient_requires_three_good_distinct_pairs_and_preserves_bytes(tmp_path, monkeypatch):
    rig, observer = setup_observer(tmp_path)
    original_manifest = json.loads((rig.folder / "head.json").read_text())
    original_bytes = (rig.folder / original_manifest["image"]).read_bytes()
    calls = fail_patch(observer, monkeypatch)
    start = rig.now
    result = observer.observe()
    assert len(calls) == 4
    assert result.sequences["head"] == 4
    assert rig.now - start < 1
    folder = observer.failure_folder / "failure-0001"
    assert (folder / "head.jpg").read_bytes() == original_bytes
    saved = json.loads((folder / "failure.json").read_text())
    assert saved["camera"] == "head" and saved["feature"] == "tool"
    assert saved["frames"]["head"]["sha256"] == hashlib.sha256(original_bytes).hexdigest()


def test_persistent_failure_expires_without_extending_original_deadline(tmp_path, monkeypatch):
    rig, observer = setup_observer(tmp_path)
    fail_patch(observer, monkeypatch, failures=None)
    start = rig.now
    with pytest.raises(Refused, match="reacquisition expired: head.tool"):
        observer.observe(timeout=2)
    assert rig.now - start <= 1.1


def test_anchor_quality_failure_is_immediate_even_when_tool_also_fails(tmp_path, monkeypatch):
    rig, observer = setup_observer(tmp_path)
    tool_calls = fail_patch(observer, monkeypatch, failures=None)
    anchor_calls = fail_patch(observer, monkeypatch, failures=None, feature="anchor")
    start = rig.now
    with pytest.raises(Refused, match="head.anchor"):
        observer.observe()
    assert rig.now == start and len(anchor_calls) == 1 and tool_calls == []


def test_duplicate_pairs_cannot_count_as_recovery_passes(tmp_path, monkeypatch):
    rig, observer = setup_observer(tmp_path, advance=False)
    calls = fail_patch(observer, monkeypatch)
    with pytest.raises(Refused, match="reacquisition expired"):
        observer.observe()
    assert len(calls) == 1


@pytest.mark.parametrize("reason", ["Patch scale is outside its local tracking envelope",
                                    "Patch left its reviewed local image envelope"])
def test_scale_and_envelope_failures_never_retry(tmp_path, monkeypatch, reason):
    rig, observer = setup_observer(tmp_path)
    calls = fail_patch(observer, monkeypatch, failures=None, reason=reason)
    start = rig.now
    with pytest.raises(Refused, match=reason):
        observer.observe()
    assert rig.now == start and len(calls) == 1


def test_health_failure_during_recovery_stops_immediately(tmp_path, monkeypatch):
    rig, observer = setup_observer(tmp_path)
    calls = fail_patch(observer, monkeypatch)
    health_calls = []
    def health():
        health_calls.append(1)
        if len(health_calls) > 1:
            raise Refused("motor fault")
        return {"phase": "holding"}, rig.q.copy()
    observer.health_check = health
    with pytest.raises(Refused, match="motor fault"):
        observer.observe()
    assert len(calls) == 1
