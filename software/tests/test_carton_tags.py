"""AprilTag commissioning: real detector, encoded frames, no physical devices."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton.servo import cli
from carton.servo.common import Refused, atomic_json
from carton.servo.features import FeatureTracks, tags_from_bgr
from carton.servo.tag_check import TagView, check_tags, default_regions
from carton.servo.tag_kit import make_kit, marker_image


def scene(tool_x=240, anchor_x=30, target_id=3, tool_size=80):
    image = np.full((480, 640, 3), 255, np.uint8)
    for tag_id, x, y, size in ((1, anchor_x, 50, 80), (2, tool_x, 180, tool_size), (target_id, 465, 330, 80)):
        if tag_id is not None:
            im = marker_image(tag_id, 1)[1:9, 1:9]
            image[y:y+size, x:x+size] = cv2.resize(im, (size, size), interpolation=cv2.INTER_NEAREST)
    return image


class Stream:
    def __init__(self, folder, variant=None):
        self.folder = folder
        folder.mkdir()
        self.now, self.seq = 1000.0, -1
        self.variant = variant
        self.publish()

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        seq = int((self.now-1000)/.2)
        if seq != self.seq:
            self.publish()

    def publish(self):
        self.seq = int((self.now-1000)/.2)
        for name in ("head", "right_wrist"):
            im = scene()
            if self.variant:
                im = self.variant(self.seq, name, im)
            path = self.folder / f"{name}-{self.seq}.png"
            cv2.imwrite(str(path), im)
            m = {"schema": 1, "camera_id": name, "stream_id": "stream-"+name,
                 "seq": self.seq, "captured_at": 1000+self.seq*.2,
                 "image": path.name, "width": 640, "height": 480,
                 "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            atomic_json(self.folder / f"{name}.json", m)

    def check(self, out):
        return check_tags(out, seconds=2, frames_dir=self.folder,
                          clock=self.clock, monotonic=self.clock, sleep=self.sleep)


def test_printed_kit_decodes_and_defines_physical_scale(tmp_path):
    result = make_kit(tmp_path / "kit")
    assert result["family"] == "tag36h11" and result["motor_writes"] == 0
    for row in result["markers"]:
        image = cv2.imread(str(tmp_path / "kit" / row["png"]))
        assert set(tags_from_bgr(image)) == {row["tag_id"]}
        assert row["cut_square_mm"] == row["black_square_mm"]*1.25
        assert all(v == "1" for v in row["grid_black_is_1"][0])
    assert 'width="75mm"' in (tmp_path / "kit" / "anchor-1.svg").read_text()
    assert "100 mm" in (tmp_path / "kit" / "print.html").read_text()
    with pytest.raises(FileExistsError):
        make_kit(tmp_path / "kit")


@pytest.mark.parametrize("size", [0, 19, 81, float("nan")])
def test_invalid_print_sizes_refuse(tmp_path, size):
    with pytest.raises(Refused): make_kit(tmp_path / "kit", tool_mm=size)
    assert not (tmp_path / "kit").exists()


def test_view_uses_same_controller_points_and_reports_identity():
    image = scene()
    specs = default_regions(True)
    view = TagView(specs, reference=image)
    moved = scene(tool_x=258)
    expected = FeatureTracks(image, specs).locate(moved)
    actual = view.measure(moved)
    assert actual["ok"]
    for name, point in expected.items():
        assert actual["features"][name]["point"] == pytest.approx(point)
        assert actual["features"][name]["hamming"] == 0
        assert actual["features"][name]["margin"] >= 30


@pytest.mark.parametrize("kwargs,word", [
    ({"target_id": None}, "missing"), ({"target_id": 4}, "missing"),
    ({"target_id": 2}, "duplicate"), ({"anchor_x": 35}, "anchor moved"),
    ({"tool_x": 350}, "envelope"), ({"tool_size": 16}, "small"),
])
def test_failures_keep_reason_and_do_not_reuse_points(kwargs, word):
    view = TagView(default_regions(True), reference=scene())
    result = view.measure(scene(**kwargs))
    assert not result["ok"] and word in json.dumps(result)
    assert all("point" not in v for v in result["features"].values() if not v["ok"])


def test_perspective_tag_motion_uses_homography():
    im = scene()
    moved = im.copy()
    moved[180:260, 240:320] = 255
    source = marker_image(2, 10)
    transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [99, 0], [99, 99], [0, 99]]),
                                          np.float32([[242, 170], [335, 188], [317, 273], [250, 256]]))
    warped = cv2.warpPerspective(source, transform, (640, 480), borderValue=(255, 255, 255))
    moved = np.minimum(moved, warped)
    result = TagView(default_regions(True), reference=im).measure(moved)
    assert result["ok"]
    assert result["features"]["tool"]["point"] == pytest.approx(tags_from_bgr(moved)[2]["center"], abs=.5)


def test_camera_only_audit_passes_and_preserves_raw_frames(tmp_path, monkeypatch):
    from carton.servo.transport import SessionTransport
    def forbidden(*a, **kw): raise AssertionError("A tag check must never construct a motor transport")
    monkeypatch.setattr(SessionTransport, "__init__", forbidden)
    stream = Stream(tmp_path / "frames")
    result = stream.check(tmp_path / "audit")
    assert result["status"] == "TAG_CHECK_PASSED", result
    assert result["pairs"] >= 8 and result["motor_writes"] == 0
    trace = [json.loads(l) for l in Path(result["trace"]).read_text().splitlines()]
    pair = next(r for r in trace if r["event"] == "pair")
    raw = tmp_path / "audit" / pair["frames"]["head"]["raw_image"]
    assert np.array_equal(cv2.imread(str(raw)), scene())
    assert pair["frames"]["head"]["raw_sha256"] == hashlib.sha256(raw.read_bytes()).hexdigest()
    assert len(list((tmp_path / "audit").glob("*-raw.png"))) == 2*result["pairs"]
    assert not list(tmp_path.rglob("command.json"))


def test_transient_loss_is_recorded_even_if_later_frames_recover(tmp_path):
    def variant(seq, name, im): return scene(target_id=None) if seq == 3 else im
    stream = Stream(tmp_path / "frames", variant)
    result = stream.check(tmp_path / "audit")
    assert result["status"] == "TAG_CHECK_FAILED" and result["failed_pairs"] == 1
    pairs = [json.loads(l) for l in Path(result["trace"]).read_text().splitlines() if json.loads(l)["event"] == "pair"]
    failed = next(r for r in pairs if not r["ok"])
    assert failed["detections"]["head"]["features"]["target"]["ok"] is False
    assert pairs[-1]["ok"] is True
    raw = tmp_path / "audit" / failed["frames"]["head"]["raw_image"]
    assert 3 not in tags_from_bgr(cv2.imread(str(raw)))


@pytest.mark.parametrize("fault", ["restart", "hash", "stale", "same_device", "frozen", "skew"])
def test_camera_contract_failures_cannot_pass(tmp_path, fault):
    stream = Stream(tmp_path / "frames")
    path = stream.folder / "right_wrist.json"
    if fault == "same_device":
        m = json.loads(path.read_text()); m["camera_id"] = "head"; atomic_json(path, m)
    original_sleep = stream.sleep
    def sleep(seconds):
        if fault == "frozen":
            stream.now += seconds
            return
        original_sleep(seconds)
        if stream.seq >= 3:
            m = json.loads(path.read_text())
            if fault == "restart": m["stream_id"] = "restarted"
            elif fault == "hash": m["sha256"] = "wrong"
            elif fault == "stale": m["captured_at"] -= 5
            elif fault == "skew": m["captured_at"] -= .4
            atomic_json(path, m)
    stream.sleep = sleep
    result = stream.check(tmp_path / "audit")
    assert result["status"] == "TAG_CHECK_FAILED", result
    assert result["motor_writes"] == 0 and result["failures"]


def test_seed_persists_minimum_size_in_controller_config(tmp_path):
    image = tmp_path / "reference.png"; cv2.imwrite(str(image), scene())
    config = {"session_dir": "missing-session", "calibration_file": "missing-calibration",
              "cameras": {"head": {"reference": str(image), "manifest": "missing-stream", "regions": {}}}}
    path = tmp_path / "experiment.json"; atomic_json(path, config)
    assert cli.main(["seed", "--config", str(path), "--camera", "head", "--tag", "tool:2"]) == 0
    seeded = json.loads(path.read_text())
    assert seeded["cameras"]["head"]["regions"]["tool"]["min_edge_px"] == 24
    before = path.read_bytes()
    assert cli.main(["seed", "--config", str(path), "--camera", "head", "--tag", "target:3",
                     "--min-tag-edge-px", "150"]) == 2
    assert path.read_bytes() == before


@pytest.mark.parametrize("anchor_shift,passed", [(0, True), (5, False)])
def test_seeded_check_uses_saved_reference_without_motor_files(tmp_path, anchor_shift, passed):
    from carton.servo.simulation import PixelRig
    config = PixelRig(tmp_path / "old-experiment").config
    stream = Stream(tmp_path / "frames", lambda seq, name, im: scene(anchor_x=30+anchor_shift))
    for name, cam in config["cameras"].items():
        cv2.imwrite(cam["reference"], scene())
        cam.update(manifest=str(stream.folder / f"{name}.json"), camera_id=name, regions=default_regions(name == "head"))
    config.update(session_dir=str(tmp_path / "nonexistent-session"), calibration_file=str(tmp_path / "nonexistent-calibration"))
    result = check_tags(tmp_path / "audit", seconds=2, config=config,
                        clock=stream.clock, monotonic=stream.clock, sleep=stream.sleep)
    assert result["mode"] == "SEEDED_TAG_TRACKING"
    assert (result["status"] == "TAG_CHECK_PASSED") == passed
    assert result["motor_writes"] == 0
    assert not Path(config["session_dir"]).exists()
    assert not Path(config["calibration_file"]).exists()
