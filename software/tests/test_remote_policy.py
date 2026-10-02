"""Policy server + RemotePolicy end to end over an in-process HTTP client. No network, no torch."""
import numpy as np
import pytest
from fastapi.testclient import TestClient

from farm.learning.remote import RemotePolicy, RemotePolicyError
from farm.learning.server import decode_image, encode_image, make_app


class FakeRunner:
    """Stands in for PolicyRunner: returns a chunk that counts up from the state, and records what it saw."""

    def __init__(self, chunk=8, n_action_steps=4, action_dim=6):
        self.chunk, self.n, self.dim = chunk, n_action_steps, action_dim
        self.resets = 0
        self.seen = []
        self.fail = None

    def input_spec(self):
        return {"policy_type": "fake", "cameras": {"observation.images.wrist": (48, 64)}, "camera_names": ["wrist"], "state_dim": self.dim,
                "action_dim": self.dim, "chunk_size": self.chunk, "n_action_steps": self.n}

    def reset(self):
        self.resets += 1

    def act_chunk(self, state, images, task=None):
        if self.fail:
            raise self.fail
        if state.shape[0] != self.dim:
            raise ValueError(f"state has {state.shape[0]} dims, policy expects {self.dim}")
        self.seen.append({k: v.shape for k, v in images.items()})
        return np.stack([state + i for i in range(1, self.chunk + 1)]).astype(np.float32)


def _pair(runner, token=None, client_token=None, **kw):
    client = TestClient(make_app(runner, token=token, checkpoint="ckpt"))
    return RemotePolicy("http://testserver", token=client_token, client=client, **kw)


def test_image_roundtrip_keeps_shape_and_channel_order():
    img = np.zeros((48, 64, 3), dtype=np.uint8); img[..., 0] = 200      # red in RGB
    out = decode_image(encode_image(img, quality=95))
    assert out.shape == img.shape and out[..., 0].mean() > 150 and out[..., 2].mean() < 60


def test_remote_policy_plays_n_action_steps_then_asks_again():
    runner = FakeRunner(chunk=8, n_action_steps=4)
    pol = _pair(runner)
    assert pol.input_spec()["checkpoint"] == "ckpt" and pol.input_spec()["cameras"] == {"observation.images.wrist": [48, 64]}
    pol.reset()
    assert runner.resets == 1
    img = {"observation.images.wrist": np.zeros((48, 64, 3), dtype=np.uint8)}
    state = np.zeros(6, dtype=np.float32)
    acts = [pol.act(state, img) for _ in range(4)]
    assert [float(a[0]) for a in acts] == [1.0, 2.0, 3.0, 4.0] and pol.calls == 1        # one request for four actions
    a5 = pol.act(state + 10, img)
    assert float(a5[0]) == 11.0 and pol.calls == 2                                       # fresh observation, fresh chunk
    assert runner.seen[0] == {"observation.images.wrist": (48, 64, 3)}


def test_remote_policy_errors_raise_and_leave_no_stale_actions():
    runner = FakeRunner()
    pol = _pair(runner)
    img = {"observation.images.wrist": np.zeros((48, 64, 3), dtype=np.uint8)}
    with pytest.raises(RemotePolicyError, match="422"):
        pol.act(np.zeros(3, dtype=np.float32), img)                                      # wrong state size
    runner.fail = RuntimeError("boom")
    with pytest.raises(Exception):
        pol.act(np.zeros(6, dtype=np.float32), img)
    assert pol._queue == []

    class Dead:
        def request(self, *a, **k):
            raise ConnectionError("refused")
    dead = RemotePolicy("http://10.0.0.9:8766", client=Dead())
    with pytest.raises(RemotePolicyError, match="unreachable"):
        dead.act(np.zeros(6, dtype=np.float32), img)


def test_token_is_enforced():
    runner = FakeRunner()
    with pytest.raises(RemotePolicyError, match="401"):
        _pair(runner, token="s3cret", client_token="wrong").input_spec()
    assert _pair(runner, token="s3cret", client_token="s3cret").input_spec()["policy_type"] == "fake"


def test_policy_skill_stops_and_holds_when_the_server_dies(sim):
    """The skill treats a dead server like any failed step: stop, report, nothing more is commanded."""
    from farm.adapters.base import ARM_JOINTS, arm_joint
    from farm.skills.policy import PolicySkill
    s = sim()
    joints = [arm_joint("right", j) for j in ARM_JOINTS]
    runner = FakeRunner(chunk=4, n_action_steps=2)
    pol = _pair(runner)
    skill = PolicySkill(s.skills, s.cameras, pol, joints, {"observation.images.wrist": "right_wrist"}, hz=50, max_steps=6)

    calls = {"n": 0}
    real = pol._request

    def flaky(method, path, **kw):
        if path == "/act":
            calls["n"] += 1
            if calls["n"] >= 2:
                raise RemotePolicyError("policy server unreachable at http://testserver: timeout")
        return real(method, path, **kw)
    pol._request = flaky
    out = skill.run("pour")
    assert not out.ok and "policy error" in out.reason and "unreachable" in out.reason
    assert out.steps == 2                                    # the two actions of the first chunk ran, then it stopped
    sent = len(s.robot.sent)
    import time
    time.sleep(0.1)
    assert len(s.robot.sent) == sent                         # nothing more is commanded after the failure
