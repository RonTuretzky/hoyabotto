"""carton.fold_policy_chat (the chat Mac's fold-policy tools) and tools/install_fold_policy_chat.py. No robot, network
or chat server: the wrapped robot is a recorder and the runner is a stand-in module run as a real subprocess."""
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import pytest

from carton import fold_policy_chat as C
from tools import install_fold_policy_chat as I

FAKE_RUNNER = '''
import argparse, json, os, signal, sys, time
from pathlib import Path
ap = argparse.ArgumentParser()
for flag in ("--checkpoint", "--transport", "--pilot-root", "--out", "--max-steps", "--gripper-mode", "--device",
             "--parent-pid", "--operator"):
    ap.add_argument(flag)
ap.add_argument("--joint-map", action="append"); ap.add_argument("--camera", action="append")
ap.add_argument("--execute", action="store_true")
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True)
(out / "argv.json").write_text(json.dumps(sys.argv[1:]))
behaviour = json.loads(Path(os.environ["FAKE_BEHAVIOUR"]).read_text())
(out / "preflight.json").write_text(json.dumps({"warnings": behaviour.get("warnings", [])}))
stopped = []
signal.signal(signal.SIGINT, lambda *_: stopped.append(1))
deadline = time.time() + behaviour.get("run_s", 0)
while time.time() < deadline and not stopped:
    time.sleep(0.01)
(out / "summary.json").write_text(json.dumps({"steps": int(a.max_steps), "execute": a.execute,
    "aborted": "Stop requested by the operator hook" if stopped else behaviour.get("aborted"),
    "effective_hz": 10.0, "commands_sent": 3 if a.execute else 0, "clamp_counts": {}}))
sys.exit(1 if stopped or behaviour.get("aborted") else 0)
'''


class Inner:
    """The pilot's robot chain below the wrapper: a catalog and a call recorder."""
    def __init__(self, config):
        self.config, self.calls = str(config), []

    def catalog(self):
        return {'tools': [{'type': 'function', 'function': {'name': n, 'description': '', 'parameters': {}}}
                          for n in ('robot_get_state', 'robot_move_joint_targets', 'robot_stop')]}

    def call(self, name, args, request_id=None):
        self.calls.append(name)
        return {'ok': True, 'result': {'tool': name}}


@pytest.fixture
def rig(tmp_path):
    private = tmp_path / 'pilot/.private'
    private.mkdir(parents=True)
    software = tmp_path / 'software'
    software.mkdir()
    (software / 'fake_runner.py').write_text(FAKE_RUNNER)
    ckpt = tmp_path / 'ckpt'
    ckpt.mkdir()
    (ckpt / 'model.safetensors').write_bytes(b'weights')
    maps = {}
    for arm in ('left', 'right'):
        maps[arm] = str(tmp_path / f'{arm}.json')
        Path(maps[arm]).write_text('{}')
    behaviour = tmp_path / 'behaviour.json'
    behaviour.write_text('{}')
    cfg = I.default_config(tmp_path / 'pilot', ckpt, maps, python=sys.executable, runs_dir=tmp_path / 'runs')
    cfg.update(software_root=str(software), runner_module='fake_runner', blockers=[])
    (private / 'fold-policy.json').write_text(json.dumps(cfg))
    inner = Inner(private / 'robot.json')
    robot = C.FoldPolicyRobot(inner)
    monkey = pytest.MonkeyPatch()
    monkey.setenv('FAKE_BEHAVIOUR', str(behaviour))
    yield type('Rig', (), {'robot': robot, 'inner': inner, 'private': private, 'behaviour': behaviour,
                           'tmp': tmp_path})
    monkey.undo()


def configure(rig, **changes):
    path = rig.private / 'fold-policy.json'
    cfg = json.loads(path.read_text())
    cfg.update(changes)
    path.write_text(json.dumps(cfg))


def argv(result):
    return json.loads((Path(result['result']['run_dir']) / 'argv.json').read_text())


def test_catalog_adds_the_tools_only_when_configured(rig, tmp_path):
    names = [t['function']['name'] for t in rig.robot.catalog()['tools']]
    assert names[-3:] == list(C.TOOLS)
    bare = C.FoldPolicyRobot(Inner(tmp_path / 'other/robot.json'))
    assert [t['function']['name'] for t in bare.catalog()['tools']] == ['robot_get_state', 'robot_move_joint_targets',
                                                                          'robot_stop']


def test_dry_run_never_executes_and_records_a_clean_run(rig):
    out = rig.robot.call(C.DRY_RUN, {'max_steps': 20})
    assert out['ok'] is True and out['result']['clean'] is True and out['result']['commands_sent'] == 0
    args = argv(out)
    assert '--execute' not in args and '--operator' not in args
    assert args[args.index('--parent-pid') + 1].isdigit() and args[args.index('--gripper-mode') + 1] == 'hold'
    assert {'front=oak', 'left_wrist=left_wrist', 'right_wrist=right_wrist'} <= set(args)
    assert rig.inner.calls == []                        # the wrapper itself calls nothing on the robot
    status = rig.robot.call(C.STATUS)['result']
    assert status['last_dry_run']['clean'] is True and status['execute_enabled'] is False


def test_dry_run_warnings_make_it_unclean_except_the_motors_enabled_note(rig):
    rig.behaviour.write_text(json.dumps({'warnings': ['camera_aspect: front frames are 640x360 ...']}))
    assert rig.robot.call(C.DRY_RUN, {})['result']['clean'] is False
    rig.behaviour.write_text(json.dumps({'warnings': ['Dry-run with motors enabled: the owner releases ...']}))
    assert rig.robot.call(C.DRY_RUN, {})['result']['clean'] is True


def test_run_is_refused_until_the_owner_config_enables_it(rig):
    rig.robot.call(C.DRY_RUN, {})
    out = rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})
    assert out['ok'] is False and 'execute_enabled false' in out['result']['error']
    assert "operator 'Ron' is not listed" in out['result']['error'] and out['result']['motor_writes'] == 0
    configure(rig, execute_enabled=True, operators=['Ron'], blockers=['owner stream mode not deployed'])
    out = rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})
    assert out['ok'] is False and 'open blocker: owner stream mode not deployed' in out['result']['error']
    configure(rig, blockers=[])
    out = rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 11})
    assert out['ok'] is False and 'max_steps must be 1..10' in out['result']['error']
    out = rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})
    assert out['ok'] is True and out['result']['commands_sent'] == 3 and 'HOLDING' in out['result']['note']
    args = argv(out)
    assert args[args.index('--operator') + 1] == 'Ron' and '--execute' in args


def test_run_needs_a_recent_clean_dry_run_of_the_same_files(rig):
    configure(rig, execute_enabled=True, operators=['Ron'])
    assert 'no dry run on record' in rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})['result']['error']
    rig.behaviour.write_text(json.dumps({'aborted': 'Camera watchdog'}))
    rig.robot.call(C.DRY_RUN, {})
    assert 'not clean' in rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})['result']['error']
    rig.behaviour.write_text('{}')
    rig.robot.call(C.DRY_RUN, {})
    (rig.tmp / 'ckpt/model.safetensors').write_bytes(b'other weights')
    assert 'changed since the last dry run' in rig.robot.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})['result']['error']
    rig.robot.call(C.DRY_RUN, {})
    later = C.FoldPolicyRobot(rig.inner, clock=lambda: time.time() + 1000)
    assert 's old (limit 900 s)' in later.call(C.RUN, {'operator': 'Ron', 'max_steps': 5})['result']['error']


def test_while_a_job_runs_motion_is_refused_reads_pass_and_stop_interrupts_it(rig):
    rig.behaviour.write_text(json.dumps({'run_s': 30}))
    result = {}
    worker = threading.Thread(target=lambda: result.update(rig.robot.call(C.DRY_RUN, {'max_steps': 100})))
    worker.start()
    deadline = time.time() + 10
    while not (rig.robot.running() and (Path(rig.robot.job['run_dir']) / 'preflight.json').exists()) \
            and time.time() < deadline:
        time.sleep(0.01)
    assert rig.robot.running()
    blocked = rig.robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {}})
    assert blocked['ok'] is False and 'is running' in blocked['result']['error']
    assert rig.robot.call('robot_get_state', {})['ok'] is True
    assert rig.robot.call(C.STATUS)['result']['running']['kind'] == 'dry run'
    second = rig.robot.call(C.DRY_RUN, {})
    assert second['ok'] is False and 'Another local motion client' in second['result']['error']
    t0 = time.time()
    assert rig.robot.call('robot_stop', {})['ok'] is True
    worker.join(timeout=10)
    assert not worker.is_alive() and time.time() - t0 < 5
    assert rig.inner.calls == ['robot_get_state', 'robot_stop']          # the refused move never reached the robot
    assert result['ok'] is False and result['result']['interrupted'] == 'robot_stop'
    assert 'Stop requested' in result['result']['aborted']


SNIPPET = '''import json
from farm.perception.gemma_tags import TagRobot
from farm.perception.gemma_calibration import CalibrationRobot
from farm.perception.twin_robot import TwinRobot
MOVE_TOOLS=('robot_set_motor_enable','robot_move_joint_targets','robot_move_path','robot_set_gripper','robot_halt_motion','robot_move_base','robot_stop')
def main(args):
    chat=Chat(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config)))));lan=LanAccess(chat,port=args.lan_port,local_port=args.port)
'''


def test_installer_patch_is_idempotent_and_reversible(tmp_path):
    patched = I.patch_source(SNIPPET, module=Path('/repo/software/carton/fold_policy_chat.py'))
    assert 'chat=Chat(FoldPolicyRobot(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config))))));lan=' in patched
    assert "run_path('/repo/software/carton/fold_policy_chat.py')['FoldPolicyRobot']" in patched
    assert "'robot_stop','robot_fold_policy_dry_run','robot_fold_policy_run')" in patched
    assert I.patch_source(patched) == patched
    assert I.unpatch_source(patched) == SNIPPET
    with pytest.raises(ValueError):
        I.patch_source(SNIPPET.replace('chat=Chat(', 'chat=Other('))


def test_installer_writes_a_disabled_config_and_backs_up_the_pilot(tmp_path):
    pilot = tmp_path / 'pilot'
    pilot.mkdir()
    (pilot / 'chat_server.py').write_text(SNIPPET)
    ckpt = tmp_path / 'ckpt'
    ckpt.mkdir()
    (ckpt / 'model.safetensors').write_bytes(b'w')
    maps = {arm: str(tmp_path / f'{arm}.json') for arm in ('left', 'right')}
    for path in maps.values():
        Path(path).write_text('{}')
    out = I.install(pilot, ckpt, maps)
    assert out == {'changed': True, 'config_written': True, 'config': str(pilot / '.private/fold-policy.json'),
                   'restart_required': True, 'motor_writes': 0}
    cfg = json.loads((pilot / '.private/fold-policy.json').read_text())
    assert cfg['execute_enabled'] is False and cfg['operators'] == [] and cfg['blockers'] and cfg['gripper_mode'] == 'hold'
    assert len(list((pilot / '.private/fold-policy-backups').iterdir())) == 1
    assert I.install(pilot)['changed'] is False        # second run: config kept, source already patched
