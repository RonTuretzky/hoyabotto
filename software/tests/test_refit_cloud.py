import pytest

from tools.train_refit_cloud import require_collection
from tools.package_refit_cloud import relocate


def row():
    return dict(success=True, stop_reason='task flaps folded and held', exit_code=0,
                controller_error=None, contact_audit=dict(steps=10, first_refusal=None,
                                                        independent_score=dict(passed=True)))


def test_cloud_gate_rejects_stale_success_flag():
    rows = [row() for _ in range(16)]
    rows[0]['contact_audit']['independent_score']['passed'] = False
    with pytest.raises(RuntimeError, match='disagrees'):
        require_collection(dict(episodes=16, successes=16, results=rows), minimum=13)


def test_cloud_gate_retains_minimum_fraction_and_count():
    rows = [row() for _ in range(16)]
    for r in rows[:4]:
        r['success'] = False
    with pytest.raises(RuntimeError, match='gate failed'):
        require_collection(dict(episodes=16, successes=12, results=rows), minimum=10)
    assert require_collection(dict(episodes=16, successes=12, results=rows), minimum=10, fraction=.75) == 12


def test_relocated_mesh_paths_are_absolute_and_idempotent(tmp_path):
    for name in ('scene-assets/arm-import.xml', 'scene-assets/jaw-collision/manifest.json'):
        p = tmp_path/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('__SIMULATION_ROOT__/some-asset')
    relocate(tmp_path)
    relocate(tmp_path)
    assert (tmp_path/'scene-assets/arm-import.xml').read_text() == f'{tmp_path}/some-asset'


def test_policy_render_excludes_teacher_marker_and_collision_hulls(tmp_path):
    import numpy as np
    from tools.fold_demos_to_lerobot import render_trial
    run = tmp_path/'run'
    run.mkdir()
    (run/'scene.xml').write_text('''<mujoco><visual><headlight diffuse="1 1 1"/></visual>
      <worldbody><camera name="front" pos="0 0 2"/>
      <geom type="box" size=".5 .5 .1" rgba="0 0 1 1" group="0"/>
      <geom type="box" pos="-.25 0 .3" size=".24 .5 .1" rgba="1 0 0 1" group="3"/>
      <geom type="box" pos=".25 0 .3" size=".24 .5 .1" rgba="0 1 0 1" group="4"/>
      </worldbody></mujoco>''')
    np.savez(tmp_path/'demo.npz', qpos=np.empty((1,0)))
    rgb = render_trial((str(tmp_path),0,64,64,{'front':'front'}))['front'][0]
    center = rgb[25:39,25:39].astype(float).mean(axis=(0,1))
    assert center[2] > 2*max(center[0], center[1])
