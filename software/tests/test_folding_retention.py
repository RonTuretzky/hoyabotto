"""Loaded support must be on both outer panel faces, with real contact forces."""
import numpy as np
import pytest

from carton.folding_retention import right_support_evidence


def support_scene(*, below=False, missing_right=False):
    mj = pytest.importorskip('mujoco')
    z = -.02 if below else .02
    gravity = 9.81 if below else -9.81
    model = mj.MjModel.from_xml_string(f'''<mujoco><compiler angle="radian"/>
      <option timestep=".002" gravity="0 0 {gravity}"/>
      <worldbody>
        <body name="short_left" pos="-.06 0 0" euler="0 1.57079632679 0">
          <geom name="short_left_cardboard" type="box" size=".0015 .04 .03"/>
        </body>
        <body name="short_right" pos=".06 0 0" euler="0 -1.57079632679 0">
          <geom name="short_right_cardboard" type="box" size=".0015 .04 .03"/>
        </body>
        <body name="right_gripper_link" pos="0 0 {z}"><freejoint/>
          <geom name="right_wrist_roll_follower_probe" type="sphere" size=".01" pos="-.06 0 0" mass=".1"/>
          <geom name="right_moving_jaw_probe" type="sphere" size=".01" pos="{.25 if missing_right else .06} 0 0" mass=".1"/>
        </body>
      </worldbody></mujoco>''')
    data = mj.MjData(model)
    for _ in range(500):
        mj.mj_step(model, data)
    return model, data


def test_two_loaded_outer_faces_are_required():
    model, data = support_scene()
    before = {key:getattr(data,key).copy() for key in ('qpos','qvel','ctrl','qacc_warmstart')}
    time=data.time
    row = right_support_evidence(model, data)
    assert row['both_loaded']
    assert all(force > .02 for force in row['forces_N'].values())
    assert data.time==time
    for key,value in before.items():np.testing.assert_array_equal(value,getattr(data,key))


def test_under_panel_contact_cannot_be_counted_as_hold_down():
    model, data = support_scene(below=True)
    row = right_support_evidence(model, data)
    assert row['contacts']
    assert all(not c['outer_face'] for c in row['contacts'])
    assert not row['both_loaded']


def test_missing_second_support_does_not_pass():
    model, data = support_scene(missing_right=True)
    row = right_support_evidence(model, data)
    assert row['forces_N']['short_right'] == 0
    assert not row['both_loaded']


def test_cached_contact_before_integration_does_not_prove_current_support():
    model,data=support_scene()
    old_contacts=data.ncon
    assert old_contacts>0
    data.qpos[2]+=.1
    # Deliberately stale contact arrays: evidence must evaluate current qpos
    # in its own copy, without updating the executing world's caches.
    row=right_support_evidence(model,data)
    assert not row['both_loaded'] and row['contacts']==[]
    assert data.ncon==old_contacts
