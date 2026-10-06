"""Material-component checks; these do not validate robot tape placement."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from carton.folding_tape import TapeSpec, add_tape


def coupon(adhesion=.02, flipped=False, timestep=.00005):
    root = ET.fromstring(f'''<mujoco>
      <option timestep="{timestep}" integrator="implicitfast" solver="Newton"
              iterations="120" tolerance="1e-10" cone="elliptic"/>
      <worldbody><geom type="plane" size="1 1 .1"/>
        <body name="plate" pos="0 0 .003"><freejoint name="plate_free"/>
          <geom type="box" size=".06 .035 .003" mass=".05"/>
        </body>
      </worldbody></mujoco>''')
    spec = TapeSpec(length_m=.01, segments=2, adhesion_per_contact_N=adhesion)
    add_tape(root, spec, start_m=[-.005, 0, .00606], flipped=flipped)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    assert model.nu == model.neq == 0
    assert model.joint('plate_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert model.body_subtreemass[model.body('tape_0').id] == pytest.approx(spec.mass_kg)
    return model, mujoco.MjData(model)


def pull(model, data, force, seconds):
    """Explicit material-test load, uniformly applied to both small segments."""
    root = model.body('tape_0').id
    for index in range(2):
        data.xfrc_applied[model.body(f'tape_{index}').id, 2] = force
    peak_tension = 0.
    for _ in range(round(seconds/model.opt.timestep)):
        mujoco.mj_step(model, data)
        assert not any(w.number for w in data.warning)
        for ci in range(data.ncon):
            contact_force = np.zeros(6)
            mujoco.mj_contactForce(model, data, ci, contact_force)
            peak_tension = max(peak_tension, -contact_force[0])
        if data.xpos[root, 2] > .008:
            break
    return float(data.xpos[root, 2]), peak_tension


@pytest.mark.parametrize('adhesion,flipped,holds', [
    (.02, False, True), (0., False, False), (.02, True, False),
])
def test_only_the_sticky_face_supports_a_small_tensile_load(adhesion, flipped, holds):
    model, data = coupon(adhesion, flipped)
    pull(model, data, 0, .05)
    z, tension = pull(model, data, .005, .25)
    if holds:
        assert .0058 < z < .0061
        assert 0 < tension <= adhesion
    else:
        assert z > .008
        assert tension == 0


@pytest.mark.parametrize('timestep', [.00005, .000025])
def test_adhesion_breaks_under_finite_overload_without_pinning_the_plate(timestep):
    model, data = coupon(timestep=timestep)
    pull(model, data, 0, .05)
    z, _ = pull(model, data, .005, .25)
    assert z < .0061
    # 0.24 N total is above the maximum for this two-segment contact patch,
    # while remaining below the plate's 0.49 N weight.
    z, tension = pull(model, data, .12, .10)
    assert z > .008
    assert tension <= .02 + 1e-10
    assert abs(data.body('plate').xpos[2] - .003) < .0005
