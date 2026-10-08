"""Material-component checks; these do not validate robot tape placement."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from carton.folding_tape import TapeSpec, add_tape


def coupon(adhesion=.02, flipped=False, timestep=.00005, integrator='implicitfast', exact_impedance=False):
    root = ET.fromstring(f'''<mujoco>
      <option timestep="{timestep}" integrator="{integrator}" solver="Newton"
              iterations="120" tolerance="1e-10" cone="elliptic"/>
      <worldbody><geom type="plane" size="1 1 .1"/>
        <body name="plate" pos="0 0 .003"><freejoint name="plate_free"/>
          <geom type="box" size=".06 .035 .003" mass=".05"/>
        </body>
      </worldbody></mujoco>''')
    if exact_impedance:
        ET.SubElement(root.find('option'), 'flag', diagexact='enable')
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
@pytest.mark.parametrize('integrator', ['implicitfast', 'discrete'])
def test_only_the_sticky_face_supports_a_small_tensile_load(adhesion, flipped, holds, integrator):
    model, data = coupon(adhesion, flipped, integrator=integrator)
    pull(model, data, 0, .05)
    z, tension = pull(model, data, .005, .25)
    if holds:
        assert .0058 < z < .0061
        assert 0 < tension <= adhesion
    else:
        assert z > .008
        assert tension == 0


@pytest.mark.parametrize('timestep', [.00005, .000025])
@pytest.mark.parametrize('integrator', ['implicitfast', 'discrete'])
def test_adhesion_breaks_under_finite_overload_without_pinning_the_plate(timestep, integrator):
    model, data = coupon(timestep=timestep, integrator=integrator)
    pull(model, data, 0, .05)
    z, _ = pull(model, data, .005, .25)
    assert z < .0061
    # 0.24 N total is above the maximum for this two-segment contact patch,
    # while remaining below the plate's 0.49 N weight.
    z, tension = pull(model, data, .12, .10)
    assert z > .008
    assert tension <= .02 + 1e-10
    assert abs(data.body('plate').xpos[2] - .003) < .0005


@pytest.mark.parametrize('timestep', [.00005, .000025, .00001])
def test_exact_impedance_keeps_finite_native_adhesion_with_no_hidden_constraints(timestep):
    model, data = coupon(timestep=timestep, integrator='discrete', exact_impedance=True)
    assert model.nu == model.neq == 0
    assert model.geom_adhesion[model.geom('tape_adhesive_0').id] == .02
    pull(model, data, 0, .05)
    held, tension = pull(model, data, .005, .25)
    released, peak = pull(model, data, .12, .10)
    assert held < .0061 and released > .008
    assert 0 < tension <= .02 and peak <= .02 + 1e-10


def test_step_resolution_timeline_preserves_chatter_and_late_recontact():
    from tools.diagnose_tape_adhesion import ContactTimeline
    timeline = ContactTimeline()
    timeline.update(0., 0, 0.)
    timeline.update(.1, 4, .02)
    timeline.update(.2, 0, 0.)
    timeline.update(.21, 2, .01)
    assert timeline.report(.3)['first_separation_at_least_50ms_s'] is None
    timeline.update(.4, 0, 0.)
    assert timeline.report(.46)['first_separation_at_least_50ms_s'] == .4
    timeline.update(.5, 2, .01)
    report = timeline.report(.6)
    assert report['first_separation_at_least_50ms_s'] == .4
    assert report['last_adhesive_contact_s'] == .5
    assert report['recontacts_after_complete_separation'] == 2
    assert report['complete_separation_intervals_s'] == [(.2, .21), (.4, .5)]
