"""Physics regressions for G4 rest support and evaluator-only evidence."""
from pathlib import Path

import numpy as np
import pytest

from planter.g4_sim import G4Simulation
from tools.diagnose_g4_staging import settled_diagnostic


SOURCE = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot')
CAD = Path('/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4')
pytestmark = pytest.mark.skipif(not SOURCE.exists() or not CAD.exists(), reason='Local SO101/G4 assets unavailable')


def test_legacy_staging_failure_and_explicit_support_repair(tmp_path):
    legacy = settled_diagnostic(SOURCE, CAD, tmp_path / 'legacy',
                                dict(rest_height=.04), (-.004, .006))
    repaired = settled_diagnostic(SOURCE, CAD, tmp_path / 'repaired',
                                  dict(rest_height=.04, rest_near_edge=.393), (-.004, .006))
    assert not legacy['stable']
    assert legacy['grip_drift_mm'] > 30
    assert repaired['stable']
    assert repaired['grip_drift_mm'] < .1
    assert repaired['staging_support_normal_force_n'] > .1
    assert not repaired['initial_intersections']


def test_wider_rest_does_not_imply_unbounded_placement_robustness(tmp_path):
    result = settled_diagnostic(SOURCE, CAD, tmp_path, dict(rest_height=.04, rest_near_edge=.393), (-.008, .008))
    assert not result['stable']
    assert result['grip_drift_mm'] > 30


def test_defaults_remain_legacy_and_contact_normals_are_world_evidence(tmp_path):
    sim = G4Simulation(SOURCE, CAD, tmp_path)
    rest = sim.model.geom('staging_rest')
    assert np.allclose(rest.pos, [.425, .025, -.05])
    assert np.allclose(rest.size, [.030, .055, .010])
    state = sim.score_state()
    rotation = np.array(state['gripper_world_rotation'])
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
    contacts = [c for c in state['contacts'] if c['geoms'][0] == 'staging_rest' and c['normal_force_n'] > .005]
    assert contacts
    for contact in contacts:
        normal = np.array(contact['normal_world_geom1_to_geom2'])
        # Compliant contact permits a slight settled tilt, but support points
        # up into the tool rather than down into the fixed rest.
        assert np.isclose(np.linalg.norm(normal), 1., atol=1e-8)
        assert normal[2] > .999
        assert len(contact['position_world_m']) == 3
    assert sim.model.nu == 6 and sim.model.neq == 0 and sim.model.nmocap == 0


@pytest.mark.parametrize('geometry', [dict(rest_near_edge=.46), dict(rest_half_width=0),
                                      dict(rest_height=-.02), dict(rest_far_edge=float('nan'))])
def test_invalid_support_geometry_refuses_before_simulation(tmp_path, geometry):
    with pytest.raises(ValueError, match='Staging rest dimensions'):
        G4Simulation(SOURCE, CAD, tmp_path, **geometry)
