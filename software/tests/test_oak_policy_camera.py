"""Projection and fail-closed input checks for the commissioned OAK view."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton.oak_policy_camera import contract_digest, policy_maps, preprocess


@pytest.fixture
def contract():
    return json.loads((Path(__file__).parents[1] / 'config/oak-policy-camera-20261009.json').read_text())


def manifest(contract):
    source = copy.deepcopy(contract['source'])
    source['width'], source['height'] = source['size_wh']
    source['rgb_lens_position'] = source['calibrated_lens_position']
    return source


def test_policy_rays_reproject_to_the_samples_used(contract):
    mx, my = policy_maps(contract)
    # Probe center, corners and asymmetric interior rays. Independently project the
    # virtual camera's 3-D rays through the complete source lens model.
    pixels = np.array([[0, 0], [319, 239], [160, 120], [73, 191], [301, 18]], dtype=float)
    rays = (np.linalg.inv(contract['policy']['intrinsics']) @
            np.c_[pixels, np.ones(len(pixels))].T).T
    projected, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3),
                                   np.array(contract['source']['intrinsics']),
                                   np.array(contract['source']['distortion_coefficients']))
    sampled = np.array([[mx[int(y), int(x)], my[int(y), int(x)]] for x, y in pixels])
    np.testing.assert_allclose(sampled, projected[:, 0], atol=4e-5, rtol=0)


def test_bgr_capture_becomes_rgb_policy_without_missing_pixels(contract):
    raw = np.zeros((780, 1040, 3), dtype=np.uint8)
    raw[:] = [17, 63, 241]
    result = preprocess(raw, manifest(contract), contract, input_color_order='BGR')
    assert result.shape == (240, 320, 3)
    assert np.all(result == [241, 63, 17])


@pytest.mark.parametrize('key,value', [('config_sha256', 'old-stream'),
    ('rgb_lens_position', 80), ('camera_id', 'other-oak'), ('width', 640)])
def test_changed_camera_is_rejected(contract, key, value):
    meta = manifest(contract)
    meta[key] = value
    with pytest.raises(ValueError):
        preprocess(np.zeros((780, 1040, 3), dtype=np.uint8), meta, contract)


def test_unreviewed_contract_change_is_rejected(contract):
    contract['policy']['intrinsics'][0][0] += 1
    with pytest.raises(ValueError, match='checksum'):
        preprocess(np.zeros((780, 1040, 3), dtype=np.uint8), manifest(contract), contract)


def test_factory_distortion_pole_cannot_be_used_for_wider_policy(contract):
    contract['policy']['intrinsics'][0][0] = 244.927
    contract['policy']['intrinsics'][1][1] = 244.927
    contract['contract_sha256'] = contract_digest(contract)
    with pytest.raises(ValueError, match='unstable distortion'):
        policy_maps(contract)
