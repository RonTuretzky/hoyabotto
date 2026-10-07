"""Explicit camera additions preserve all physical model elements."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from carton.folding_additional_view_profiles import (
    add_additional_view_camera, additional_view_camera_declaration,
    verify_additional_view_camera,
)


def scene():
    return ET.fromstring('<mujoco><worldbody><camera name="front" pos="0 -.4 .5" fovy="48"/>'
                         '<body name="robot"><geom name="arm" type="box" size=".1 .1 .1"/>'
                         '</body><geom name="table" type="box" size=".6 .6 .02"/></worldbody></mujoco>')


def model_camera(name):
    mujoco = pytest.importorskip('mujoco')
    root = scene(); add_additional_view_camera(root, name)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    camera = model.camera(name)
    return SimpleNamespace(fovy=camera.fovy.copy(), pos=camera.pos.copy(), quat=camera.quat.copy(),
                           bodyid=camera.bodyid.copy(), mode=camera.mode.copy())


def test_default_front_is_exact_noop():
    root = scene(); original = ET.tostring(root)
    declaration = add_additional_view_camera(root, 'front')
    assert ET.tostring(root) == original
    assert declaration['physical_installation_verified'] is False


@pytest.mark.parametrize('name', ['front_left_back', 'front_right_back'])
def test_named_camera_only_appends_requested_camera_and_binds_nominal_pose(name):
    root = scene(); original = ET.tostring(root)
    declaration = add_additional_view_camera(root, name)
    cameras = root.findall('.//camera')
    assert len(cameras) == 2 and cameras[-1].get('name') == name
    root.find('worldbody').remove(cameras[-1])
    assert ET.tostring(root) == original
    assert verify_additional_view_camera(name, model_camera(name)) == declaration
    assert declaration['physical_installation_verified'] is False
    assert declaration['mounting_hardware_modelled'] is False


def test_duplicate_camera_is_not_replaced():
    root = scene(); add_additional_view_camera(root, 'front_left_back')
    with pytest.raises(ValueError, match='duplicate'):
        add_additional_view_camera(root, 'front_left_back')


@pytest.mark.parametrize('name', ['overhead', 'side', '', None])
def test_unsupported_profiles_refuse(name):
    with pytest.raises(ValueError): additional_view_camera_declaration(name)


@pytest.mark.parametrize('field,value', [('pos', [0, -.85, 1.05]), ('quat', [1., 0, 0, 0]),
    ('fovy', [60.]), ('bodyid', [1]), ('mode', [1]), ('pos', [np.nan, -.85, 1.05])])
def test_changed_nominal_pose_cannot_keep_profile_identity(field, value):
    camera = model_camera('front_left_back'); setattr(camera, field, np.asarray(value))
    with pytest.raises(ValueError): verify_additional_view_camera('front_left_back', camera)


def test_nondefault_profile_cannot_borrow_intrinsics_only_facade():
    with pytest.raises(ValueError, match='pose metadata'):
        verify_additional_view_camera('front_left_back', SimpleNamespace(fovy=[48.]))
