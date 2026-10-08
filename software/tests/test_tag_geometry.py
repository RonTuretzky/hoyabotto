"""Metric pose evidence: projection, distortion, ambiguity and frame binding."""
import copy

import cv2
import numpy as np
import pytest

from farm.perception.gemma_tags import TagObserver, TagRobot
from farm.perception.tag_geometry import TagGeometry, camera_calibration, estimate_square, square_points
from test_gemma_tags import envelope, CameraRobot


K = np.array([[500., 0, 320], [0, 505, 240], [0, 0, 1]])


def config():
    return {"schema": 1, "family": "tag36h11", "camera_ids": ["oak-test"],
            "tags": {str(i): {"black_square_mm": size, "source": "test measured width"}
                     for i, size in ((1, 60), (2, 40), (3, 40))}}


def calibrated_payload():
    p = envelope(phone=False)
    p['result']['cameras']['oak'].update(intrinsics=K.tolist(), projection='rectified_pinhole',
        coordinate_frame='test_optical', distortion_coefficients=[-4, 16, 0, 0, -20])
    p['images'][0]['projection'] = 'rectified_pinhole'
    return p


def projected(size=40, distortion=None, rotation=(.55, .35, .1)):
    distortion = np.zeros(5) if distortion is None else distortion
    translation = np.array([.03, -.02, .55])
    pixels, _ = cv2.projectPoints(square_points(size/1000), np.array(rotation, dtype=float), translation, K, distortion)
    return pixels[:, 0], translation


def test_known_tilted_square_recovers_camera_position_and_orientation():
    points, position = projected()
    result = estimate_square(points, 40, K, np.zeros(5))
    assert result['center_camera_mm'] == pytest.approx(position*1000, abs=1e-5)
    assert result['reprojection_rms_px'] < 1e-7
    assert not result['orientation_ambiguous']
    rotation, _ = cv2.Rodrigues(np.array([.55, .35, .1]))
    assert np.asarray(result['camera_from_tag'])[:3, :3] == pytest.approx(rotation, abs=1e-6)


def test_print_scaling_changes_metric_distance_proportionately():
    points, position = projected()
    result = estimate_square(points, 60, K, np.zeros(5))
    assert result['center_camera_mm'] == pytest.approx(position*1500, abs=1e-5)


def test_raw_distortion_is_applied_but_not_applied_again_to_rectified_rgb():
    d = np.array([-.2, .06, .001, -.002, .01])
    points, position = projected(distortion=d)
    result = estimate_square(points, 40, K, d)
    assert result['center_camera_mm'] == pytest.approx(position*1000, abs=.01)
    p = calibrated_payload()
    _, effective, _ = camera_calibration(p['result']['cameras']['oak'], p['images'][0], (480, 640, 3))
    assert np.all(effective == 0)
    p['images'][0]['projection'] = p['result']['cameras']['oak']['projection'] = 'camera_pinhole_with_factory_distortion'
    _, effective, _ = camera_calibration(p['result']['cameras']['oak'], p['images'][0], (480, 640, 3))
    assert effective.tolist() == [-4, 16, 0, 0, -20]


def test_small_square_with_two_similar_fits_does_not_claim_orientation():
    points, _ = projected(rotation=(.12, .1, .4))
    result = estimate_square(points, 40, K, np.zeros(5))
    assert result['orientation_ambiguous']
    assert result['camera_from_tag'] is None
    assert result['center_camera_mm'][2] > 0


@pytest.mark.parametrize('mutation', [
    lambda p: p['result']['cameras']['oak'].update(intrinsics=None),
    lambda p: p['result']['cameras']['oak'].update(projection='unknown'),
    lambda p: p['images'][0].update(projection='raw'),
    lambda p: p['result']['cameras']['oak'].pop('camera_id'),
    lambda p: p['result']['cameras']['oak'].update(coordinate_frame=None),
])
def test_invalid_metric_metadata_preserves_2d_but_never_invents_distance(mutation):
    p = calibrated_payload()
    mutation(p)
    result = TagObserver(clock=lambda: 1000.1, geometry=config()).observe(p, ['oak'])
    assert result['ok'] and result['result']['metric_pose_available'] is False
    row = result['result']['observations']['oak']
    assert row['gripper_to_paddle_px'] is not None
    assert row['pose_3d']['status'] == 'UNAVAILABLE'
    assert row['pose_3d']['gripper_to_paddle_mm'] is None


def test_configured_geometry_is_live_camera_relative_and_not_robot_target():
    result = TagObserver(clock=lambda: 1000.1, geometry=config()).observe(calibrated_payload(), ['oak'])
    assert result['result']['metric_pose_available']
    metric = result['result']['observations']['oak']['pose_3d']
    assert not metric['robot_frame_calibrated'] and not metric['physical_accuracy_validated']
    assert not metric['gripper_to_paddle_mm']['robot_motion_target']
    assert metric['gripper_to_paddle_mm']['tag_center_distance_mm'] > 0


def test_confirmed_mount_reaches_metric_observations_and_config_fingerprint():
    cfg = config()
    mount = dict(arm='right', body='fixed_gripper_housing', source='user confirmation')
    old = TagObserver(clock=lambda: 1000.1, geometry=cfg).observe(calibrated_payload(), ['oak'])
    cfg['tags']['2']['mount'] = mount
    new = TagObserver(clock=lambda: 1000.1, geometry=cfg).observe(calibrated_payload(), ['oak'])
    old_pose = old['result']['observations']['oak']['pose_3d']
    new_pose = new['result']['observations']['oak']['pose_3d']
    assert new_pose['geometry_config_sha256'] != old_pose['geometry_config_sha256']
    assert next(t for t in new_pose['tags'] if t['tag_id'] == 2)['mount'] == mount


@pytest.mark.parametrize('mount', [False, 'right', {},
    dict(arm='right', body='unknown', source='user'),
    dict(arm='right', body='fixed_gripper_housing', source=' ')])
def test_incomplete_mount_configuration_is_rejected(mount):
    cfg = config()
    cfg['tags']['2']['mount'] = mount
    with pytest.raises(ValueError, match='gripper mount'):
        TagGeometry(cfg)


def test_stale_image_removes_metric_geometry_too():
    p = calibrated_payload()
    p['result']['cameras']['oak']['rgb_captured_at'] = 900
    result = TagObserver(clock=lambda: 1000.1, geometry=config()).observe(p, ['oak'])
    assert not result['ok'] and not result['result']['metric_pose_available']
    assert result['result']['observations']['oak']['pose_3d'] is None


def test_unconfigured_camera_and_missing_size_are_not_guessed():
    cfg = config()
    cfg['camera_ids'] = ['different-camera']
    obs = TagObserver(clock=lambda: 1000.1, geometry=cfg)
    assert not obs.observe(calibrated_payload(), ['oak'])['result']['metric_pose_available']
    cfg = config()
    del cfg['tags']['3']
    result = TagObserver(clock=lambda: 1000.1, geometry=cfg).observe(calibrated_payload(), ['oak'])
    assert result['result']['observations']['oak']['pose_3d']['gripper_to_paddle_mm'] is None


@pytest.mark.parametrize('size', [True, 0, 400, float('nan')])
def test_unusable_sizes_rejected_before_measurement(size):
    cfg = config()
    cfg['tags']['2']['black_square_mm'] = size
    with pytest.raises(ValueError):
        TagGeometry(cfg)


def test_local_commissioning_file_is_opt_in_and_cannot_be_set_by_model(tmp_path):
    import json
    robot = CameraRobot()
    robot.config = tmp_path/'robot.json'
    assert TagRobot(robot).observer.geometry is None
    (tmp_path/'apriltag-geometry.json').write_text(json.dumps(config()))
    tagged = TagRobot(robot)
    assert tagged.observer.geometry is not None
    assert 'millimetres' in tagged.catalog()['tools'][-1]['function']['description']
    with pytest.raises(ValueError):
        tagged.call('robot_get_tags', {'tag_size': 100})


# OAK --wide: raw lens image, factory OpenCV rational coefficients (14 values).
OAK_WIDE_D = np.array([-.32, .11, .0012, -.0008, -.018, .04, -.01, .002, 0, 0, 0, 0, 0, 0])


def wide_payload(stream='stream1', **camera):
    p = calibrated_payload()
    p['images'][0]['projection'] = 'camera_pinhole_with_factory_distortion'
    p['images'][0]['stream_id'] = stream
    p['result']['cameras']['oak'].update({'projection': 'camera_pinhole_with_factory_distortion', 'stream_id': stream,
        'distortion_coefficients': OAK_WIDE_D.tolist(), 'distortion_model': 'CameraModel.Perspective', **camera})
    return p


@pytest.mark.parametrize('rotation,translation', [((.55, .35, .1), (.03, -.02, .55)),
                                                  ((.3, -.4, .2), (.28, .16, .45)),
                                                  ((.3, -.4, .2), (.35, .2, .45))])  # tag at the wide-FOV corner
def test_wide_projection_undistorts_corners_before_pose(rotation, translation):
    # OpenCV's built-in IPPE undistortion (5 iterations) left ~1mm here; the
    # converged undistortion must recover the pose exactly.
    rotation, translation = np.array(rotation, float), np.array(translation)
    pixels, _ = cv2.projectPoints(square_points(.04), rotation, translation, K, OAK_WIDE_D)
    p = wide_payload()
    k, d, binding = camera_calibration(p['result']['cameras']['oak'], p['images'][0], (480, 640, 3))
    assert binding['projection'] == 'camera_pinhole_with_factory_distortion' and d.tolist() == OAK_WIDE_D.tolist()
    result = estimate_square(pixels[:, 0], 40, k, d)
    assert result['center_camera_mm'] == pytest.approx(translation*1000, abs=1e-3)
    assert result['reprojection_rms_px'] < 1e-6
    assert np.asarray(result['camera_from_tag'])[:3, :3] == pytest.approx(cv2.Rodrigues(rotation)[0], abs=1e-6)
    # Treating the raw pixels as rectified (no undistortion) would misplace the tag.
    try:
        wrong = estimate_square(pixels[:, 0], 40, k, np.zeros(5))['center_camera_mm']
        assert np.linalg.norm(np.subtract(wrong, translation*1000)) > 1
    except ValueError as exc:
        assert 'reprojection' in str(exc)


def test_camera_geometry_hash_survives_restart_but_not_projection_or_intrinsics_change():
    def metric(p):
        row = TagObserver(clock=lambda: 1000.1, geometry=config()).observe(p, ['oak'])['result']['observations']['oak']
        assert row['pose_3d']['status'] == 'CAMERA_RELATIVE_ESTIMATE'
        return row['pose_3d']
    wide = metric(wide_payload())
    assert wide['camera_calibration']['effective_distortion'] == OAK_WIDE_D.tolist()
    assert 'stream_id' not in wide['camera_calibration']
    assert metric(wide_payload(stream='restarted'))['calibration_sha256'] == wide['calibration_sha256']
    assert metric(calibrated_payload())['calibration_sha256'] != wide['calibration_sha256']
    assert metric(wide_payload(intrinsics=(K*[[1.01], [1.01], [1]]).tolist()))['calibration_sha256'] != wide['calibration_sha256']


def test_non_pinhole_lens_model_is_refused_for_wide_projection():
    p = wide_payload(distortion_model='CameraModel.Fisheye')
    with pytest.raises(ValueError, match='Perspective'):
        camera_calibration(p['result']['cameras']['oak'], p['images'][0], (480, 640, 3))
