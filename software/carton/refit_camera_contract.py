"""Shared commissioned OAK projection for offline renders and inference adapters.

Optical intrinsics only: head/arm extrinsics remain explicit model assumptions.
"""
from pathlib import Path
import hashlib
import json

import numpy as np

from carton import oak_policy_camera as oak

DEFAULT_CONTRACT = Path(__file__).resolve().parents[1] / 'config/oak-policy-camera-20261009.json'
CONTRACT_FILE = 'oak-policy-camera.json'
METADATA_FILE = 'camera-contract-metadata.json'


def load_contract(path=DEFAULT_CONTRACT):
    contract = json.loads(Path(path).read_text())
    if (contract.get('schema') != 'xlerobot-oak-policy-camera/1' or
            contract.get('contract_sha256') != oak.contract_digest(contract)):
        raise ValueError('OAK contract schema/checksum mismatch')
    p = contract['policy']
    if (p['color_order'] != 'RGB' or not np.array_equal(p['rectification_rotation'], np.eye(3)) or
            np.any(p['distortion_coefficients'])):
        raise ValueError('Expected the rectified RGB pinhole contract')
    k = np.asarray(p['intrinsics'], float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or
            k[1, 1] <= 0 or k[0, 1] != 0 or k[1, 0] != 0 or
            not np.array_equal(k[2], [0, 0, 1])):
        raise ValueError('Unsupported virtual camera intrinsics')
    oak.policy_maps(contract)  # Reject missing/folded source rays before a job starts.
    return contract


def provenance(contract):
    return dict(contract_sha256=contract['contract_sha256'],
                helper_sha256=hashlib.sha256(Path(oak.__file__).read_bytes()).hexdigest(),
                renderer_adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                simulator_sampling='OpenGL half-integer centers shifted to OpenCV integer centers via frustum',
                physical_registration_verified=False,
                extrinsics='model assumptions; observed head ticks are not a calibrated transform')


def save_contract(directory, contract):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / CONTRACT_FILE).write_text(json.dumps(contract, indent=2) + '\n')
    (directory / METADATA_FILE).write_text(json.dumps(provenance(contract), indent=2) + '\n')


def trial_contract(trial):
    path = Path(trial) / 'run' / CONTRACT_FILE
    return load_contract(path) if path.exists() else None


def render_policy_camera(renderer, data, camera, option=None, contract=None):
    """Render the virtual pinhole directly; never reapply raw sensor distortion.

    OpenGL samples at pixel centers i+0.5, while contract K uses integer centers i.
    Adjust both GL eye frusta by half a pixel, including the flipped image y axis.
    The camera pose, lighting and collision geometry are untouched.
    """
    if option is None and contract is not None:
        import mujoco
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0  # collision hulls have separate CAD visuals
        option.geomgroup[4] = 0  # scripted teacher anchor is not a policy input
    renderer.update_scene(data, camera=camera, scene_option=option)
    if camera == 'front' and contract is not None:
        w, h = contract['policy']['size_wh']
        if (renderer.width, renderer.height) != (w, h):
            raise ValueError('Renderer size differs from policy camera contract')
        k = np.asarray(contract['policy']['intrinsics'])
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        for eye in renderer.scene.camera:
            n = eye.frustum_near
            eye.frustum_center = (w / 2 - cx - .5) * n / fx
            # MuJoCo stores the HALF width of its symmetric horizontal frustum.
            eye.frustum_width = w * n / (2 * fx)
            eye.frustum_top = (cy + .5) * n / fy
            eye.frustum_bottom = -(h - cy - .5) * n / fy
    return renderer.render().copy()


def verify_rendered_projection(contract=None):
    """Sample actual rasterized rays; runs on the selected local/EGL renderer."""
    import mujoco
    contract = load_contract() if contract is None else contract
    k = np.asarray(contract['policy']['intrinsics'])
    pixels = np.array([(160,120),(43.2,32.7),(290.3,204.1),(75.6,191.4)])
    geoms = ''
    for u, v in pixels:
        x, y, _ = np.linalg.inv(k) @ [u, v, 1]
        geoms += f'<geom type="sphere" pos="{x} {-y} -1" size=".009" rgba="1 0 0 1"/>'
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><visual><headlight ambient="1 1 1"
      diffuse="0 0 0" specular="0 0 0"/></visual><worldbody><camera name="front" fovy="45"/>
      {geoms}</worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    w,h = contract['policy']['size_wh']
    with mujoco.Renderer(model,h,w) as renderer:
        image = render_policy_camera(renderer,data,'front',contract=contract)
    observed = []
    for u,v in pixels:
        yy,xx = np.mgrid[int(v)-8:int(v)+9,int(u)-8:int(u)+9]
        weights = image[yy,xx,0].astype(float)
        if weights.sum() <= 0:
            raise RuntimeError('Projected test point is missing from render')
        observed.append([(xx*weights).sum()/weights.sum(),(yy*weights).sum()/weights.sum()])
    error = float(np.max(np.abs(np.asarray(observed)-pixels)))
    if not np.isfinite(error) or error > .2:
        raise RuntimeError(f'Rendered policy camera differs from integer-center K: {error} pixels')
    return dict(expected_pixels=pixels.tolist(),observed_pixels=observed,max_axis_error_px=error,
                tolerance_px=.2,passed=True,**provenance(contract))


def checkpoint_contract(checkpoint):
    """Load a local or Hub sidecar; only an absent legacy contract may fall back."""
    path = Path(checkpoint)
    if path.is_dir():
        file = path / CONTRACT_FILE
        return load_contract(file) if file.exists() else None
    from huggingface_hub import hf_hub_download
    from huggingface_hub.errors import EntryNotFoundError
    try:
        file = hf_hub_download(str(checkpoint), CONTRACT_FILE)
    except EntryNotFoundError:
        return None
    return load_contract(file)
