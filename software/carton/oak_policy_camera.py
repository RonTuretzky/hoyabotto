"""Explicit OAK-to-policy projection shared by training and physical runners.

No robot access. Output is RGB; OpenCV callers must declare BGR input.
The caller must pass the manifest for the same immutable source frame.
"""
from __future__ import annotations
import hashlib
import json
import numpy as np


def contract_digest(contract):
    body={k:v for k,v in contract.items() if k!='contract_sha256'}
    return hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def validate_source(image, manifest, contract):
    if contract.get('schema')!='xlerobot-oak-policy-camera/1':
        raise ValueError('Unsupported OAK policy camera contract')
    if contract.get('contract_sha256')!=contract_digest(contract):
        raise ValueError('OAK camera contract checksum mismatch')
    s=contract['source'];w,h=s['size_wh']
    if image.dtype!=np.uint8 or image.shape!=(h,w,3):
        raise ValueError('OAK input shape/type differs from trained source')
    for key in ('camera_id','rgb_sensor_mode','calibrated_lens_position','config_sha256'):
        if manifest.get(key)!=s[key]:raise ValueError('OAK source mismatch: '+key)
    if [manifest.get('width'),manifest.get('height')]!=[w,h]:raise ValueError('OAK manifest size mismatch')
    if manifest.get('rgb_lens_position')!=s['calibrated_lens_position']:
        raise ValueError('OAK physical focus readback differs from calibration')
    for key in ('intrinsics','distortion_coefficients'):
        if not np.allclose(manifest.get(key,[]),s[key],atol=1e-6,rtol=0):
            raise ValueError('OAK projection mismatch: '+key)


def policy_maps(contract):
    import cv2
    s=contract['source'];p=contract['policy'];w,h=s['size_wh']
    mx,my=cv2.initUndistortRectifyMap(np.array(s['intrinsics']),np.array(s['distortion_coefficients']),
        np.eye(3),np.array(p['intrinsics']),tuple(p['size_wh']),cv2.CV_32FC1)
    # Full factory rational D must not be extrapolated across its pole near r=.718.
    # Reject missing rays and folded maps, not just finite output images with filled black holes.
    if not (np.isfinite(mx).all() and np.isfinite(my).all() and
            (mx>=0).all() and (mx<w-1).all() and (my>=0).all() and (my<h-1).all() and
            (np.diff(mx,axis=1)>0).all() and (np.diff(my,axis=0)>0).all()):
        raise ValueError('Policy projection has missing rays or an unstable distortion map')
    return mx,my


def preprocess(image, manifest, contract, *, input_color_order="RGB"):
    import cv2
    if input_color_order not in ("RGB", "BGR") or contract['policy'].get('color_order') != 'RGB':
        raise ValueError('Policy requires RGB output and explicit RGB/BGR input')
    validate_source(image,manifest,contract)
    mx,my=policy_maps(contract)
    if input_color_order == "BGR":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return cv2.remap(image,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
