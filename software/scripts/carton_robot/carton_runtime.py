"""Deployment paths and registered identities; no private machine defaults."""
import json
import os
import sys
from pathlib import Path

PACKAGE_SOFTWARE = Path(__file__).resolve().parents[2]
LIVE = Path(os.environ.get('CARTON_LIVE_SOFTWARE', PACKAGE_SOFTWARE)).expanduser().resolve()
SOFTWARE = Path(os.environ.get('CARTON_UTILITY_SOFTWARE', PACKAGE_SOFTWARE)).expanduser().resolve()
ROOT = Path(os.environ.get('CARTON_WORKSPACE_ROOT', PACKAGE_SOFTWARE.parent)).expanduser().resolve()
SESSION = Path(os.environ.get('CARTON_SESSION_DIR', ROOT/'work/carton-session')).expanduser().resolve()
FRAMES = Path(os.environ.get('CARTON_FRAMES_DIR', ROOT/'work/robot-camera-stream')).expanduser().resolve()
PROFILE = os.environ.get('CARTON_PROFILE', 'paper-tray-v0')
sys.path[:0] = [str(LIVE), str(SOFTWARE)]

def camera_identity(name):
    """Require an independently registered expected ID, never infer from feed."""
    identity = os.environ.get('CARTON_'+name.upper()+'_ID')
    if not identity:
        config = json.loads((SESSION/'config.json').read_text())
        identity = config.get('camera', {}).get('identities', {}).get(name)
    if not isinstance(identity, str) or not identity.strip():
        raise RuntimeError('Registered camera identity required for '+name)
    return identity

def calibration_file():
    explicit=os.environ.get('CARTON_CALIBRATION')
    if explicit:return Path(explicit).expanduser().resolve()
    from farm.config import load_profile
    cfg=load_profile(PROFILE).robot
    folder=Path(cfg.calibration_dir).expanduser() if cfg.calibration_dir else Path.home()/'.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels'
    return folder/(cfg.id+'.json')
