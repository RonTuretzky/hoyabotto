"""Wrist-camera manifests written by the native capture publisher; reads files only, never opens a camera.

The publisher (work/capture-single, built from software/docs/session-archive-2026-10-05/capture-single.swift)
writes <name>.json next to an immutable hashed JPEG. Each wrist is pinned to its AVFoundation uniqueID so
a swapped or re-enumerated camera is refused rather than mislabelled.
"""
import json,os,time
from pathlib import Path
WRIST_CAMERA_IDS={'right_wrist':os.environ.get('XLEROBOT_RIGHT_WRIST_ID','0x12200005a39230'),
                  'left_wrist':os.environ.get('XLEROBOT_LEFT_WRIST_ID','0x12140005a39230')}
FRESH_S=1


def wrist_dirs(root):
    configured=os.environ.get('XLEROBOT_WRIST_DIRS')
    if configured:return [Path(p) for p in configured.split(os.pathsep) if p]
    # Own wrist stream first, then the older head/wrist task stream if it is still publishing.
    return [Path(root)/'work/wrist-camera-stream',Path(root)/'work/robot-camera-stream']


def select_wrist_manifest(name,dirs,now=None):
    """Freshest manifest for this wrist across the stream folders, with its identity checked."""
    if name not in WRIST_CAMERA_IDS:raise ValueError('Unsupported wrist camera: '+str(name))
    now=time.time() if now is None else now;found=[];errors=[]
    for folder in dirs:
        try:
            meta=json.loads((Path(folder)/(name+'.json')).read_text())
            if meta.get('camera_id')!=WRIST_CAMERA_IDS[name]:
                errors.append(f"{folder}: camera_id {meta.get('camera_id')} is not the expected {name} {WRIST_CAMERA_IDS[name]}");continue
            found.append((meta['captured_at'],Path(folder),meta))
        except FileNotFoundError:continue
        except (OSError,ValueError,KeyError,TypeError) as exc:errors.append(f'{folder}: {exc}')
    if not found:raise RuntimeError(name+' manifest unavailable'+(': '+'; '.join(errors) if errors else ' (no publisher output)'))
    stamp,folder,meta=max(found,key=lambda f:f[0])
    if not 0<=now-stamp<=FRESH_S:raise RuntimeError(f'{name} image is stale age_s={round(now-stamp,3)}')
    return folder,meta


def wrist_status(dirs,now=None):
    now=time.time() if now is None else now;result={}
    for name in WRIST_CAMERA_IDS:
        try:
            folder,meta=select_wrist_manifest(name,dirs,now)
            result[name]={'available':True,'fresh':True,'age_s':now-meta['captured_at'],'captured_at':meta['captured_at'],'seq':meta.get('seq'),
                          'stream_id':meta.get('stream_id'),'camera_id':meta['camera_id'],'folder':str(folder),'robot_frame_calibrated':False}
        except (RuntimeError,ValueError) as exc:
            result[name]={'available':'stale' in str(exc),'fresh':False,'error':str(exc)}
    return result
