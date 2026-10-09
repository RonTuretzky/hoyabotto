"""Wrist-camera manifests written by the native capture publisher; reads files only, never opens a camera.

The publisher (work/capture-single, built from software/docs/session-archive-2026-10-05/capture-single.swift)
writes <name>.json next to an immutable hashed JPEG. Each wrist is pinned to its AVFoundation uniqueID so
a swapped or re-enumerated camera is refused rather than mislabelled.
"""
import json,os,re,signal,subprocess,threading,time
from pathlib import Path
ENV={'right_wrist':'XLEROBOT_RIGHT_WRIST_ID','left_wrist':'XLEROBOT_LEFT_WRIST_ID'}
WRIST_CAMERA_IDS={'right_wrist':os.environ.get(ENV['right_wrist'],'0x12200005a39230'),
                  'left_wrist':os.environ.get(ENV['left_wrist'],'0x12140005a39230')}
IDENTITY_VERIFIED={'right_wrist':True,'left_wrist':True}  # False when left/right was auto-assigned
# The head's old USB camera was the same model as the wrists. Its ID is a port path, so it is a saved setting
# (work/wrist-cameras.json head_camera_id, set with robot_admin.py wrist-ids), never a hard-coded port. Since
# 2026-10-09 the head camera is the OAK (the USB head camera was removed): a saved ID that the Mac no longer lists is
# dropped by prune_head_camera, quietly; the OAK is never a wrist candidate anyway (NOT_A_WRIST).
HEAD_CAMERA_ID=os.environ.get('XLEROBOT_HEAD_CAMERA_ID') or None
NOT_A_WRIST=re.compile(r'iphone|ipad|facetime|desk view|continuity|oak|luxonis|depthai|macbook|built-in|virtual|obs',re.I)
CONFIG='wrist-cameras.json'
FRESH_S=1


def configure(root):
    """Load the wrist IDs the restart script detected (work/wrist-cameras.json); environment variables still win."""
    global HEAD_CAMERA_ID
    try:saved=json.loads((Path(root)/'work'/CONFIG).read_text())
    except (OSError,ValueError):return
    if not os.environ.get('XLEROBOT_HEAD_CAMERA_ID') and isinstance(saved.get('head_camera_id'),str):HEAD_CAMERA_ID=saved['head_camera_id']
    for name in WRIST_CAMERA_IDS:
        entry=saved.get(name) or {}
        if os.environ.get(ENV[name]) or not isinstance(entry.get('camera_id'),str):continue
        WRIST_CAMERA_IDS[name]=entry['camera_id'];IDENTITY_VERIFIED[name]=entry.get('identity_verified') is True


def prune_head_camera(listed):
    """Forget a saved USB head camera ID the Mac no longer lists (removed for the OAK on 2026-10-09). An ID set by
    XLEROBOT_HEAD_CAMERA_ID is left alone. Returns the dropped ID, or None."""
    global HEAD_CAMERA_ID
    available={d.get('camera_id') for d in listed if isinstance(d,dict)}
    if HEAD_CAMERA_ID and HEAD_CAMERA_ID not in available and not os.environ.get('XLEROBOT_HEAD_CAMERA_ID'):
        dropped,HEAD_CAMERA_ID=HEAD_CAMERA_ID,None;return dropped
    return None


def resolve_ids(listed,current=None,verified=None):
    """Match wrists to the cameras the Mac lists now. A configured ID that is still present is kept; a wrist whose
    ID disappeared gets a remaining external camera, marked unverified because left/right cannot be told apart here."""
    current=dict(current or WRIST_CAMERA_IDS);verified=dict(verified or IDENTITY_VERIFIED)
    available={d.get('camera_id'):d.get('name','') for d in listed if isinstance(d,dict)}
    candidates=sorted(i for i,name in available.items() if i and not NOT_A_WRIST.search(name or '') and i!=HEAD_CAMERA_ID)
    ids,ok={},{}
    for name in ('right_wrist','left_wrist'):
        if current.get(name) in available:ids[name]=current[name];ok[name]=verified.get(name,False)
    free=[i for i in candidates if i not in ids.values()]
    # Same hub port path on a different USB bus (e.g. hub moved to another Mac port) is the same camera:
    # uniqueID = 0x + 2-digit bus + port path + vendor/product.
    for name in ('right_wrist','left_wrist'):
        old=current.get(name) or ''
        same_path=[i for i in free if len(i)==len(old) and len(old)>4 and i[4:]==old[4:]]
        if name not in ids and len(same_path)==1:ids[name]=same_path[0];ok[name]=verified.get(name,False);free.remove(same_path[0])
    for name in ('right_wrist','left_wrist'):
        if name not in ids and free:ids[name]=free.pop(0);ok[name]=False
    missing=[n for n in ('right_wrist','left_wrist') if n not in ids]
    return ids,ok,missing


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


_revive_lock=threading.Lock();_last_revive={}
REVIVE_EVERY_S=10;REVIVE_WAIT_S=8


def capture_pids(camera_id):
    """Native capture publishers (executable named capture*) started with this camera ID."""
    out=subprocess.run(['ps','-axo','pid=,args='],capture_output=True,text=True).stdout;found=[]
    for line in out.splitlines():
        parts=line.split()
        if len(parts)>2 and Path(parts[1]).name.startswith('capture') and any(a.endswith('='+camera_id) for a in parts[2:]):found.append(int(parts[0]))
    return found


def revive(name,root,clock=time.time,sleep=time.sleep):
    """On-demand restart of a wrist publisher whose camera stopped sending frames (the left wrist stalls about 15 s
    after each start: a USB fault). Returns True once a fresh frame exists. At most one attempt per 10 s per camera."""
    work=Path(root)/'work';capture=work/'capture-single';stream=work/'wrist-camera-stream';cid=WRIST_CAMERA_IDS[name]
    with _revive_lock:
        if clock()-_last_revive.get(name,0)<REVIVE_EVERY_S or not capture.exists():return False
        _last_revive[name]=clock()
        for pid in capture_pids(cid):
            try:os.kill(pid,signal.SIGTERM)
            except OSError:pass
        sleep(.5);stream.mkdir(parents=True,exist_ok=True)
        with open(stream/(name+'.log'),'ab') as log:
            subprocess.Popen([str(capture),str(stream),f'{name}={cid}'],cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        deadline=clock()+REVIVE_WAIT_S
        while clock()<deadline:
            try:select_wrist_manifest(name,wrist_dirs(root));return True
            except (RuntimeError,ValueError):sleep(.2)
        return False


def setup_report(root):
    """Last wrist-camera setup result written by the restart script (messages, IDs, freshness), for remote diagnosis."""
    try:return json.loads((Path(root)/'work'/'wrist-camera-setup.json').read_text())
    except (OSError,ValueError):return {'available':False,'note':'restart script has not recorded a camera setup yet'}


def wrist_status(dirs,now=None):
    now=time.time() if now is None else now;result={}
    for name in WRIST_CAMERA_IDS:
        try:
            folder,meta=select_wrist_manifest(name,dirs,now)
            result[name]={'available':True,'fresh':True,'age_s':now-meta['captured_at'],'captured_at':meta['captured_at'],'seq':meta.get('seq'),
                          'stream_id':meta.get('stream_id'),'camera_id':meta['camera_id'],'folder':str(folder),'robot_frame_calibrated':False,
                          'identity_verified':IDENTITY_VERIFIED[name]}
        except (RuntimeError,ValueError) as exc:
            result[name]={'available':'stale' in str(exc),'fresh':False,'error':str(exc)}
    return result
