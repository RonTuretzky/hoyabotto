import json,tempfile,hashlib
from pathlib import Path
from wrist_cameras import select_wrist_manifest,wrist_status,WRIST_CAMERA_IDS
with tempfile.TemporaryDirectory() as tmp:
 own,old=Path(tmp,'own'),Path(tmp,'old');own.mkdir();old.mkdir()
 def publish(folder,name,at,camera_id=None):
  data=b'jpeg'+str(at).encode();image=f'{name}-s-{at}.jpg';(folder/image).write_bytes(data)
  (folder/(name+'.json')).write_text(json.dumps({'schema':1,'camera_id':camera_id or WRIST_CAMERA_IDS[name],'stream_id':'s','seq':int(at),'captured_at':at,'image':image,'sha256':hashlib.sha256(data).hexdigest()}))
 publish(own,'right_wrist',100.0);publish(old,'right_wrist',100.5)
 folder,meta=select_wrist_manifest('right_wrist',[own,old],now=101.0);assert folder==old and meta['captured_at']==100.5 # freshest folder wins
 try:select_wrist_manifest('right_wrist',[own,old],now=102.0)
 except RuntimeError as x:assert 'stale' in str(x)
 else:raise AssertionError('Stale wrist frame accepted')
 publish(own,'left_wrist',100.9,camera_id=WRIST_CAMERA_IDS['right_wrist']) # swapped camera
 try:select_wrist_manifest('left_wrist',[own,old],now=101.0)
 except RuntimeError as x:assert 'not the expected left_wrist' in str(x)
 else:raise AssertionError('Swapped wrist camera accepted')
 s=wrist_status([own,old],now=101.0);assert s['right_wrist']['fresh'] and not s['left_wrist']['fresh'] and 'error' in s['left_wrist']
 assert not wrist_status([Path(tmp,'missing')],now=101.0)['right_wrist']['available']
from wrist_cameras import resolve_ids
R,Lw,H='0x12200005a39230','0x12140005a39230','0x12400005a39230'
import wrist_cameras;wrist_cameras.HEAD_CAMERA_ID=H  # saved head_camera_id setting (no longer hard-coded)
cur={'right_wrist':R,'left_wrist':Lw};ok={'right_wrist':True,'left_wrist':True}
ids,ver,miss=resolve_ids([{'name':'USB Camera','camera_id':R},{'name':'USB Camera','camera_id':Lw},{'name':'USB Camera','camera_id':H},{'name':'FaceTime HD Camera','camera_id':'0xFT'}],cur,ok)
assert ids==cur and ver==ok and miss==[] # unchanged IDs stay verified
ids,ver,miss=resolve_ids([{'name':'USB Camera','camera_id':R},{'name':'USB Camera','camera_id':'0x13140005a39230'},{'name':'USB Camera','camera_id':H},{'name':'iPhone Camera','camera_id':'0xIP'}],cur,ok)
assert ids=={'right_wrist':R,'left_wrist':'0x13140005a39230'} and ver==ok and miss==[] # same port path on another bus: matched; head and iPhone skipped
ids,ver,miss=resolve_ids([{'name':'USB Camera','camera_id':R},{'name':'USB Camera','camera_id':'0x12300005a39230'},{'name':'USB Camera','camera_id':H}],cur,ok)
assert ids=={'right_wrist':R,'left_wrist':'0x12300005a39230'} and ver=={'right_wrist':True,'left_wrist':False} # different port: assigned but unverified
ids,ver,miss=resolve_ids([{'name':'USB Camera','camera_id':'0x13200005a39230'},{'name':'USB Camera','camera_id':'0x13140005a39230'},{'name':'USB Camera','camera_id':'0x13400005a39230'}],cur,ok)
assert ids=={'right_wrist':'0x13200005a39230','left_wrist':'0x13140005a39230'} and ver==ok # hub moved to another bus: same port paths, still verified
ids,ver,miss=resolve_ids([{'name':'USB Camera','camera_id':H}],cur,ok);assert ids=={} and miss==['right_wrist','left_wrist']
print('Wrist cameras: freshest folder, 1 s staleness, pinned identity, status and ID auto-detection checks passed; no camera access')
