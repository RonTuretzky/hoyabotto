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
print('Wrist cameras: freshest folder, 1 s staleness, pinned identity and status checks passed; no camera access')
