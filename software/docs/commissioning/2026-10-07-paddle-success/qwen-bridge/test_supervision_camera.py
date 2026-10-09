"""The motor-enable camera gate accepts the head OAK stream when the phone overview feed is absent or stale."""
import json, tempfile, time
from pathlib import Path
import gemma_hardware_owner as O
d = Path(tempfile.mkdtemp()); O.PHONE_CAMERA = d / 'phone.json'; O.OAK_CAMERA = d / 'oak.json'
assert O.supervision_metadata() == {}                                     # neither feed: gate stays closed
now = time.time()
O.OAK_CAMERA.write_text(json.dumps({'captured_at': now, 'seq': 7}))
m = O.supervision_metadata(); assert m['source'] == 'oak' and m['received_at'] == now and m['seq'] == ('oak', 7)
O.PHONE_CAMERA.write_text(json.dumps({'received_at': now - 600, 'seq': 3}))  # stale phone: OAK still wins
assert O.supervision_metadata()['source'] == 'oak'
O.PHONE_CAMERA.write_text(json.dumps({'received_at': now + 1, 'seq': 4}))    # fresher phone is used
assert O.supervision_metadata()['source'] == 'phone'
from paddle_camera_gate import PaddleCameraGate
O.PHONE_CAMERA.unlink(); assert PaddleCameraGate(O.supervision_metadata).update() is True
O.OAK_CAMERA.write_text(json.dumps({'captured_at': now - 60, 'seq': 8}))
try: PaddleCameraGate(O.supervision_metadata).update(); raise SystemExit('stale OAK must not open the gate')
except RuntimeError: pass
print('supervision camera: ok')
