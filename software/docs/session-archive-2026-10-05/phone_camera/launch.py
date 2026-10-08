"""Start the camera server without replacing the phone's trusted certificate."""
import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

base=Path(__file__).resolve().parent
cfg=json.loads((base/'config.json').read_text())
try:
    request=urllib.request.Request('http://127.0.0.1:8876/api/status',headers={'Authorization':'Bearer '+cfg['viewer_token']})
    with urllib.request.urlopen(request,timeout=2) as r:
        if r.status==200:
            print('Robot camera server is already running. Open the phone setup link or scan its QR code.')
            sys.exit(0)
except (OSError,urllib.error.URLError):
    pass
print('Keep this Terminal window open while using the phone camera. Press Control-C to stop the camera server.')
print('Scan Robot-Camera-QR.png from the outputs folder on a phone using the same Wi-Fi.')
os.execv(sys.executable,[sys.executable,str(base/'server.py')])
