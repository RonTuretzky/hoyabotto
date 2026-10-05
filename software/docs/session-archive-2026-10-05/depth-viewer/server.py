"""Local camera viewer; reads existing capture manifests, never opens devices."""
import json,time,hashlib
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parents[2]
FRAMES=ROOT/'work/robot-camera-stream'
HTML='''<!doctype html><html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Live depth camera — aim the paddle</title>
<style>body{margin:0;background:#121820;color:#edf3fa;font:15px system-ui}header{padding:10px 16px}h1{font-size:20px;margin:0 0 8px}.view{position:relative;width:min(90vw,calc((100dvh - 180px)*1.7778));max-width:960px;margin:auto}img{display:block;width:100%;aspect-ratio:16/9;object-fit:contain;background:black}.guide{position:absolute;inset:15% 15%;border:2px dashed #6fffc688;pointer-events:none}footer{padding:12px 24px;color:#b9ccd9}#status{color:#6fffc6}button{background:#263744;border:1px solid #567;color:white;padding:8px 14px;border-radius:8px}</style>
<header><h1>Live depth camera</h1><div>Frame the <b>entire white paddle, claw, and tabletop together</b>. Keep the handle inside the dashed area.</div><p id="status">Connecting…</p></header>
<div class="view"><img id="feed" src="/stream.mjpg"><div class="guide"></div></div>
<footer>Camera view only • No robot movement controls • <button onclick="document.querySelector('#feed').src='/stream.mjpg?t='+Date.now()">Reconnect view</button></footer>
<script>async function status(){try{let s=await(await fetch('/status',{cache:'no-store'})).json();let e=document.querySelector('#status');e.textContent=s.ok?'LIVE • frame '+s.seq+' • '+s.age_s.toFixed(2)+' seconds old':'Waiting for camera: '+s.error;e.style.color=s.ok?'#6fffc6':'#ffc36f'}catch(e){document.querySelector('#status').textContent='Viewer connection lost'}}setInterval(status,1000);status();</script></html>'''

def frame():
 m=json.loads((FRAMES/'oak.json').read_text());age=time.time()-m['captured_at']
 p=(FRAMES/m['image']).resolve()
 if p.parent!=FRAMES.resolve():raise ValueError('Invalid frame path')
 if not 0<=age<3:raise ValueError('Capture paused; latest frame is %.1f seconds old'%age)
 data=p.read_bytes()
 if hashlib.sha256(data).hexdigest()!=m['sha256']:raise ValueError('Frame hash mismatch')
 return m,data,age
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def do_GET(self):
  if self.path.split('?')[0]=='/stream.mjpg':
   self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.send_header('Cache-Control','no-store');self.end_headers()
   previous=None
   try:
    while True:
     try:
      m,data,age=frame();identity=(m['stream_id'],m['seq'])
      if identity!=previous:
       self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n');self.wfile.flush();previous=identity
     except (OSError,ValueError,KeyError):pass
     time.sleep(.07)
   except (BrokenPipeError,ConnectionResetError):return
  elif self.path=='/status':
   try:m,data,age=frame();value={'ok':True,'seq':m['seq'],'age_s':age}
   except Exception as e:value={'ok':False,'error':str(e)}
   data=json.dumps(value).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
  elif self.path=='/':
   self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(HTML.encode())
  else:self.send_error(404)
server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
url='http://127.0.0.1:'+str(server.server_port)+'/'
(ROOT/'work/depth-viewer/access.json').write_text(json.dumps({'url':url}))
print(url,flush=True);server.serve_forever()
