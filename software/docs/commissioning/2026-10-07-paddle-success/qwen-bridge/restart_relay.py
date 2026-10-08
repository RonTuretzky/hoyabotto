"""Start/reuse only the robot's mTLS TCP relay. Never opens servo ports."""
import subprocess,json,time,re
from pathlib import Path
root=Path('/Users/teachera/Documents/Codex/2026-10-05/m');record=root/'work/gemma-hardware-relay-process.json';log=root/'work/qwen-server-recovery/relay.log'
if record.exists():
 d=json.loads(record.read_text());cmd=subprocess.run(['ps','-p',str(d['pid']),'-o','command='],capture_output=True,text=True).stdout
 if 'cloudflared tunnel' in cmd and 'tcp://127.0.0.1:1241' in cmd:
  urls=re.findall(r'https://([a-z0-9-]+\.trycloudflare\.com)',Path(d['log']).read_text())
  print(json.dumps({'pid':d['pid'],'relay_hostname':urls[-1] if urls else None,'reused':True}));raise SystemExit(0)
with log.open('w') as f:p=subprocess.Popen([str(root/'work/bin/cloudflared'),'tunnel','--url','tcp://127.0.0.1:1241','--no-autoupdate'],cwd=root,stdout=f,stderr=f,start_new_session=True)
record.write_text(json.dumps({'pid':p.pid,'started':time.time(),'log':str(log)},indent=2))
for _ in range(100):
 if p.poll() is not None:raise RuntimeError('Relay exited; inspect relay.log')
 urls=re.findall(r'https://([a-z0-9-]+\.trycloudflare\.com)',log.read_text())
 if urls:print(json.dumps({'pid':p.pid,'relay_hostname':urls[-1],'reused':False}));break
 time.sleep(.1)
else:raise RuntimeError('Relay hostname not yet available')
