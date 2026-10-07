import subprocess,json,time,os,sys
from pathlib import Path
root=Path.cwd();record=root/'work/gemma-hardware-owner-process.json';old=json.loads(record.read_text());cmd=subprocess.run(['ps','-p',str(old['pid']),'-o','command='],capture_output=True,text=True)
if 'gemma_hardware_owner.py' in cmd.stdout:raise RuntimeError('Existing hardware owner still active; will not create another')
status=root/'work/gemma-hardware-session/status.json'
if status.exists():(root/'outputs/Gemma-Previous-Owner-Fault-Status.json').write_bytes(status.read_bytes())
with (root/'work/gemma-hardware-owner.log').open('ab') as log:
 p=subprocess.Popen(['/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python',str(root/'work/gemma_hardware_owner.py')]+(['--read-only'] if '--read-only' in sys.argv else [])+(['--right-arm-only'] if '--right-arm-only' in sys.argv else []),cwd=root,stdout=log,stderr=log,start_new_session=True)
record.write_text(json.dumps({'pid':p.pid,'started':time.time()},indent=2))
for _ in range(60):
 if p.poll() is not None:raise RuntimeError('Owner startup failed; inspect log')
 try:
  s=json.loads(status.read_text())
  if s['started']>old['started'] and time.time()-s['time']<1 and s['phase']=='idle':break
 except (OSError,ValueError):pass
 time.sleep(.1)
else:raise RuntimeError('No fresh startup status')
assert len(s['rows'])==16 and all(row['Torque_Enable']==0 for row in s['rows'].values()) and s['motor_writes']==0
print(json.dumps({'pid':p.pid,'owner_session_started':s['started'],'phase':s['phase'],'all16_released':True,'motor_writes':s['motor_writes']},indent=2))
