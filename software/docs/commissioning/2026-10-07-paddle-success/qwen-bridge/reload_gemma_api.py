import json,os,signal,subprocess,time
from pathlib import Path
root=Path.cwd();record=root/'work/gemma-robot-tools-process.json';old=json.loads(record.read_text());pid=old['pid']
cmd=subprocess.check_output(['ps','-p',str(pid),'-o','command='],text=True)
if 'gemma_robot_tools.py' not in cmd:raise RuntimeError('API process identity mismatch')
os.kill(pid,signal.SIGTERM)
for _ in range(40):
 if subprocess.run(['kill','-0',str(pid)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:break
 time.sleep(.1)
log_path=root/'work/qwen-server-recovery/api.log'
env=dict(os.environ,XLEROBOT_PASSIVE_RECOVERY='1',XLEROBOT_OAK_RAW_DIR='/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/work/oak-live-stream')
with log_path.open('ab') as log:
 p=subprocess.Popen(['/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python',str(root/'work/gemma_robot_tools.py')],cwd=root,stdout=log,stderr=log,start_new_session=True,env=env)
record.write_text(json.dumps({'pid':p.pid,'started_at':time.time(),'log':str(log_path),'detached':True,'armed':False},indent=2))
time.sleep(3)
if p.poll() is not None:raise RuntimeError('API restart failed')
s=json.loads((root/'work/gemma-hardware-session/status.json').read_text());print(json.dumps({'api_pid':p.pid,'owner_status_age_s':time.time()-s['time'],'phase':s['phase'],'enabled_motors':s['enabled_motors'],'motor_writes':s['motor_writes'],'right_elbow':s['rows'].get('right_arm_elbow_flex')},indent=2))
