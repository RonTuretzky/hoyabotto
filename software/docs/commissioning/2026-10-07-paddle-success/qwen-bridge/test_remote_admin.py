import json,subprocess,tempfile,time
from pathlib import Path
import remote_admin as A
def sh(*a,cwd=None):subprocess.run(a,cwd=cwd,check=True,capture_output=True)
with tempfile.TemporaryDirectory() as tmp:
 tmp=Path(tmp);root=tmp/'robot';(root/'work').mkdir(parents=True)
 # A fake origin with two commits; the robot checkout is a clone of it.
 origin=tmp/'origin.git';sh('git','init','--bare','-b','main',str(origin))
 seed=tmp/'seed';sh('git','clone',str(origin),str(seed))
 (seed/'restart-robot-server.sh').write_text('#!/usr/bin/env bash\necho "restart args: $*"\nsleep "${SLEEP:-0}"\n')
 for cmd in (['git','add','-A'],['git','-c','user.email=t@t','-c','user.name=t','commit','-m','one'],['git','push','origin','HEAD:main']):sh(*cmd,cwd=seed)
 checkout=tmp/'checkout';sh('git','clone',str(origin),str(checkout))
 (seed/'new.txt').write_text('two')
 for cmd in (['git','add','-A'],['git','-c','user.email=t@t','-c','user.name=t','commit','-m','two'],['git','push','origin','HEAD:main']):sh(*cmd,cwd=seed)
 newest=subprocess.run(['git','-C',str(seed),'rev-parse','HEAD'],capture_output=True,text=True).stdout.strip()
 assert A.deploy_status(root)['available'] is False
 try:A.start_deploy(root,'main','restart')
 except ValueError as x:assert 'No deploy record' in str(x)
 else:raise AssertionError('deploy without record')
 (root/'work/deploy.json').write_text(json.dumps({'checkout':str(checkout)}))
 assert A.MODES['phone-only']==['--phone-only']
 assert A.MODES['phone-dry-run']==['--phone-only','--dry-run']
 for bad in ['../x','-rf','main;rm','']:
  try:A.start_deploy(root,bad,'restart')
  except ValueError:pass
  else:raise AssertionError('bad ref accepted: '+bad)
 try:A.start_deploy(root,'main','shell')
 except ValueError:pass
 else:raise AssertionError('bad mode accepted')
 job=A.start_deploy(root,'main','cameras-only')
 for _ in range(100):
  s=A.job_status(root,job['id'])
  if s['state']!='running':break
  time.sleep(.1)
 assert s['state']=='succeeded' and s['head']==newest and any('restart args: --cameras-only' in l for l in s['log']['lines']),s
 st=A.deploy_status(root);assert st['head']==newest and st['recent_jobs'][-1]['state']=='succeeded'
 try:A.job_status(root,'../../etc/passwd')
 except ValueError:pass
 else:raise AssertionError('job id traversal accepted')
 (root/'work/gemma-hardware-owner.log').write_text('\n'.join(f'line {i}' for i in range(500)))
 L=A.logs(root,['owner','camera_setup','nope'],lines=3)
 assert L['owner']['lines']==['line 497','line 498','line 499'] and L['camera_setup']['missing'] and 'unknown log name' in L['nope']['error']
print('Remote admin: record required, ref/mode/job-id validation, detached fetch+checkout+restart job, status and log tails passed; no hardware')
