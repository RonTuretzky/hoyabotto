"""Operator administration over the pinned-mTLS robot API: read logs, deploy a git ref, restart.

Served at /admin/* by gemma_robot_tools.py. These are not LLM tools: the chat only calls /tools and /call.
Code only comes from the repository checkout's own origin (git fetch + checkout of a ref); there is no
arbitrary shell or file upload. A restart runs ./restart-robot-server.sh detached from the API (which it
replaces), refuses while motors are holding, and rolls back to the previous files if the new ones fail.
Usage inside the robot work folder: python remote_admin.py run-job <job.json>
"""
import json,os,re,signal,subprocess,sys,time
from pathlib import Path

REF=re.compile(r'^[A-Za-z0-9][A-Za-z0-9._/-]{0,99}$')
MODES={'joycon-ready':['--joycon-teleop','--native-joycon-reference','--no-wrist-cams'],'joycon-ready-dry-run':['--joycon-teleop','--native-joycon-reference','--no-wrist-cams','--dry-run'],'restart':[],'api-only':['--api-only'],'network-only':['--network-only'],'oak-off':['--cameras-only','--oak','off'],'oak-on':['--cameras-only','--oak','on'],'cameras-only':['--cameras-only'],'dry-run':['--dry-run'],'checkout-only':None}
MODES.update({'phone-only': ['--phone-only'], 'phone-dry-run': ['--phone-only', '--dry-run']})
MAX_LOG_BYTES=64*1024


def paths(root):
    work=Path(root)/'work'
    return {'work':work,'record':work/'deploy.json','jobs':work/'deploy-jobs',
            'logs':{'owner':work/'gemma-hardware-owner.log','api':work/'qwen-server-recovery/api.log',
                    'relay':work/'qwen-server-recovery/relay.log','redeploy':work/'redeploy.log',
                    'wrist_right':work/'wrist-camera-stream/right_wrist.log','wrist_left':work/'wrist-camera-stream/left_wrist.log',
                    'oak':work/'oak-stream.log'}}


def tail(path,lines=80):
    try:
        with open(path,'rb') as f:
            f.seek(0,2);size=f.tell();f.seek(max(0,size-MAX_LOG_BYTES));data=f.read().decode(errors='replace')
        return {'path':str(path),'size':size,'modified':os.path.getmtime(path),'lines':data.splitlines()[-max(1,min(int(lines),2000)):]}
    except OSError as e:return {'path':str(path),'missing':True,'error':str(e)}


def logs(root,names=None,lines=80):
    p=paths(root);wanted=names or list(p['logs'])+['camera_setup','last_job']
    out={}
    for name in wanted:
        if name in p['logs']:out[name]=tail(p['logs'][name],lines)
        elif name=='camera_setup':
            try:out[name]=json.loads((p['work']/'wrist-camera-setup.json').read_text())
            except (OSError,ValueError) as e:out[name]={'missing':True,'error':str(e)}
        elif name=='last_job':
            jobs=sorted(p['jobs'].glob('*.json')) if p['jobs'].exists() else []
            out[name]=job_status(root,jobs[-1].stem,lines) if jobs else {'missing':True}
        else:out[name]={'error':'unknown log name; known: '+', '.join(list(p['logs'])+['camera_setup','last_job'])}
    return out


def git(checkout,*args,timeout=60):
    r=subprocess.run(['git','-C',str(checkout),*args],capture_output=True,text=True,timeout=timeout)
    return r.returncode,(r.stdout+r.stderr).strip()


def deploy_status(root):
    p=paths(root)
    try:record=json.loads(p['record'].read_text())
    except (OSError,ValueError):return {'available':False,'note':'No deploy record yet: run ./restart-robot-server.sh once on the robot Mac with this version.'}
    checkout=record.get('checkout');status={'available':True,'record':record}
    if checkout and Path(checkout).is_dir():
        status['head']=git(checkout,'rev-parse','HEAD')[1];status['branch']=git(checkout,'rev-parse','--abbrev-ref','HEAD')[1]
        status['dirty']=bool(git(checkout,'status','--porcelain')[1]);status['subject']=git(checkout,'log','-1','--format=%s')[1]
    try:status['network']=json.loads((p['work']/'network.json').read_text())  # LAN address and relay hostname
    except (OSError,ValueError):status['network']=None
    jobs=sorted(p['jobs'].glob('*.json')) if p['jobs'].exists() else []
    status['recent_jobs']=[job_status(root,j.stem,0) for j in jobs[-5:]]
    return status


def job_status(root,job_id,lines=120):
    p=paths(root)
    if not re.fullmatch(r'[0-9]{8}-[0-9]{6}-[0-9a-f]{6}',job_id or ''):raise ValueError('Unknown job id')
    try:job=json.loads((p['jobs']/(job_id+'.json')).read_text())
    except (OSError,ValueError):raise ValueError('Unknown job id')
    if lines:job['log']=tail(p['jobs']/(job_id+'.log'),lines)
    return job


def start_deploy(root,ref,mode,python=sys.executable):
    p=paths(root)
    if not isinstance(ref,str) or not REF.match(ref) or '..' in ref:raise ValueError('ref must be a branch, tag or commit name')
    if mode not in MODES:raise ValueError('mode must be one of '+', '.join(MODES))
    try:record=json.loads(p['record'].read_text())
    except (OSError,ValueError):raise ValueError('No deploy record yet: run ./restart-robot-server.sh once on the robot Mac with this version.')
    checkout=record.get('checkout')
    if not checkout or not (Path(checkout)/'restart-robot-server.sh').exists():raise ValueError('Recorded checkout is missing restart-robot-server.sh')
    p['jobs'].mkdir(parents=True,exist_ok=True)
    busy=running_job(root)
    if busy:raise ValueError(f"Another job is running: {busy['id']} ({busy.get('kind','deploy')})")
    job_id=time.strftime('%Y%m%d-%H%M%S-')+os.urandom(3).hex()
    job={'id':job_id,'ref':ref,'mode':mode,'checkout':checkout,'state':'running','created':time.time()}
    path=p['jobs']/(job_id+'.json');path.write_text(json.dumps(job,indent=2))
    with open(p['jobs']/(job_id+'.log'),'ab') as log:
        proc=subprocess.Popen([python,str(Path(__file__).resolve()),'run-job',str(path)],cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    job['pid']=proc.pid;path.write_text(json.dumps(job,indent=2))
    return job


def running_job(root):
    p=paths(root)
    for j in sorted(p['jobs'].glob('*.json')) if p['jobs'].exists() else []:
        job=json.loads(j.read_text())
        if job.get('state')=='running' and _alive(job.get('pid')):return job
    return None


def start_calibration(root,arm,velocity=300,python=sys.executable):
    """Detached automatic calibration of one arm (calibration_job.py). The owner stops for its duration."""
    p=paths(root)
    if arm not in ('left','right'):raise ValueError('arm must be left or right')
    if velocity not in (200,300):raise ValueError('velocity must be 200 or 300 (the speeds that have completed on this robot)')
    try:record=json.loads(p['record'].read_text())
    except (OSError,ValueError):raise ValueError('No deploy record yet: run ./restart-robot-server.sh once on the robot Mac with this version.')
    busy=running_job(root)
    if busy:raise ValueError(f"Another job is running: {busy['id']} ({busy.get('kind','deploy')})")
    p['jobs'].mkdir(parents=True,exist_ok=True)
    job_id=time.strftime('%Y%m%d-%H%M%S-')+os.urandom(3).hex()
    job={'id':job_id,'kind':'calibration','arm':arm,'velocity':velocity,'checkout':record['checkout'],'work':str(p['work']),'state':'running','phase':'precheck','created':time.time()}
    path=p['jobs']/(job_id+'.json');path.write_text(json.dumps(job,indent=2))
    with open(p['jobs']/(job_id+'.log'),'ab') as log:
        proc=subprocess.Popen([python,str(Path(__file__).resolve().with_name('calibration_job.py')),str(path)],cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    job['pid']=proc.pid;path.write_text(json.dumps(job,indent=2))
    return job


def start_restore(root,arm,python=sys.executable):
    """Detached, no-motion job: write the saved calibration file's values for one arm into its servos, then restart."""
    p=paths(root)
    if arm not in ('left','right'):raise ValueError('arm must be left or right')
    try:record=json.loads(p['record'].read_text())
    except (OSError,ValueError):raise ValueError('No deploy record yet: run ./restart-robot-server.sh once on the robot Mac with this version.')
    busy=running_job(root)
    if busy:raise ValueError(f"Another job is running: {busy['id']} ({busy.get('kind','deploy')})")
    p['jobs'].mkdir(parents=True,exist_ok=True)
    job_id=time.strftime('%Y%m%d-%H%M%S-')+os.urandom(3).hex()
    job={'id':job_id,'kind':'calibration','action':'restore','arm':arm,'checkout':record['checkout'],'work':str(p['work']),'state':'running','phase':'precheck','created':time.time()}
    path=p['jobs']/(job_id+'.json');path.write_text(json.dumps(job,indent=2))
    with open(p['jobs']/(job_id+'.log'),'ab') as log:
        proc=subprocess.Popen([python,str(Path(__file__).resolve().with_name('calibration_job.py')),str(path)],cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    job['pid']=proc.pid;path.write_text(json.dumps(job,indent=2))
    return job


def processes():
    """Read-only: the robot-service processes (owner, API, cameras, relay, jobs) with their start time and command."""
    out=subprocess.run(['ps','-axo','pid=,ppid=,lstart=,args='],capture_output=True,text=True).stdout
    keys=('gemma_hardware_owner','gemma_robot_tools','capture','oak_camera','cloudflared','phone_camera','depth-viewer','remote_admin','calibration_job','redeploy_robot_server','upstream_pr3282')
    rows=[]
    for line in out.splitlines():
        if any(k in line for k in keys) and 'ps -axo' not in line:
            parts=line.split(None,7)
            rows.append({'pid':int(parts[0]),'ppid':int(parts[1]),'started':' '.join(parts[2:7]),'command':parts[7][:300] if len(parts)>7 else ''})
    return rows


CAMERA_ID=re.compile(r'^0x[0-9a-f]{14}$')

def set_wrist_ids(root,body):
    """Pin which camera is which (IDs are USB port paths, so they change when cables move). Writes
    work/wrist-cameras.json; the caller then restarts the camera streams. No motors involved."""
    if not isinstance(body,dict) or not body or set(body)-{'left_wrist','right_wrist','head_camera','verified'}:
        raise ValueError('Give left_wrist, right_wrist and/or head_camera IDs (and optionally verified)')
    ids={k:v for k,v in body.items() if k!='verified'}
    if not ids or any(not isinstance(v,str) or not CAMERA_ID.match(v) for v in ids.values()):raise ValueError('Camera IDs look like 0x12400005a39230')
    if len(set(ids.values()))!=len(ids):raise ValueError('Each camera ID can be used once')
    path=paths(root)['work']/'wrist-cameras.json'
    try:config=json.loads(path.read_text())
    except (OSError,ValueError):config={}
    for name in ('left_wrist','right_wrist'):
        if name in ids:config[name]={'camera_id':ids[name],'identity_verified':body.get('verified') is True,'set_by':'admin','set_at':time.time()}
    if 'head_camera' in ids:config['head_camera_id']=ids['head_camera']
    wrists={config.get(n,{}).get('camera_id') for n in ('left_wrist','right_wrist')}
    if config.get('head_camera_id') in wrists:raise ValueError('The head camera cannot also be a wrist camera')
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(config,indent=2));tmp.replace(path)
    return {k:config.get(k) for k in ('left_wrist','right_wrist','head_camera_id')}


def start_camera_restart(root,python=sys.executable):
    """Detached job: run the restart script's camera step only (wrist publishers + OAK stream). No motors, no git."""
    p=paths(root)
    try:record=json.loads(p['record'].read_text())
    except (OSError,ValueError):raise ValueError('No deploy record yet: run ./restart-robot-server.sh once on the robot Mac with this version.')
    busy=running_job(root)
    if busy:raise ValueError(f"Another job is running: {busy['id']} ({busy.get('kind','deploy')})")
    p['jobs'].mkdir(parents=True,exist_ok=True)
    job_id=time.strftime('%Y%m%d-%H%M%S-')+os.urandom(3).hex()
    job={'id':job_id,'kind':'cameras','ref':'(deployed)','mode':'cameras-only','checkout':record['checkout'],'state':'running','created':time.time(),'skip_git':True}
    path=p['jobs']/(job_id+'.json');path.write_text(json.dumps(job,indent=2))
    with open(p['jobs']/(job_id+'.log'),'ab') as log:
        proc=subprocess.Popen([python,str(Path(__file__).resolve()),'run-job',str(path)],cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    job['pid']=proc.pid;path.write_text(json.dumps(job,indent=2))
    return job


def interrupt_calibration(root):
    """STOP during a calibration sweep: SIGINT the runner (upstream routine makes the motors limp, then cleans up)."""
    job=running_job(root)
    if not job or job.get('kind')!='calibration':return None
    pid=job.get('runner_pid')
    if _alive(pid):
        os.killpg(pid,signal.SIGINT);return {'calibration_interrupted':True,'job':job['id']}
    return {'calibration_interrupted':False,'job':job['id'],'phase':job.get('phase')}


def _alive(pid):
    if not isinstance(pid,int):return False
    state=subprocess.run(['ps','-o','stat=','-p',str(pid)],capture_output=True,text=True).stdout.strip()
    return bool(state) and not state.startswith('Z')


def run_job(path):
    """Detached job: fetch, check out the ref, then run the restart script (which replaces the API that started us)."""
    path=Path(path);job=json.loads(path.read_text());checkout=job['checkout']
    if job.get('skip_git'):
        print('== restart-robot-server.sh --cameras-only (no git changes)',flush=True)
        code=subprocess.run(['bash',str(Path(checkout)/'restart-robot-server.sh'),'--cameras-only'],cwd=checkout,stdin=subprocess.DEVNULL).returncode
        job.update(state='succeeded' if code==0 else 'failed',exit_code=code,finished=time.time());path.write_text(json.dumps(job,indent=2))
        sys.exit(0 if code==0 else 1)
    def step(label,code,out):
        print(f'== {label} (exit {code})\n{out}',flush=True)
        return code
    def finish(state,code):
        job.update(state=state,exit_code=code,finished=time.time(),head=git(checkout,'rev-parse','HEAD')[1]);path.write_text(json.dumps(job,indent=2))
        sys.exit(0 if state=='succeeded' else 1)
    if step('git fetch origin',*git(checkout,'fetch','--prune','origin',timeout=120)):finish('failed',1)
    code,remote=git(checkout,'rev-parse','--verify','--quiet','origin/'+job['ref']+'^{commit}')
    target=remote if code==0 else git(checkout,'rev-parse','--verify','--quiet',job['ref']+'^{commit}')[1]
    if not re.fullmatch(r'[0-9a-f]{40}',target or ''):step('resolve ref',1,'Unknown ref: '+job['ref']);finish('failed',1)
    if step(f'git checkout --detach {target}',*git(checkout,'checkout','--detach',target)):finish('failed',1)
    job['target']=target;path.write_text(json.dumps(job,indent=2))
    if MODES[job['mode']] is None:finish('succeeded',0)
    print('== restart-robot-server.sh '+' '.join(MODES[job['mode']]),flush=True)
    code=subprocess.run(['bash',str(Path(checkout)/'restart-robot-server.sh'),*MODES[job['mode']]],cwd=checkout,stdin=subprocess.DEVNULL).returncode
    finish('succeeded' if code==0 else 'failed',code)


if __name__=='__main__' and sys.argv[1:2]==['run-job']:run_job(sys.argv[2])
