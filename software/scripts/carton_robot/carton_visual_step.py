"""Explicit camera-reviewed steps through the existing sole motor owner.

No automatic image-to-motion mapping. The caller reviews immutable images
between steps; bounded partial jaw motion reports contact, never a proven grasp.
"""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
from carton_runtime import ROOT, LIVE, SOFTWARE, SESSION, FRAMES, PROFILE, camera_identity, calibration_file
sys.path.insert(0,str(SOFTWARE))
from carton.servo.cli import prepare
from carton.servo.common import Limits,atomic_json
from carton.servo.transport import SessionTransport
from carton.servo.vision import ManifestCamera
RUN=int(os.environ.get('CARTON_STEP_RUN','1'))
FOLDER=ROOT/('work/carton-camera-reviewed-approach' if RUN==1 else f'work/carton-camera-reviewed-approach-{RUN}')
# SESSION is shared with the configured owner.

def frames(label):
    folder=FOLDER/label;folder.mkdir(exist_ok=False)
    for name,identity in [('head',camera_identity('head')),('right_wrist',camera_identity('right_wrist'))]:
        f=ManifestCamera(FRAMES/f'{name}.json',identity,1.0).read()
        (folder/f'{name}.jpg').write_bytes(f.raw_bytes)
        atomic_json(folder/f'{name}.json',f.manifest)
    return folder

def release():
    atomic_json(SESSION/'command.json',{'id':time.time_ns(),'op':'stop'})
    end=time.monotonic()+6
    while time.monotonic()<end:
        s=json.loads((SESSION/'status.json').read_text())
        if s.get('released'):
            print('RELEASED',s.get('release_errors',[]),flush=True);return
        time.sleep(.1)
    raise RuntimeError('Owner release not yet verified')

def start():
    FOLDER.mkdir(parents=True,exist_ok=True)
    from farm.config import load_profile
    cfg=load_profile(PROFILE).robot
    for port in (cfg.port1,cfg.port2):
        if not port:raise RuntimeError('Both motor ports must be explicitly configured')
        q=subprocess.run(['lsof','-t',port],capture_output=True,text=True)
        if q.returncode!=1 or q.stdout.strip():raise RuntimeError('Motor port already owned')
    frames('start-before')
    started=time.time()
    log=(FOLDER/'owner.log').open('wb')
    owner=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('carton_session.py')),'--arm','right','--recover-right-elbow'],
        cwd=LIVE,env=dict(os.environ,PYTHONPATH=str(SOFTWARE)),stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    atomic_json(FOLDER/'owner.json',{'pid':owner.pid,'started':started})
    try:
        end=time.monotonic()+10
        while time.monotonic()<end:
            if owner.poll() is not None:raise RuntimeError((FOLDER/'owner.log').read_text())
            s=json.loads((SESSION/'status.json').read_text())
            if s.get('started',0)>=started and s.get('phase')=='holding' and s.get('ok'):
                out=FOLDER/'experiment'
                prepare(argparse.Namespace(out=str(out),session=str(SESSION),frames=str(FRAMES),calibration=str(calibration_file()),arm='right'))
                c=json.loads((out/'experiment.json').read_text());atomic_json(FOLDER/'config.json',c)
                atomic_json(FOLDER/'budget.json',{'owner_started':s['started'],'origin':{n:r['Present_Position'] for n,r in s['rows'].items()},'travel':0,'steps':0})
                print('RECOVERED_AND_HOLDING',s['rows']['right_arm_elbow_flex'],flush=True)
                print(frames('start-held'),flush=True);return
            time.sleep(.1)
        raise RuntimeError('Owner did not become ready')
    except BaseException:
        if owner.poll() is None:owner.send_signal(signal.SIGTERM);owner.wait(timeout=8)
        raise
    finally:log.close()

def step(joint,delta):
    if joint not in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','gripper') or not 0<abs(delta)<=68:
        raise ValueError('One reviewed joint, at most six degrees')
    budget=json.loads((FOLDER/'budget.json').read_text());phase=budget.get('phase',0)
    label=(f"phase-{phase}-" if phase else '')+f"step-{budget['steps']+1:02d}"
    if budget['steps']>=5 or budget['travel']+abs(delta)>240:raise RuntimeError('Five-step reviewed approach allowance exhausted')
    c=json.loads((FOLDER/'config.json').read_text())
    if joint=='gripper':
        c['joints']=list(c['joints'])+['right_arm_gripper']
    try:
        frames(label+'-before')
        # Camera-reviewed endpoints use measured pose with one-degree tolerance;
        # precision calibration retains its unchanged five-tick requirement.
        with SessionTransport(c,Limits(),execute=True) as t:
            s,q=t.status()
            if s['started']!=budget['owner_started']:raise RuntimeError('Owner changed')
            n='right_arm_'+joint
            if abs(q[n]+delta-budget['origin'][n])>128:raise RuntimeError('Cumulative joint envelope exceeded')
            q,stamp=t.move(n,delta,reviewed_settle_ticks=24 if joint=='elbow_flex' else 11)
            budget['travel']+=abs(delta);budget['steps']+=1
            atomic_json(FOLDER/'budget.json',budget)
            time.sleep(.3)
            print(json.dumps({'positions':q,'evidence':str(frames(label+'-after'))}),flush=True)
    except BaseException:
        release();raise

def checkpoint():
    """Start a newly camera-reviewed local series without releasing the arm.

    No motion, lease renewal, limit changes, or automated repeated execution.
    Each invocation follows the caller's explicit review of both current views.
    """
    budget=json.loads((FOLDER/'budget.json').read_text())
    phase=budget.get('phase',0)+1
    if phase>3:raise RuntimeError('Reviewed phase allowance exhausted')
    c=json.loads((FOLDER/'config.json').read_text())
    with SessionTransport(c,Limits(),execute=False) as t:
        s,q=t.status()
        if s['started']!=budget['owner_started'] or s['phase']!='holding':raise RuntimeError('Unsettled or changed owner')
        if any(abs(q[n]-s['goals'][n])>24 for n in q):raise RuntimeError('Holding pose not settled')
        for _ in range(2):
            time.sleep(.15);s2,q2=t.status()
            if s2['phase']!='holding' or any(abs(q2[n]-q[n])>2 for n in q):raise RuntimeError('Checkpoint pose unstable')
        evidence=frames(f'phase-{phase}-review')
        atomic_json(FOLDER/f'budget-before-phase-{phase}.json',budget)
        atomic_json(FOLDER/'budget.json',{'owner_started':s['started'],'phase':phase,'origin':q2,'travel':0,'steps':0})
        print(json.dumps({'status':'REVIEWED_PHASE_READY','phase':phase,'positions':q2,'evidence':str(evidence)}),flush=True)

def grip(delta=None):
    """One partial closure; report only possible contact, never a proven grasp."""
    c=json.loads((FOLDER/'config.json').read_text());c['joints']+=['right_arm_gripper']
    label='grip' if delta is None else 'jaw-'+str(time.time_ns())
    frames(label+'-before')
    try:
        with SessionTransport(c,Limits(),execute=True) as t:
            s,before=t.status();n='right_arm_gripper'
            if delta is None:delta=-min(68,int(before[n])-1397)
            if type(delta) is not int or not 12<=abs(delta)<=68:raise RuntimeError('Jaw command must be twelve to sixty-eight ticks')
            goal=int(before[n])+delta
            if not c['ranges'][n][0]+4<=goal<=c['ranges'][n][1]-4:raise RuntimeError('Grip goal outside range')
            path=SESSION/'command.json'
            if path.exists() and json.loads(path.read_text()).get('id')!=t.last_id:raise RuntimeError('Foreign command writer')
            ident=time.time_ns();atomic_json(path,{'id':ident,'op':'move','delta_ticks':{n:delta}})
            t.last_id=ident;t.path_ticks=abs(delta)
            end=time.monotonic()+3.5;last_time=0;stable=[];count=0
            while time.monotonic()<end:
                s,q=t.status()
                if any(abs(q[j]-before[j])>5 for j in before if j!=n):raise RuntimeError('Other joint drift during grip')
                row=s['rows'][n]
                if abs(row['Present_Load'])>250:raise RuntimeError('Grip load exceeds cap')
                if not min(goal,before[n])-3<=q[n]<=max(goal,before[n])+3:raise RuntimeError('Jaw left its reviewed envelope')
                if s['time']>last_time:
                    last_time=s['time'];count+=1
                    frames(f'{label}-observe-{count:02d}')
                    stable.append(q[n]);stable=stable[-3:]
                    if s.get('completed')==ident and s['phase']=='holding' and len(stable)==3 and max(stable)-min(stable)<=2:
                        moved=(q[n]-before[n])*(1 if delta>0 else -1);error=abs(q[n]-goal)
                        if moved>=12 and (error<=11 or (11<error<=24 and abs(row['Present_Load'])>=50)):
                            result={'status':'PARTIAL_CLOSURE_ONLY' if delta<0 else 'PARTIAL_OPEN_ONLY','possible_contact':delta<0 and error>11,'position':q[n],'load':row['Present_Load'],'moved_ticks':moved,'goal':goal,'grasp_verified':False,'images':str(frames(label+'-after'))}
                            atomic_json(FOLDER/(label+'-result.json'),result);print(json.dumps(result),flush=True);return
                time.sleep(.08)
            raise RuntimeError('Partial grip did not settle or show bounded contact')
    except BaseException:
        release();raise

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('op',choices=['start','step','grip','jaw','checkpoint','stop']);p.add_argument('joint',nargs='?');p.add_argument('delta',type=int,nargs='?',default=0);a=p.parse_args()
    if a.op=='start':start()
    elif a.op=='step':step(a.joint,a.delta)
    elif a.op=='grip':grip()
    elif a.op=='jaw':grip(int(a.joint))
    elif a.op=='checkpoint':checkpoint()
    else:release()
