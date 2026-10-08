import json,os,subprocess,sys,tempfile,time
from pathlib import Path
import calibration_job as J
import remote_admin as A
now=1000.0
fresh={'received_at':now-1,'seq':5}
assert J.precheck({'enabled_motors':[],'rows':{'a':{'Torque_Enable':0}}},fresh,now)==[]
assert any('powered' in b for b in J.precheck({'enabled_motors':['x'],'rows':{}},fresh,now))
assert any('camera' in b for b in J.precheck({'enabled_motors':[],'rows':{}},{'received_at':now-30,'seq':1},now))

class Bus:
    def __init__(self,torque=0):self.r={j:{'Torque_Enable':torque,'Operating_Mode':1,'Homing_Offset':1,'Min_Position_Limit':2,'Max_Position_Limit':3} for j in J.JOINTS};self.torque=torque
    def disable_torque(self,**kw):
        if not self.torque:
            for j in self.r:self.r[j]['Torque_Enable']=0
    def read(self,reg,j,**kw):return self.r[j][reg]
    def write(self,reg,j,v,**kw):self.r[j][reg]=v
before={f'left_arm_{j}':{'homing_offset':100+i,'range_min':900,'range_max':3000} for i,j in enumerate(J.JOINTS)}
b=Bus();rep=J.restore_registers(b,'left',before)
assert rep['restored'] and b.r['elbow_flex']=={'Torque_Enable':0,'Operating_Mode':0,'Homing_Offset':102,'Min_Position_Limit':900,'Max_Position_Limit':3000}
b=Bus(torque=1);rep=J.restore_registers(b,'left',before);assert not rep['restored'] and 'torque still on' in rep['error'] and b.r['elbow_flex']['Homing_Offset']==1
assert 'no entry' in J.restore_registers(Bus(),'right',before)['error']

def scenario(result,camera=fresh,status=None):
    tmp=Path(tempfile.mkdtemp());work=tmp/'work';(work/'gemma-hardware-session').mkdir(parents=True);(work/'phone_camera').mkdir()
    (work/'gemma-hardware-session/status.json').write_text(json.dumps(status or {'enabled_motors':[],'rows':{'m':{'Torque_Enable':0}},'phase':'idle'}))
    (work/'phone_camera/latest.json').write_text(json.dumps(camera))
    job=tmp/'job.json';job.write_text(json.dumps({'id':'j','arm':'left','velocity':300,'checkout':str(tmp/'checkout'),'work':str(work),'state':'running'}))
    calls=[]
    def runner(software,job_,evidence,save):
        calls.append('runner');evidence.mkdir(parents=True)
        if result is not None:(evidence/'result.json').write_text(json.dumps(result));(evidence/'calibration-before.json').write_text(json.dumps(before))
        return 0
    out=J.run(job,stop_owner=lambda w:calls.append('stop') or True,run_runner=runner,
              restore=lambda s,a,bf:calls.append('restore') or {'restored':bf==before},restart=lambda c:calls.append('restart') or 0,now=lambda:now)
    return out,calls
out,calls=scenario({'upstream_return':0,'routine_completed':True,'release_verified':True,'full_range_validated':True,'installed':True})
assert out['state']=='succeeded' and out['outcome']=='installed' and calls==['stop','runner','restart']
out,calls=scenario({'upstream_return':0,'routine_completed':True,'release_verified':True,'full_range_validated':False,'installed':False,'problems':['pan short']})
assert out['outcome']=='not_installed_restored' and calls==['stop','runner','restore','restart'] and out['summary']['problems']==['pan short']
out,calls=scenario({'upstream_return':1,'routine_completed':False,'release_verified':False,'installed':False})
assert out['outcome']=='release_unverified' and calls==['stop','runner'] # no restart, no restore: 12 V off first
out,calls=scenario({'error':'Preflight requires both arms released','release_verified':True,'installed':False})
assert out['outcome']=='not_started' and calls==['stop','runner','restart']
out,calls=scenario(None,camera={'received_at':now-60,'seq':1})
assert out['state']=='refused' and calls==[]
out,calls=scenario(None,status={'enabled_motors':['right_arm_elbow_flex'],'rows':{},'phase':'holding'})
assert out['state']=='refused' and calls==[]

# Job control: validation, one job at a time, STOP interrupts a running sweep.
with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);(root/'work').mkdir()
    try:A.start_calibration(root,'left',300)
    except ValueError as x:assert 'deploy record' in str(x)
    (root/'work/deploy.json').write_text(json.dumps({'checkout':str(root/'checkout')}))
    for arm,v in [('head',300),('left',1000)]:
        try:A.start_calibration(root,arm,v)
        except ValueError:pass
        else:raise AssertionError('bad calibration request accepted')
    sweep=subprocess.Popen([sys.executable,'-c','import time\ntry:time.sleep(30)\nexcept KeyboardInterrupt:pass'],start_new_session=True)
    holder=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    (root/'work/deploy-jobs').mkdir();(root/'work/deploy-jobs/20261007-000000-aaaaaa.json').write_text(json.dumps({'id':'20261007-000000-aaaaaa','kind':'calibration','state':'running','pid':holder.pid,'runner_pid':sweep.pid}))
    try:A.start_calibration(root,'right',300)
    except ValueError as x:assert 'Another job is running' in str(x)
    else:raise AssertionError('second job accepted')
    r=A.interrupt_calibration(root);assert r=={'calibration_interrupted':True,'job':'20261007-000000-aaaaaa'}
    sweep.wait(5);holder.kill()
print('Calibration job: precheck refusals, verified register restore, installed/restored/release-unverified/not-started outcomes, one job at a time, STOP interrupt passed; no hardware')
