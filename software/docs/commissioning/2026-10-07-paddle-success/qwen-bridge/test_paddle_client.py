import tempfile,json,threading,time
from pathlib import Path
from gemma_direct_client import DirectJointClient,atomic_json
n='right_arm_shoulder_lift'
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);state={'hardware_server':True,'execution_profile':'paddle-success-v1','control_mode':'direct_joint','supportsselectedjoints':[n],'supported_motors':[n],'commandable_motors':[n],'ranges':{n:[826,3268]},'operator_armed':True,'started':7,'phase':'holding','ok':True,'enabled_motors':[n],'lease_remaining':1,'rows':{n:{'Torque_Enable':1,'Status':0,'Present_Position':2697,'Present_Temperature':30,'Present_Load':0}}}
 def save():state['time']=time.time();atomic_json(folder/'status.json',state)
 save();c=DirectJointClient(folder,{n:{'range_min':826,'range_max':3268}})
 state['lease_remaining']=0;save()
 try:c.execute({n:2450},2)
 except RuntimeError as exc:assert 'Insufficient owner lease' in str(exc)
 else:raise AssertionError('Expired lease accepted')
 assert not (folder/'command.json').exists()
 state['lease_remaining']=1;save()
 stop=threading.Event()
 def owner():
  while not stop.wait(.01):
   if (folder/'command.json').exists():
    command=json.loads((folder/'command.json').read_text());state['rows'][n]['Present_Position']=2475;state['completed']=command['id'];save();return
 thread=threading.Thread(target=owner,daemon=True);thread.start()
 try:
  result=c.execute({n:2450},2)
  assert result['completed'] and result['readbacks'][n]==2475 and result['execution_profile']=='paddle-success-v1'
 finally:stop.set();thread.join(1)
print('Pickup client accepts measured 25-tick endpoint error and reserves movement time despite one-second remaining hold lease under named profile; no hardware access')
