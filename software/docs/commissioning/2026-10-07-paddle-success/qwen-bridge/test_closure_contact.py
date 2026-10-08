"""Pickup: a gripper closure that stops on an object far above its target is a holding outcome, not an error (12:55 today)."""
import tempfile,json,threading,time
from pathlib import Path
from gemma_direct_client import DirectJointClient,atomic_json
g='right_arm_gripper'
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp)
    state={'hardware_server':True,'execution_profile':'paddle-success-v1','control_mode':'direct_joint','supportsselectedjoints':[g],'supported_motors':[g],
           'commandable_motors':[g],'ranges':{g:[1270,2824]},'operator_armed':True,'started':7,'phase':'holding','ok':True,'enabled_motors':[g],'lease_remaining':100,
           'rows':{g:{'Torque_Enable':1,'Status':0,'Present_Position':1843,'Present_Temperature':30,'Present_Load':0,'Present_Voltage':121}}}
    def save():state['time']=time.time();atomic_json(folder/'status.json',state)
    save();c=DirectJointClient(folder,{g:{'range_min':1270,'range_max':2824}})
    def owner(position,outcome):
        stop=threading.Event()
        def run():
            while not stop.wait(.01):
                if (folder/'command.json').exists():
                    cmd=json.loads((folder/'command.json').read_text())
                    if cmd.get('op')=='gripper_target' and cmd['id']!=state.get('completed'):
                        state['rows'][g]['Present_Position']=position;state.update(completed=cmd['id'],closure_outcome=outcome,settle_residual_ticks={g:position-1550});save();return
        t=threading.Thread(target=run,daemon=True);t.start();return stop,t
    stop,t=owner(1620,'stationary_closure_unverified')
    try:
        r=c._command({'op':'gripper_target','positions':{g:1550},'duration_s':6})
        assert r['closure_outcome']=='stationary_closure_unverified' and r['holding'] and r['readbacks'][g]==1620 and not r['completed'],r
        assert json.loads((folder/'command.json').read_text())['op']=='gripper_target'  # no STOP written
    finally:stop.set();t.join(1)
    stop,t=owner(1400,'stationary_closure_unverified')  # cannot have closed past the target
    try:
        try:c._command({'op':'gripper_target','positions':{g:1550},'duration_s':6})
        except RuntimeError as e:assert 'past its target' in str(e)
        else:raise AssertionError('closure past target accepted')
    finally:stop.set();t.join(1)
print('Closure contact: jaws stopped on an object = holding outcome without STOP; past-target readback still an error')
