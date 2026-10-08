"""Pickup profile: a long gripper closure runs as consecutive <=300-tick closures, stopping at the first short one."""
import ast,time,types
from pathlib import Path
tree=ast.parse(Path(__file__).with_name('gemma_robot_tools.py').read_text())
keep=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='set_gripper' or isinstance(n,ast.Assign) and any(getattr(t,'id','')=='GRIPPER_CLOSE_CHUNK' for t in n.targets)]
calls=[]
class Client:
    cancel_generation=0;short_at=None
    def status(self):return {'execution_profile':'paddle-success-v1','rows':{'right_arm_gripper':{'Present_Position':2500}}}
    def set_gripper(self,arm,target,d):
        calls.append(target);done=self.short_at is None or target>self.short_at
        return {'completed':done,'closure_outcome':'endpoint_settled' if done else 'settled_short','readbacks':{'right_arm_gripper':target}}
space={'DIRECT_CLIENT':Client()};exec(compile(ast.Module(keep,[]),'gemma_robot_tools.py','exec'),space)
r=space['set_gripper']('right',1800,3)   # 700 ticks -> 3 pieces of <=300
assert calls==[2267,2033,1800] and r['completed'] and len(r['closure_parts'])==3,calls
calls.clear();space['DIRECT_CLIENT'].short_at=2100;r=space['set_gripper']('right',1800,3)
assert calls==[2267,2033] and not r['completed'],calls   # stops at the first closure that met something
calls.clear();space['DIRECT_CLIENT'].short_at=None;space['set_gripper']('right',2300,3);assert calls==[2300]  # short closure: one command
calls.clear();space['set_gripper']('right',2700,3);assert calls==[2700]  # opening: one command
print('Gripper chunks: long closure split into <=300-tick pieces, stops at the first short piece')
