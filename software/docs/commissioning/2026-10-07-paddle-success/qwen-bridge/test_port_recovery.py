"""A servo port left busy by an interrupted transaction is recovered by the owner, not left failing forever."""
from types import SimpleNamespace as C
from gemma_hardware_owner import recover_ports
class Ser:
 def __init__(self,reset_ok=True):self.reset_ok=reset_ok;self.calls=[]
 def reset_input_buffer(self):
  self.calls.append('reset')
  if not self.reset_ok:raise OSError('device not configured')
 def close(self):self.calls.append('close')
 def open(self):self.calls.append('open')
def bus(port,busy,reset_ok=True):return C(port=port,port_handler=C(is_using=busy,ser=Ser(reset_ok)))
left,right=bus('/dev/left',True),bus('/dev/right',False);state={}
assert recover_ports([left,right],'Coherent servo read communication failure: -1; transaction={}',state,5.0)
assert left.port_handler.is_using is False and left.port_handler.ser.calls==['reset'] and right.port_handler.ser.calls==[]
assert state['port_recoveries']==[{'time':5.0,'port':'/dev/left','outcome':'cleared stale busy flag'}] and state['port_recovery_count']==1
# Buffer reset fails: the port is reopened.
dead=bus('/dev/left',True,reset_ok=False);state={}
assert recover_ports([dead],'Coherent servo read communication failure: -1',state,6.0)
assert dead.port_handler.ser.calls==['reset','close','open'] and state['port_recoveries'][0]['outcome'].startswith('reopened port')
# Other failures (timeouts, faults) and ports not marked busy are left alone.
other=bus('/dev/left',True);state={}
assert not recover_ports([other],'Coherent servo read communication failure: -6',state,7.0) and other.port_handler.is_using and state=={}
assert not recover_ports([bus('/dev/left',False)],'Coherent servo read communication failure: -1',{},8.0)
print('Port recovery: stale busy flag cleared, reopen fallback, other failures untouched; no hardware')
