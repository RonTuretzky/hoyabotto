"""Nonblocking phone upload-receipt gate; receipt time is not capture time."""
import math,time
class PaddleCameraGate:
 def __init__(self,metadata,clock=time.monotonic,wall=time.time):
  self.metadata=metadata;self.clock=clock;self.wall=wall;self.paused_at=None;self.initial_seq=None;self.events=[]
 def update(self,holding=False):
  try:d=self.metadata()
  except (OSError,ValueError,TypeError):d={}
  stamp=d.get('received_at');seq=d.get('seq');age=self.wall()-stamp if type(stamp) in (int,float) and math.isfinite(stamp) else None
  fresh=age is not None and 0<=age<10 and seq is not None
  if self.paused_at is None:
   if fresh:return True
   if not holding:raise RuntimeError('Pickup phone feed stale before hold established')
   self.paused_at=self.clock();self.initial_seq=seq;self.events.append({'started_at':self.wall(),'initial_sequence':seq});self.events=self.events[-20:]
  if self.clock()-self.paused_at>=20:
   self.events[-1]['timed_out']=True;raise RuntimeError('Pickup phone feed stale after monitored 20-second hold')
  if age is not None and 0<=age<2 and seq is not None and seq!=self.initial_seq:
   self.events[-1]['resumed_after_s']=self.clock()-self.paused_at;self.paused_at=None;return True
  return False
