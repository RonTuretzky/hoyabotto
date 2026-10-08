"""A crashed OAK that the watchdog is restarting is waited for (up to 15 s); one that has been down long fails at once."""
import ast,time,types
from pathlib import Path
# gemma_robot_tools only imports on the robot Mac: run the real select_oak_manifest from its source.
tree=ast.parse(Path(__file__).with_name('gemma_robot_tools.py').read_text())
keep=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='select_oak_manifest'
      or isinstance(n,ast.Assign) and any(getattr(t,'id','') in ('OAK_RESTART_WAIT_S','OAK_RECENT_S') for t in n.targets)]
space={'time':time};exec(compile(ast.Module(keep,[]),'gemma_robot_tools.py','exec'),space)
G=types.SimpleNamespace()
def select(**kw):
    space['_oak_last_frame_time']=G._oak_last_frame_time;space['_select_oak_manifest']=G._select_oak_manifest
    return space['select_oak_manifest'](**kw)
G.select_oak_manifest=select
now=[1000.0];slept=[]
G._oak_last_frame_time=lambda:last[0]
def sleep(d):slept.append(d);now[0]+=d
calls=[0]
def flaky():
    calls[0]+=1
    if now[0]<1008:raise RuntimeError('No fresh OAK stream: raw_fallback stale age_s=3')
    return ('folder',{'captured_at':now[0]},'raw_fallback')
G._select_oak_manifest=flaky
last=[997.0]  # fresh 3 s ago: restarting
assert G.select_oak_manifest(clock=lambda:now[0],sleep=sleep)[2]=='raw_fallback' and 7<=sum(slept)<=9
slept.clear();now[0]=2000.0;last=[1900.0]  # down for 100 s: no wait
G._select_oak_manifest=lambda:(_ for _ in ()).throw(RuntimeError('No fresh OAK stream'))
try:G.select_oak_manifest(clock=lambda:now[0],sleep=sleep)
except RuntimeError:assert slept==[]
else:raise AssertionError('stale OAK accepted')
slept.clear();last=[now[0]-1]  # restarting but never recovers: gives up after the wait
try:G.select_oak_manifest(clock=lambda:now[0],sleep=sleep)
except RuntimeError:assert 14.5<=sum(slept)<=15.5
else:raise AssertionError('never-fresh OAK accepted')
print('OAK wait: restart waited out, long outage fails fast, bounded at 15 s')
