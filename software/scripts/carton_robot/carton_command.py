"""Send one explicit bounded command to the supervised carton session."""
import argparse
import json
import time
from pathlib import Path

from carton_runtime import SESSION
folder=SESSION

def send(joint=None,delta=0,op='move'):
    d=json.loads((folder/'status.json').read_text())
    if d['phase'] not in ('holding','moving') or time.time()-d['time']>2:
        raise RuntimeError('No fresh active session')
    if op=='move' and d['phase']!='holding':raise RuntimeError('Previous movement has not settled')
    cmd={'id':time.time_ns(),'op':op}
    if op=='move':cmd['delta_ticks']={joint:delta}
    p=folder/'command.json';tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(cmd));tmp.replace(p)
    end=time.monotonic()+6
    while time.monotonic()<end:
        d=json.loads((folder/'status.json').read_text())
        if (op=='hold' and d.get('last_hold_accepted')==cmd['id']) or (op=='move' and d.get('completed')==cmd['id']) or (op=='stop' and d.get('released') and not d.get('release_errors')):
            print(json.dumps({'phase':d['phase'],'positions':{n:r['Present_Position'] for n,r in d['rows'].items()},'completed':d.get('completed')}));return
        if d['phase']=='stopped':raise RuntimeError(d.get('error'))
        if d.get('last_rejected',{}).get('id')==cmd['id']:raise RuntimeError(d['last_rejected']['reason'])
        time.sleep(.1)
    raise RuntimeError('No completion acknowledgement')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('joint',nargs='?');p.add_argument('delta',type=int,nargs='?',default=0);p.add_argument('--op',default='move',choices=['move','hold','stop']);a=p.parse_args();send(a.joint,a.delta,a.op)
