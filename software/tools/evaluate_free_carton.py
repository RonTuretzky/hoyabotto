"""Sensitivity tests, not measured material values or success-rate estimates."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

from carton.geometry import Box


CASES=[
    ('loaded-weak',dict(contents_mass=.96,table_friction=.7,stiffness=.008)),
    ('empty-weak',dict(stiffness=.008)),
    ('empty-resistant',{}),
    ('empty-slippery',dict(table_friction=.15)),
    ('loaded-resistant',dict(contents_mass=.96)),
    ('loaded-strong',dict(contents_mass=.96,stiffness=.04)),
    ('empty-unequal',dict(flap_stiffness=[.012,.024,.04,.018])),
    ('empty-heavy',dict(cardboard_mass=.45)),
    ('light-load',dict(contents_mass=.15)),
]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    p.add_argument('--video',action='store_true')
    a=p.parse_args();out=Path(a.out).resolve()
    if out.exists():raise ValueError('Output exists; preserve previous trials')
    out.mkdir(parents=True)
    b=Box();yaw=math.radians(30);dy=b.length/2*math.sin(yaw)+b.width/2*math.cos(yaw)-b.width/2
    base=dict(strategy='diagonal',base_height=.06,base_to_table_edge=.15,box_from_table_edge=.01,
              base_spacing=.30,table_tag_x=-.5,table_tag_y=.55,backup_table_tag_x=.45,backup_table_tag_y=.70,
              yaw=30,dy=dy,width=960,height=540,contents_mass=0.,table_friction=.35,stiffness=.018)
    rows=[]
    for name,changes in CASES:
        config={**base,**changes,'simulation_root':a.simulation_root,'out':str(out/name)}
        cmd=[sys.executable,str(Path(__file__).with_name('simulate_bimanual_folding.py'))]
        for key,value in config.items():
            cmd.append('--'+key.replace('_','-'))
            cmd.extend(str(v) for v in value) if isinstance(value,list) else cmd.append(str(value))
        if not a.video:cmd.append('--no-video')
        subprocess.run(cmd,check=True)
        result=json.loads((out/name/'result.json').read_text())
        rows.append({'case':name,'configuration':config,'success':result['success'],
                     'held_closure_passed':result['held_closure_passed'],'release_test':result['release_test'],
                     'error':result['controller'].get('error'),'carton_motion':result['physics']['carton_motion']})
        (out/'matrix.json').write_text(json.dumps({'parameters_measured':False,'cases':rows},indent=2))


if __name__=='__main__':main()
