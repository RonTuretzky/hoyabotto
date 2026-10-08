"""Matched offline claw/paddle trials. This does not talk to robot hardware."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

from carton.geometry import Box

CASES=[
    ('empty-resistant',dict()),
    ('empty-weak',dict(stiffness=.008)),
    ('loaded-resistant',dict(contents_mass=.96)),
    ('loaded-weak',dict(contents_mass=.96,table_friction=.7,stiffness=.008)),
]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    p.add_argument('--no-video',action='store_true')
    a=p.parse_args();out=Path(a.out).resolve()
    if out.exists():raise ValueError('Preserve previous trials: choose a new output directory')
    out.mkdir(parents=True)
    b=Box();yaw=math.radians(30)
    base=dict(strategy='diagonal',base_height=.06,base_to_table_edge=.15,box_from_table_edge=.01,
        base_spacing=.30,table_tag_x=-.5,table_tag_y=.55,backup_table_tag_x=.45,backup_table_tag_y=.70,
        initial_right_roll=1.5,
        yaw=30,dy=b.length/2*math.sin(yaw)+b.width/2*math.cos(yaw)-b.width/2,
        width=960,height=540,seed=1,contents_mass=0.,table_friction=.35,stiffness=.018,
        noise=.0008,dropout=.25,release_seconds=5.)
    configurations=[]
    for case,material in CASES:
        for tool in ('claws','paddle'):
            configurations.append((case+'-'+tool,{**base,**material,'tool':tool}))
    # Separate diagnostics: an ideal non-slipping attachment and grip sensitivity.
    for case,material in [CASES[0],CASES[-1]]:
        configurations.append((case+'-rigid-diagnostic',{**base,**material,'tool':'paddle','paddle_attachment':'rigid-diagnostic'}))
    for name,extra in [('zero-friction',dict(paddle_friction=0.)),('high-friction',dict(paddle_friction=1.2)),('light-paddle',dict(paddle_mass=.015))]:
        configurations.append(('grip-'+name,{**base,'tool':'paddle',**extra}))
    rows=[]
    for name,config in configurations:
        config={**config,'simulation_root':a.simulation_root,'out':str(out/name)}
        command=[sys.executable,str(Path(__file__).with_name('simulate_bimanual_folding.py'))]
        for key,value in config.items():command.extend(['--'+key.replace('_','-'),str(value)])
        if a.no_video:command.append('--no-video')
        subprocess.run(command,check=True)
        r=json.loads((out/name/'result.json').read_text())
        rows.append(dict(case=name,configuration=config,success=r['success'],held_closure_passed=r['held_closure_passed'],
            release_test=r['release_test'],error=r['controller'].get('error'),
            last_action=r['physics']['events'][-1]['label'],simulation_seconds=r['physics']['events'][-1]['time'],
            final_flap_degrees=r['independent_evaluation']['final_flap_degrees'],
            carton_motion=r['physics']['carton_motion'],paddle=r['physics'].get('paddle'),
            max_forbidden_penetration_mm=r['physics']['stats']['max_bad_penetration_mm']))
        (out/'comparison.json').write_text(json.dumps(dict(simulation_only=True,parameters_measured=False,
            scope='Same station, material, RGB-D noise, flap order and contact locations. Tool-aware blade orientation and TCP; no optimized bracing or retention strategy.',
            trials=rows),indent=2))


if __name__=='__main__':main()
